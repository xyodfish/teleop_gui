#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "wbc/math/helper_functions.h"
#include "wbc/robot/traits/chain_ik.h"

namespace Wbc {
namespace robot {

ChainIkTrait::ChainIkTrait(const pinocchio::Model& model, const std::vector<Index>& fixed_joints)
    : model_(model), data_(model), nv(model.nv), fixed_joints(fixed_joints), hqp_solver() {
    hqp_data.reset(nv, 0, nv);
    J.setZero(6, nv);
    q_min = model.lowerPositionLimit;
    q_max = model.upperPositionLimit;
    joint_weights = Vector::Constant(nv, 1e-2);

    J_com.setZero(3, nv);
    pinocchio::centerOfMass(model_, data_, Vector::Zero(nv), Vector::Zero(nv), Vector::Zero(nv));
    com_ref = data_.com[0];
    com_weight.setConstant(1.0);
}

void ChainIkTrait::addIkTask(const std::string& frame_name, const SE3& target_pose, double weight,
                             const std::array<bool, 6>& mask) {
    tasks[frame_name] = IkConfig{model_.getFrameId(frame_name), target_pose, weight, std::make_optional(mask)};
    hqp_data.reset(nv, 6 * tasks.size(), nv);
}

IkResult ChainIkTrait::solveIK(const Vector& q_current, bool verbose) {
    auto time_start = std::chrono::high_resolution_clock::now();
    IkResult result;

    int count = 1;
    Vector q_out = q_current;
    bool converged = false;
    double err_sum = 0.0;
    Vector6 p_error;
    Vector delta_q(nv);

    while (count < max_iters) {
        hqp_data.reset(nv, 6 * tasks.size(), nv);  // set to zero
        auto& qp0 = hqp_data.qp0;
        auto& J0 = hqp_data.J0;
        auto& qp1 = hqp_data.qp1;
        err_sum = 0.0;
        pinocchio::computeJointJacobians(model_, data_, q_out);
        pinocchio::updateFramePlacements(model_, data_);

        int row_idx = 0;
        for (const auto& [frame_name, config] : tasks) {
            p_error = math::errorInSE3(data_.oMf[config.frame_id], config.target_pose).toVector();
            J.setZero(6, nv);
            pinocchio::getFrameJacobian(model_, data_, config.frame_id, pinocchio::LOCAL, J);

            if (config.mask.has_value()) {
                for (int i = 0; i < 6; i++) {
                    if (!config.mask->at(i)) {
                        p_error(i) = 0.0;
                        J.row(i).setZero();
                    }
                }
            }
            qp0.H += config.weight * (J.transpose() * J);
            qp0.g += -config.weight * (J.transpose() * p_error);
            J0.middleRows(row_idx, 6) = J;
            row_idx += 6;
            err_sum += p_error.squaredNorm();
        }
        if (std::sqrt(err_sum) < tol) {
            result.solution = q_out;
            result.success = true;
            result.converged = true;
            result.time_spent =
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start).count();
            if (verbose) {
                std::cout << "IK solved in " << count - 1 << " iterations with error " << std::sqrt(err_sum)
                          << " and time " << result.time_spent * 1000.0 << " ms." << std::endl;
            }
            return result;
        } else if (converged) {
            result.solution = q_out;
            result.success = false;
            result.converged = true;
            result.time_spent =
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start).count();
            if (verbose) {
                std::cout << "IK converged but not solved in " << count << " iterations with error "
                          << std::sqrt(err_sum) << " and time " << result.time_spent * 1000.0 << " ms." << std::endl;
            }
            return result;
        }

        // q_min - q_out < dq < q_max - q_out
        qp0.CI.setIdentity();
        qp0.ci_lb.noalias() = q_min - q_out;
        qp0.ci_ub.noalias() = q_max - q_out;
        // Zero velocity for fixed joints
        for (const auto& idx : fixed_joints) {
            qp0.ci_lb(idx) = 0.0;
            qp0.ci_ub(idx) = 0.0;
        }

        // Secondary objective: minimize joint motion (delta_q)
        for (int i = 0; i < nv; i++) {
            qp1.H(i, i) += joint_weights(i);
        }

        // addComCost(qp1, q_out);

        const auto& qp_output = hqp_solver.solve(hqp_data);

        if (qp_output.status != solver::QPStatus::OPTIMAL) {
            result.success = false;
            result.converged = false;
            result.time_spent =
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start).count();
            if (verbose) {
                std::cout << "IK failed at iteration " << count << " with status " << static_cast<int>(qp_output.status)
                          << " and time " << result.time_spent * 1000.0 << " ms." << std::endl;
            }
            return result;
        }

        delta_q = qp_output.x;

        q_out += delta_q;
        converged = delta_q.lpNorm<Eigen::Infinity>() < 1e-8;

        if (verbose) {
            std::cout << "Iteration " << count << ": error = " << std::sqrt(err_sum)
                      << ", delta_q norm = " << delta_q.norm() << std::endl;
        }

        count++;
    }
    result.success = false;
    result.converged = converged;
    result.time_spent = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start).count();
    if (verbose) {
        std::cout << "IK not solved after " << max_iters << " iterations. Final error: " << std::sqrt(err_sum)
                  << " and time " << result.time_spent * 1000.0 << " ms." << std::endl;
    }
    return result;
}

void ChainIkTrait::addComCost(solver::QPData& qp, const Vector& q) {
    pinocchio::centerOfMass(model_, data_, q);
    J_com = pinocchio::jacobianCenterOfMass(model_, data_, q, false);
    qp.H += J_com.transpose() * com_weight.asDiagonal() * J_com;
    qp.g += -(J_com.transpose() * com_weight.asDiagonal() * (com_ref - data_.com[0]));
    // std::cout << "com ref: " << com_ref.transpose() << ", com current: " << data_.com[0].transpose() << std::endl;
}

}  // namespace robot
}  // namespace Wbc