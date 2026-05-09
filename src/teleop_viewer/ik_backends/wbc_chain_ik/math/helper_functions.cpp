#include "wbc/math/helper_functions.h"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>
namespace Wbc {
    namespace math {

        Motion errorInSE3(const SE3& M_current, const SE3& M_desired) {
            pinocchio::SE3 M_err = M_current.inverse() * M_desired;
            Motion error;
            error.linear()  = M_err.translation();
            error.angular() = pinocchio::log3(M_err.rotation());
            return error;
        }

        Vector A_pinv_b(const Matrix& A, const Vector& b, double damping) {
            Matrix AAt = A * A.transpose();
            AAt.diagonal().array() += damping;
            return A.transpose() * AAt.llt().solve(b);
        }

        Matrix3 skew(const Vector3& v) {
            Matrix3 result;
            result << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
            return result;
        }

        Matrix2 rot2D(double qz) {
            double c = std::cos(qz);
            double s = std::sin(qz);
            Matrix2 rot;
            rot << c, s, -s, c;
            return rot;
        }

        IkLinearization computeFrameIkLinearizationLocal(const pinocchio::Model& model, pinocchio::Data& data,
                                                         pinocchio::FrameIndex frame_id, const pinocchio::SE3& target_pose,
                                                         double max_pos_err, double max_rot_err) {
            IkLinearization out;

            const int nv = model.nv;

            const pinocchio::SE3& oMf   = data.oMf[frame_id];
            const pinocchio::SE3& oMdes = target_pose;

            // current^-1 * desired
            const pinocchio::SE3 iMd = oMf.actInv(oMdes);

            Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();

            // Limit translation error.
            const double pos_norm = err.head<3>().norm();
            if (max_pos_err > 0.0 && pos_norm > max_pos_err) {
                err.head<3>() *= max_pos_err / pos_norm;
            }

            // Limit rotation error.
            const double rot_norm = err.tail<3>().norm();
            if (max_rot_err > 0.0 && rot_norm > max_rot_err) {
                err.tail<3>() *= max_rot_err / rot_norm;
            }

            Eigen::MatrixXd J_local(6, nv);
            J_local.setZero();

            pinocchio::getFrameJacobian(model, data, frame_id, pinocchio::LOCAL, J_local);

            Eigen::Matrix<double, 6, 6> Jlog;
            pinocchio::Jlog6(iMd.inverse(), Jlog);

            out.J   = -Jlog * J_local;
            out.rhs = -err;
            out.err = err;

            return out;
        }

    }  // namespace math
}  // namespace Wbc