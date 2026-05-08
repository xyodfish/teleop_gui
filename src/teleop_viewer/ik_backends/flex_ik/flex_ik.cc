#include "flex_ik.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "utils.h"

namespace flex_ik {

namespace {

pinocchio::Model buildModel(const pinocchio::Model& model, bool want_floating_base) {
    if (!want_floating_base || model.nq != model.nv) {
        return model;
    }
    pinocchio::Model root;
    static constexpr const char* kFfJointName = "floating_base";
    (void)root.addJoint(0, pinocchio::JointModelFreeFlyer(), SE3::Identity(), kFfJointName);
    if (!root.existFrame(kFfJointName)) {
        throw std::runtime_error("FlexIk: failed to add free-flyer joint frame");
    }
    const pinocchio::FrameIndex attach_fid = root.getFrameId(kFfJointName);
    return pinocchio::appendModel(root, model, attach_fid, SE3::Identity());
}

}  // namespace

FlexIk::FlexIk(const pinocchio::Model& model, bool floating_base) : FlexIk(model, Options{}, floating_base) {}

FlexIk::FlexIk(const std::string& urdf_path, bool floating_base) : FlexIk(urdf_path, Options{}, floating_base) {}

FlexIk::FlexIk(const pinocchio::Model& model, Options options, bool floating_base)
    : impl0_(floating_base ? 6 + model.nv : model.nv),
      model_(buildModel(model, floating_base)),
      is_floating_base_(floating_base),
      options_(std::move(options)) {
    data_ = pinocchio::Data(model_);
    nq_ = static_cast<std::size_t>(model_.nq);
    nv_ = static_cast<std::size_t>(model_.nv);
    na_ = nv_ - (is_floating_base_ ? 6 : 0);
    resetJointWeight();
    setOptions(options_);
    resetPositionLimitsFromModel();
}

FlexIk::FlexIk(const std::string& urdf_path, Options options, bool floating_base)
    : FlexIk(
          [urdf_path]() {
              pinocchio::Model model;
              pinocchio::urdf::buildModel(urdf_path, model);
              return model;
          }(),
          std::move(options), floating_base) {}

FlexIk::~FlexIk() = default;

FlexIk::FlexIk(FlexIk&&) = default;
FlexIk& FlexIk::operator=(FlexIk&&) = default;

void FlexIk::setOptions(Options o) {
    if (o.joint_damping < 0.0) {
        throw std::invalid_argument("FlexIk::setOptions: joint_damping must be non-negative");
    }
    if (o.qp_max_ws <= 0) {
        throw std::invalid_argument("FlexIk::setOptions: qp_max_ws must be positive");
    }
    options_ = std::move(o);
}

void FlexIk::setJointWeight(const Vector& joint_weight) {
    if ((Size)joint_weight.size() != na_) {
        throw std::invalid_argument("FlexIk::setJointWeight: size must equal " + std::to_string(na_));
    }

    if (!joint_weight.allFinite()) {
        throw std::invalid_argument("FlexIk::setJointWeight: values must be finite");
    }
    if ((joint_weight.array() <= 0.0).any()) {
        throw std::invalid_argument("FlexIk::setJointWeight: values must be positive");
    }

    joint_weight_.tail(na_) = joint_weight.normalized() * std::sqrt(static_cast<double>(na_));
}

void FlexIk::resetJointWeight() { joint_weight_ = Vector::Ones(nv_); }

void FlexIk::setIterativeParams(IterParams p) {
    if (p.max_iterations <= 0) {
        throw std::invalid_argument("FlexIk::setIterativeParams: max_iterations must be positive");
    }
    if (p.orientation_gain <= 0.0) {
        throw std::invalid_argument("FlexIk::setIterativeParams: orientation_gain must be positive");
    }
    if (p.error_tolerance <= 0.0) {
        throw std::invalid_argument("FlexIk::setIterativeParams: error_tolerance must be positive");
    }
    if (p.dq_stop < 0.0) {
        throw std::invalid_argument("FlexIk::setIterativeParams: dq_stop must be non-negative");
    }
    if (p.dq_gain <= 0.0) {
        throw std::invalid_argument("FlexIk::setIterativeParams: dq_gain must be positive");
    }
    iter_params_ = std::move(p);
}

void FlexIk::addTask(const std::string& frame_name, double weight) { addTask(frame_name, Vector6::Ones(), weight); }

void FlexIk::addTask(const std::string& frame_name, const Vector6& mask, double weight) {
    if (!model_.existFrame(frame_name)) {
        throw std::invalid_argument("FlexIk::addTask: frame '" + frame_name + "' not in model");
    }
    if (weight <= 0.0) {
        throw std::invalid_argument("FlexIk::addTask: weight must be positive");
    }
    if (tasks_.find(frame_name) != tasks_.end()) {
        throw std::invalid_argument("FlexIk::addTask: task for frame '" + frame_name + "' already exists");
    }
    if ((mask.array() < 0.0).any()) {
        throw std::invalid_argument("FlexIk::addTask: mask values must be non-negative");
    }
    IkConfig cfg;
    cfg.frame_id = model_.getFrameId(frame_name);
    cfg.mask = mask;
    cfg.weight = weight;
    primary_rows_ += (int)(mask.array() > 0.0).count();
    tasks_[frame_name] = std::move(cfg);
}

void FlexIk::clearTasks() {
    tasks_.clear();
    primary_rows_ = 0;
}

void FlexIk::updateTaskTarget(const std::string& frame_name, const SE3& target) {
    auto it = tasks_.find(frame_name);
    if (it == tasks_.end()) {
        throw std::invalid_argument("FlexIk::updateTaskTarget: no task for frame '" + frame_name + "'");
    }
    it->second.target = target;
}

void FlexIk::setTaskWeight(const std::string& frame_name, double weight) {
    if (weight <= 0.0) {
        throw std::invalid_argument("FlexIk::setTaskWeight: weight must be positive");
    }
    auto it = tasks_.find(frame_name);
    if (it == tasks_.end()) {
        throw std::invalid_argument("FlexIk::setTaskWeight: no task for frame '" + frame_name + "'");
    }
    it->second.weight = weight;
}

void FlexIk::setFixedJoints(const std::vector<std::string>& joint_names) {
    const std::size_t nv = nv_;
    std::set<Index> vidx;
    for (const auto& jn : joint_names) {
        if (!model_.existJointName(jn)) {
            throw std::invalid_argument("FlexIk::setFixedJoints: joint '" + jn + "' not in model");
        }
        const pinocchio::JointIndex jid = model_.getJointId(jn);
        const auto& joint = model_.joints[jid];
        const int i0 = joint.idx_v();
        for (int k = 0; k < joint.nv(); ++k) {
            const Index t = static_cast<Index>(i0 + k);
            if (t >= nv) {
                throw std::invalid_argument("FlexIk::setFixedJoints: tangent index out of range");
            }
            vidx.insert(t);
        }
    }
    locked_v_.assign(vidx.begin(), vidx.end());
}

void FlexIk::resetPositionLimitsFromModel() {
    q_lower_ = model_.lowerPositionLimit.tail(na_);
    q_upper_ = model_.upperPositionLimit.tail(na_);
    for (Eigen::Index i = 0; i < q_lower_.size(); ++i) {
        if (q_lower_(i) > q_upper_(i)) {
            throw std::invalid_argument("FlexIk::resetPositionLimitsFromModel: model limit lower > upper");
        }
    }
}

void FlexIk::setPositionLimits(const Vector& lower, const Vector& upper) {
    if ((Size)lower.size() != na_ || (Size)upper.size() != na_) {
        throw std::invalid_argument("FlexIk::setPositionLimits: size != " + std::to_string(na_));
    }
    for (Eigen::Index i = 0; i < lower.size(); ++i) {
        if (lower(i) > upper(i)) {
            throw std::invalid_argument("FlexIk::setPositionLimits: lower > upper");
        }
    }
    q_lower_ = lower.cwiseMax(model_.lowerPositionLimit.tail(na_));
    q_upper_ = upper.cwiseMin(model_.upperPositionLimit.tail(na_));
}

double FlexIk::buildTwistsFromPoseTargets() {
    double sum_err = 0.0;
    for (auto& kv : tasks_) {
        IkConfig& cfg = kv.second;
        const auto e = utils::se3Error(data_.oMf[cfg.frame_id], cfg.target, iter_params_.orientation_gain);
        cfg.target_twist = cfg.mask.cwiseProduct(e.toVector());
        sum_err += cfg.target_twist.squaredNorm();
    }
    return std::sqrt(sum_err);
}

Vector FlexIk::iterateOnce(const Vector& q) {
    if (q.size() != static_cast<Eigen::Index>(nq_)) {
        throw std::invalid_argument("FlexIk::iterateOnce: q size != " + std::to_string(nq_));
    }
    if (tasks_.empty()) {
        throw std::runtime_error("FlexIk::iterateOnce: no primary tasks defined");
    }

    impl0_.J.resize(primary_rows_, nv_);
    impl0_.b.resize(primary_rows_);
    int row_cursor = 0;
    for (const auto& kv : tasks_) {
        const IkConfig& cfg = kv.second;
        impl0_.J_temp.setZero();
        pinocchio::getFrameJacobian(model_, data_, cfg.frame_id, pinocchio::LOCAL, impl0_.J_temp);
        for (int i = 0; i < 6; ++i) {
            const double w = cfg.weight * cfg.mask(i);
            if (w <= 0.0) continue;
            const auto Ji = impl0_.J_temp.row(i);
            const double sqrt_w = std::sqrt(w);
            impl0_.J.row(row_cursor) = sqrt_w * Ji;
            impl0_.b(row_cursor) = sqrt_w * cfg.target_twist(i);
            ++row_cursor;
        }
    }
    impl0_.H.noalias() = impl0_.J.transpose() * impl0_.J;
    impl0_.H.diagonal().array() += options_.joint_damping * joint_weight_.array();
    impl0_.g.noalias() = -impl0_.J.transpose() * impl0_.b;
    impl0_.lb.tail(na_) = q_lower_ - q.tail(na_);
    impl0_.ub.tail(na_) = q_upper_ - q.tail(na_);

    for (Index vidx : locked_v_) {
        impl0_.lb[vidx] = 0;
        impl0_.ub[vidx] = 0;
    }

    qpOASES::int_t nWSR = options_.qp_max_ws;
    const bool use_ws = options_.warm_start && impl0_.has_ws;

    auto ret =
        impl0_.qp->init(impl0_.H.data(), impl0_.g.data(), impl0_.lb.data(), impl0_.ub.data(), nWSR, nullptr,
                        use_ws ? impl0_.x_ws.data() : nullptr, use_ws ? impl0_.y_ws.data() : nullptr, nullptr, nullptr);
    if (ret != qpOASES::SUCCESSFUL_RETURN && ret != qpOASES::RET_MAX_NWSR_REACHED) {
        if (use_ws) {
            nWSR = options_.qp_max_ws;
            ret = impl0_.qp->init(impl0_.H.data(), impl0_.g.data(), impl0_.lb.data(), impl0_.ub.data(), nWSR, nullptr,
                                  nullptr, nullptr, nullptr, nullptr);
        }
        if (ret != qpOASES::SUCCESSFUL_RETURN && ret != qpOASES::RET_MAX_NWSR_REACHED) {
            throw std::runtime_error("FlexIk::solveSingleStep: qpOASES init failed");
        }
    }

    impl0_.qp->getPrimalSolution(impl0_.x_ws.data());
    impl0_.qp->getDualSolution(impl0_.y_ws.data());
    impl0_.has_ws = true;

    return impl0_.x_ws;
}

Vector FlexIk::solveSingleStep(const Vector& q, bool reset_ws) {
    if (tasks_.empty()) {
        throw std::runtime_error("FlexIk::solveSingleStep: no tasks defined");
    }
    if (q.size() != static_cast<Eigen::Index>(nq_)) {
        throw std::invalid_argument("FlexIk::solveSingleStep: q size != " + std::to_string(nq_));
    }

    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    buildTwistsFromPoseTargets();
    if (reset_ws) impl0_.has_ws = false;
    return iterateOnce(q);
}

FlexIk::IkResult FlexIk::solve(const Vector& q) {
    if (tasks_.empty()) {
        throw std::runtime_error("FlexIk::solve: no tasks defined");
    }
    if (q.size() != static_cast<Eigen::Index>(nq_)) {
        throw std::invalid_argument("FlexIk::solve: q size != " + std::to_string(nq_));
    }

    IkResult out;
    Vector& q_iter = out.solution;
    q_iter = q;
    Vector dq(nv_);
    impl0_.has_ws = false;

    for (int iter = 0; iter < iter_params_.max_iterations; ++iter) {
        pinocchio::computeJointJacobians(model_, data_, q_iter);
        pinocchio::updateFramePlacements(model_, data_);
        const double err = buildTwistsFromPoseTargets();
        out.final_error = err;
        out.iterations = iter + 1;

        if (err < iter_params_.error_tolerance) {
            out.success = true;
            out.converged = true;
            return out;
        }

        try {
            dq = iterateOnce(q_iter);
        } catch (const std::exception&) {
            out.success = false;
            out.converged = false;
            return out;
        }

        if (dq.lpNorm<Eigen::Infinity>() < iter_params_.dq_stop) {
            out.success = false;
            out.converged = true;
            return out;
        }

        q_iter = pinocchio::integrate(model_, q_iter, dq * iter_params_.dq_gain);
    }

    out.success = false;
    out.converged = false;
    return out;
}

FlexIk::FlexIkData::FlexIkData(int nv_) {
    qp = std::make_unique<qpOASES::QProblemB>(nv_, qpOASES::HST_UNKNOWN);
    qpOASES::Options qp_opts;
    qp_opts.setToMPC();
    qp_opts.printLevel = qpOASES::PL_NONE;
    qp_opts.enableEqualities = qpOASES::BT_TRUE;
    qp_opts.enableRegularisation = qpOASES::BT_TRUE;
    qp_opts.epsRegularisation = 1e-8;
    qp->setOptions(qp_opts);
    J_temp.setZero(6, nv_);
    H.setZero(nv_, nv_);
    g.setZero(nv_);
    lb.setConstant(nv_, -1e10);
    ub.setConstant(nv_, 1e10);
    x_ws.setZero(nv_);
    y_ws.setZero(nv_);
}

}  // namespace flex_ik
