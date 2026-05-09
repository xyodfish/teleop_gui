#pragma once

/**
 * @file flex_ik.h
 * @brief FlexIk — multi end-effector IK (solveSingleStep + solve).
 */

#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <qpOASES.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "types.h"

namespace flex_ik {

class FlexIk {
   public:
    struct Options {
        double joint_damping = 1e-6;
        bool warm_start = true;
        int qp_max_ws = 100;
    };

    struct IterParams {
        int max_iterations = 100;
        double orientation_gain = 1.0;
        double error_tolerance = 1e-3;
        double dq_stop = 1e-6;
        double dq_gain = 1.0;
    };

    struct IkResult {
        bool success = false;
        bool converged = false;
        int iterations = 0;
        double final_error = 0.0;
        Vector solution;
    };

    explicit FlexIk(const pinocchio::Model& model, bool floating_base = false);
    FlexIk(const pinocchio::Model& model, Options options, bool floating_base = false);
    explicit FlexIk(const std::string& urdf_path, bool floating_base = false);
    FlexIk(const std::string& urdf_path, Options options, bool floating_base = false);

    FlexIk(const FlexIk&) = delete;
    FlexIk& operator=(const FlexIk&) = delete;
    FlexIk(FlexIk&&);
    FlexIk& operator=(FlexIk&&);
    virtual ~FlexIk();

    const pinocchio::Model& model() const { return model_; }
    const Options& options() const { return options_; }
    void setOptions(Options o);

    const Vector& jointWeight() const { return joint_weight_; }
    void setJointWeight(const Vector& joint_weight);
    void resetJointWeight();

    const IterParams& iterativeParams() const { return iter_params_; }
    void setIterativeParams(IterParams p);

    void addTask(const std::string& frame_name, double weight = 1.0);
    void addTask(const std::string& frame_name, const Vector6& mask, double weight = 1.0);

    void clearTasks();

    void updateTaskTarget(const std::string& frame_name, const SE3& target);
    void setTaskWeight(const std::string& frame_name, double weight);

    void setFixedJoints(const std::vector<std::string>& joint_names);

    const Vector& lowerPositionLimit() const { return q_lower_; }
    const Vector& upperPositionLimit() const { return q_upper_; }
    void setPositionLimits(const Vector& lower, const Vector& upper);
    void resetPositionLimitsFromModel();

    Vector solveSingleStep(const Vector& q, bool reset_ws = false);

    IkResult solve(const Vector& q);

   protected:
    using MatrixRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    struct FlexIkData {
        Matrix6x J_temp;
        std::unique_ptr<qpOASES::QProblemB> qp;
        MatrixRM H;
        Vector g;
        MatrixRM J;
        Vector b;
        Vector lb;
        Vector ub;
        Vector x_ws;
        Vector y_ws;
        bool has_ws = false;

        explicit FlexIkData(int nv_);
    };
    FlexIkData impl0_;

    struct IkConfig {
        pinocchio::FrameIndex frame_id;
        Vector6 mask = Vector6::Ones();
        double weight = 1.0;
        SE3 target;
        Vector6 target_twist;
    };

    double buildTwistsFromPoseTargets();
    virtual Vector iterateOnce(const Vector& q);

    pinocchio::Model model_;
    pinocchio::Data data_;
    bool is_floating_base_ = false;
    Size nq_ = 0;
    Size nv_ = 0;
    Size na_ = 0;
    Options options_;
    IterParams iter_params_;
    std::unordered_map<std::string, IkConfig> tasks_;
    int primary_rows_ = 0;
    std::vector<Index> locked_v_;
    Vector q_lower_;
    Vector q_upper_;
    Vector joint_weight_;
};

}  // namespace flex_ik
