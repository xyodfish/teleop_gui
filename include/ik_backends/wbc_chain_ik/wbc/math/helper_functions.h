#pragma once

#include "wbc/math/types.h"

namespace Wbc {
    namespace math {

        Motion errorInSE3(const SE3& M_desired, const SE3& M_current);

        Vector A_pinv_b(const Matrix& A, const Vector& B, double damping = 1e-6);

        Matrix3 skew(const Vector3& v);

        Matrix2 rot2D(double qz);

        struct IkLinearization {
            Eigen::Matrix<double, 6, 1> rhs;
            Eigen::MatrixXd J;
            Eigen::Matrix<double, 6, 1> err;
        };

        IkLinearization computeFrameIkLinearizationLocal(const pinocchio::Model& model, pinocchio::Data& data,
                                                         pinocchio::FrameIndex frame_id, const pinocchio::SE3& target_pose,
                                                         double max_pos_err = 0.03, double max_rot_err = 0.05);

    }  // namespace math
}  // namespace Wbc