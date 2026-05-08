#pragma once
#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace Wbc {

using Scalar = double;
using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Matrix3x = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;
using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;
using Matrix6x = Eigen::Matrix<Scalar, 6, Eigen::Dynamic>;

using DiagMatrix = Eigen::DiagonalMatrix<Scalar, Eigen::Dynamic>;
using DiagMatrix6 = Eigen::DiagonalMatrix<Scalar, 6>;

using Index = std::size_t;
using Size = std::size_t;
using Uint = unsigned int;
static constexpr Index InvalidIndex = -1;

using Model = pinocchio::Model;
using Data = pinocchio::Data;
using SE3 = pinocchio::SE3;
using Motion = pinocchio::Motion;

}  // namespace Wbc
