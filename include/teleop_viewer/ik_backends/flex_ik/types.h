#pragma once

#include <eigen3/Eigen/Dense>
#include <functional>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <utility>
#include <vector>

namespace flex_ik {

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;
using SE3 = pinocchio::SE3;
using Motion = pinocchio::Motion;

using Vector3 = Eigen::Vector3d;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6x = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using Index = std::size_t;
using Size = std::size_t;

using CostFunction =
    std::function<std::pair<Matrix, Vector>(const pinocchio::Model&, const pinocchio::Data&, const Vector&)>;

}  // namespace flex_ik
