#pragma once

#include "types.h"

namespace flex_ik {
namespace utils {

inline Motion se3Error(const SE3& current, const SE3& target, double rot_weight = 1.0) {
    SE3 M_err = current.inverse() * target;
    Motion error;
    error.linear() = M_err.translation();
    error.angular() = rot_weight * pinocchio::log3(M_err.rotation());
    return error;
}

}  // namespace utils
}  // namespace flex_ik
