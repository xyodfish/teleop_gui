#pragma once

#include "wbc/math/types.h"

namespace Wbc {
namespace math {

Motion errorInSE3(const SE3& M_desired, const SE3& M_current);

Vector A_pinv_b(const Matrix& A, const Vector& B, double damping = 1e-6);

Matrix3 skew(const Vector3& v);

Matrix2 rot2D(double qz);

}  // namespace math
}  // namespace Wbc