#include <pinocchio/spatial/explog.hpp>
#include "wbc/math/helper_functions.h"

namespace Wbc {
namespace math {

Motion errorInSE3(const SE3& M_current, const SE3& M_desired) {
    pinocchio::SE3 M_err = M_current.inverse() * M_desired;
    Motion error;
    error.linear() = M_err.translation();
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

}  // namespace math
}  // namespace Wbc