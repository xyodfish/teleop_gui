#pragma once

#include "wbc/math/types.h"

#include <iostream>

namespace Wbc {
namespace solver {

struct QPData {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H;
    Vector g;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> CI;
    Vector ci_lb;
    Vector ci_ub;

    void reset(int n_var, int n_in) {
        H.setZero(n_var, n_var);
        g.setZero(n_var);
        CI.setZero(n_in, n_var);
        ci_lb.setZero(n_in);
        ci_ub.setZero(n_in);
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "H:\n" << H << "\n";
        oss << "g:\n" << g.transpose() << "\n";
        oss << "CI:\n" << CI << "\n";
        oss << "ci_lb:\n" << ci_lb.transpose() << "\n";
        oss << "ci_ub:\n" << ci_ub.transpose() << "\n";
        return oss.str();
    }
};

struct HQPData {
    QPData qp0, qp1;
    Matrix J0;

    void reset(int n_var, int n_eq_0, int n_in_0, int n_in_1 = 0) {
        qp0.reset(n_var, n_in_0);
        qp1.reset(n_var, n_in_1 + n_in_0 + n_eq_0);  // +n_eq_0 for equality constraints from qp0
        J0.setZero(n_eq_0, n_var);
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "First QP:\n" << qp0.toString();
        oss << "Second QP:\n" << qp1.toString();
        return oss.str();
    }
};

// clang-format off
enum class QPStatus {
    UNKNOWN             = -1,
    OPTIMAL             =  0,
    INFEASIBLE          =  1,
    UNBOUNDED           =  2,
    MAX_ITER_REACHED    =  3,
    ERROR               =  4
};
// clang-format on

class QPOutput {
   public:
    QPStatus status;
    Vector x;
    Vector lambda;
    Eigen::VectorXi activeSet;
    int iterations;

    QPOutput() {}

    QPOutput(unsigned int nVar, unsigned int nEq, unsigned int nIn) { resize(nVar, nEq, nIn); }

    void resize(unsigned int nVar, unsigned int nEq, unsigned int nIn) {
        x.resize(nVar);
        lambda.resize(nEq + nIn);
        activeSet.resize(nIn);
    }
};

}  // namespace solver
}  // namespace Wbc
