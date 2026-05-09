#pragma once

#include <memory>
#include <qpOASES.hpp>
#include "wbc/solver/qp_data.h"

namespace Wbc {
namespace solver {

class QPSolver {
   public:
    QPSolver();
    ~QPSolver() = default;

    /**
     * @brief Solve the QP problem defined by QPData
     * minimize 0.5 * x^T * H * x + g^T * x
     * subject to
     * CE * x = ce0 (equality constraints - handled as bounds in qpoases or specific constraints)
     * ci_lb <= CI * x <= ci_ub
     *
     * Note: qpOASES formulation is:
     * min 0.5 * x^T * H * x + g^T * x
     * s.t. lb <= x <= ub (bounds on variables)
     *      lbA <= A * x <= ubA (constraints)
     *
     * Our QPData:
     * H, g
     * CE, ce0 -> Equality constraints convert to lbA = ubA = ce0, A top part = CE.
     * CI, ci_lb, ci_ub -> Inequality constraints.
     *
     * We need to stack CE and CI into A.
     */
    const QPOutput& solve(const QPData& qp_data);

   private:
    std::shared_ptr<qpOASES::SQProblem> solver_;
    qpOASES::Options options_;

    bool initialized_ = false;
    int nv_ = 0;
    int nc_ = 0;

    QPOutput output;
};

}  // namespace solver
}  // namespace Wbc
