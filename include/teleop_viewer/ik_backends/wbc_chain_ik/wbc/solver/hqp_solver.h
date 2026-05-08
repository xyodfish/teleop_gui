#pragma once

#include <memory>
#include "wbc/solver/qp_data.h"
#include "wbc/solver/qp_solver.h"

namespace Wbc {
namespace solver {

/**
 * @brief Hierarchical QP Solver
 *
 * Solves a two-level QP problem:
 * Level 0 (high priority): min 0.5 * x^T * H0 * x + g0^T * x
 * Level 1 (low priority): min 0.5 * x^T * H1 * x + g1^T * x
 *
 * With constraints: ci_lb <= CI * x <= ci_ub
 *
 * The solver first solves level 0, then uses the solution as an equality constraint
 * for level 1 to ensure the higher priority objective is satisfied.
 */
class HQPSolver {
   public:
    HQPSolver();
    ~HQPSolver() = default;

    /**
     * @brief Solve the HQP problem
     * @param hqp_data Contains both level 0 (qp0) and level 1 (qp1) objectives
     * @return const QPOutput& Solution with status, primal variables, and multipliers
     */
    const QPOutput& solve(HQPData& hqp_data);

   private:
    QPSolver qp0_solver_, qp1_solver_;

    QPOutput output;
};

}  // namespace solver
}  // namespace Wbc
