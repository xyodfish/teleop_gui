#include "wbc/solver/hqp_solver.h"

namespace Wbc {
namespace solver {

HQPSolver::HQPSolver() = default;

const QPOutput& HQPSolver::solve(HQPData& hqp_data) {
    auto& qp0 = hqp_data.qp0;
    auto& J0 = hqp_data.J0;
    auto& qp1 = hqp_data.qp1;
    int nVar = qp0.H.rows();
    int nEq0 = hqp_data.J0.rows();
    int nIn0 = qp0.CI.rows();
    int nIn1 = qp1.CI.rows() - nIn0 - nEq0;

    const QPOutput& sol0 = qp0_solver_.solve(qp0);

    // Solve first level qp
    if (sol0.status != QPStatus::OPTIMAL) {
        output.status = sol0.status;
        output.x.resize(nVar);
        output.x.setZero();
        output.iterations = sol0.iterations;
        return output;
    }

    if (qp0.H.determinant() > 1e-8) {
        // No feasible solution for level 1 since level 0 is strictly convex and has a unique solution
        output.status = QPStatus::OPTIMAL;
        output.x = sol0.x;  // Return level 0 solution anyway
        output.iterations = sol0.iterations;
        return output;
    }

    // Construct level 1 problem with level 0 solution as equality constraint
    qp1.CI.middleRows(nIn1, nIn0) = qp0.CI;
    qp1.ci_lb.middleRows(nIn1, nIn0) = qp0.ci_lb;
    qp1.ci_ub.middleRows(nIn1, nIn0) = qp0.ci_ub;
    qp1.CI.bottomRows(nEq0) = J0;
    qp1.ci_lb.tail(nEq0) = J0 * sol0.x;
    qp1.ci_ub.tail(nEq0) = J0 * sol0.x;
    
    const QPOutput& sol1 = qp1_solver_.solve(qp1);

    output.status = sol1.status;
    output.x = sol1.x;
    output.lambda = sol1.lambda;
    output.activeSet = sol1.activeSet;
    output.iterations = sol0.iterations + sol1.iterations;

    return output;
}

}  // namespace solver
}  // namespace Wbc
