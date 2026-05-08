#include "wbc/solver/qp_solver.h"

namespace Wbc {
namespace solver {

QPSolver::QPSolver() {
    options_.setToMPC();                     // Fast default options
    options_.printLevel = qpOASES::PL_NONE;  // Suppress output
    options_.enableEqualities = qpOASES::BT_TRUE;
    options_.enableRegularisation = qpOASES::BT_TRUE;
    options_.epsRegularisation = 1e-8;
}

const QPOutput& QPSolver::solve(const QPData& qp_data) {
    int nv = qp_data.H.rows();
    int nc = qp_data.CI.rows();

    output.resize(nv, 0, nc);

    // Resize internal storage if needed or if first run
    if (!initialized_ || nv_ != nv || nc_ != nc) {
        solver_ = std::make_shared<qpOASES::SQProblem>(nv, nc);
        solver_->setOptions(options_);
        initialized_ = true;
        nv_ = nv;
        nc_ = nc;
    }

    int nWSR = 100;  // Max working set recalculations
    qpOASES::returnValue ret;

    // if (solver_->isInitialised()) {
    if (false) {
        ret = solver_->hotstart(qp_data.H.data(), qp_data.g.data(), qp_data.CI.data(), nullptr, nullptr,
                                qp_data.ci_lb.data(), qp_data.ci_ub.data(), nWSR);
    } else {
        ret = solver_->init(qp_data.H.data(), qp_data.g.data(), qp_data.CI.data(), nullptr, nullptr,
                            qp_data.ci_lb.data(), qp_data.ci_ub.data(), nWSR);
    }

    if (ret == qpOASES::SUCCESSFUL_RETURN) {
        output.status = QPStatus::OPTIMAL;
        output.x.resize(nv);
        solver_->getPrimalSolution(output.x.data());
        output.iterations = 1000 - nWSR;  // Estimated
    } else {
        output.status = QPStatus::ERROR;
        // Map qpOASES return codes to QPStatus if needed
        if (ret == qpOASES::RET_MAX_NWSR_REACHED) output.status = QPStatus::MAX_ITER_REACHED;
        if (ret == qpOASES::RET_INIT_FAILED_INFEASIBILITY) output.status = QPStatus::INFEASIBLE;
    }

    return output;
}

}  // namespace solver
}  // namespace Wbc
