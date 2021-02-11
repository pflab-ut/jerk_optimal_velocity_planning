#include "optimizer.h"

Optimizer::Optimizer(const OptimizerSolver& solver_num, const BaseSolver::OptimizerParam& param)
{
    if(solver_num == GUROBI_QP)
        solver_ = std::make_shared<gurobi::QPSolver>(param);
    else if(solver_num == OSQP_QP)
        solver_ = std::make_shared<osqp::QPSolver>(param);
}

bool Optimizer::solve(const double &initial_vel,
                      const double &initial_acc,
                      const double &ds,
                      const std::vector<double> &ref_vels,
                      const std::vector<double> &max_vels,
                      BaseSolver::OutputInfo &output)
{
    return solver_->solve(initial_vel, initial_acc, ds, ref_vels, max_vels, output);
}

bool Optimizer::solve(const double &initial_vel,
                      const double &initial_acc,
                      const double &ds,
                      const std::vector<double> &max_vels,
                      BaseSolver::OutputInfo &output)
{
    return solver_->solve(initial_vel, initial_acc, ds, max_vels, output);
}