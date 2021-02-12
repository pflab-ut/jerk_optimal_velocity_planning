#include "optimizer.h"

Optimizer::Optimizer(const OptimizerSolver& solver_num, const BaseSolver::OptimizerParam& param)
{
    if(solver_num == GUROBI_QP)
        solver_ = std::make_shared<gurobi::QPSolver>(param);
    else if(solver_num == GUROBI_LP)
        solver_ = std::make_shared<gurobi::LPSolver>(param);
    else if(solver_num == NLOPT_NC)
        solver_ = std::make_shared<nlopt::NCSolver>(param);
}

bool Optimizer::solve(const bool& is_hard,
                      const double& initial_vel,
                      const double& initial_acc,
                      const double& ds,
                      const std::vector<double>& ref_vels,
                      const std::vector<double>& max_vels,
                      BaseSolver::OutputInfo& output)
{
    if(is_hard)
        return solver_->solveHard(initial_vel, initial_acc, ds, ref_vels, max_vels, output);
    else
        return solver_->solveSoft(initial_vel, initial_acc, ds, ref_vels, max_vels, output);
}

bool Optimizer::solvePseudo(const bool& is_hard,
                            const double& initial_vel,
                            const double& initial_acc,
                            const double& ds,
                            const std::vector<double>& ref_vels,
                            const std::vector<double>& max_vels,
                            BaseSolver::OutputInfo& output)
{
    if(is_hard)
        return solver_->solveHardPseudo(initial_vel, initial_acc, ds, ref_vels, max_vels, output);
    else
        return solver_->solveSoftPseudo(initial_vel, initial_acc, ds, ref_vels, max_vels, output);
}