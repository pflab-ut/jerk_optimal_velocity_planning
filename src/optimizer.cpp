#include "optimizer.h"

Optimizer::Optimizer(const OptimizerSolver& solver_num, const BaseSolver::OptimizerParam& param)
{
    if(solver_num == GUROBI_QP)
        solver_ = std::make_shared<gurobi::QPSolver>(param);
    else if(solver_num == GUROBI_LP)
        solver_ = std::make_shared<gurobi::LPSolver>(param);
    else if(solver_num == OSQP_QP)
        solver_ = std::make_shared<osqp::QPSolver>(param);
    else if(solver_num == OSQP_LP)
        solver_ = std::make_shared<osqp::LPSolver>(param);
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
    bool is_success = false;
    if(is_hard)
        is_success = solver_->solveHard(initial_vel, initial_acc, ds, ref_vels, max_vels, output);
    else
        is_success = solver_->solveSoft(initial_vel, initial_acc, ds, ref_vels, max_vels, output);

    // Compute Time
    if(!is_success)
        return is_success;
    else
        computeTime(ds, output);

    return true;
}

bool Optimizer::solvePseudo(const bool& is_hard,
                            const double& initial_vel,
                            const double& initial_acc,
                            const double& ds,
                            const std::vector<double>& ref_vels,
                            const std::vector<double>& max_vels,
                            BaseSolver::OutputInfo& output)
{
    bool is_success = false;
    if(is_hard)
        is_success = solver_->solveHardPseudo(initial_vel, initial_acc, ds, ref_vels, max_vels, output);
    else
        is_success = solver_->solveSoftPseudo(initial_vel, initial_acc, ds, ref_vels, max_vels, output);

    // Compute Time
    if(!is_success)
        return is_success;
    else
        computeTime(ds, output);

    return true;
}

void Optimizer::computeTime(const double& ds, BaseSolver::OutputInfo& output)
{
    double t = 0.0;
    output.time.front() = t;
    for(int i=1; i<output.velocity.size(); ++i)
    {
        if(output.velocity[i] < 1e-6)
        {
            t += 0.1;
            output.position[i] = output.position[i-1];
        }
        else
            t += ds/output.velocity[i];
        output.time[i] = t;
    }
}
