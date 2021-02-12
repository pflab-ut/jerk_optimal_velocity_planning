#ifndef FILTER_POSITION_OPTIMIZATION_OPTIMIZER_H
#define FILTER_POSITION_OPTIMIZATION_OPTIMIZER_H

#include "solver/base_solver.h"
#include "solver/lp_solver_gurobi.h"
#include "solver/qp_solver_gurobi.h"
#include "solver/nc_solver_nlopt.h"
#include <memory>

class Optimizer
{
public:
    enum OptimizerSolver
    {
        GUROBI_LP = 0,
        GUROBI_QP = 1,
        NLOPT_NC = 2,
    };

    Optimizer(const OptimizerSolver& solver_num, const BaseSolver::OptimizerParam& param);

    bool solve(const bool& is_hard,
               const double& initial_vel,
               const double& initial_acc,
               const double& ds,
               const std::vector<double>& ref_vels,
               const std::vector<double>& max_vels,
               BaseSolver::OutputInfo& output);

    bool solvePseudo(const bool& is_hard,
                     const double& initial_vel,
                     const double& initial_acc,
                     const double& ds,
                     const std::vector<double>& ref_vels,
                     const std::vector<double>& max_vels,
                     BaseSolver::OutputInfo& output);

private:
    std::shared_ptr<BaseSolver> solver_;
};

#endif //FILTER_POSITION_OPTIMIZATION_OPTIMIZER_H
