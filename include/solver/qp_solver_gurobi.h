#ifndef FILTER_POSITION_OPTIMIZATION_QP_SOLVER_GUROBI_H
#define FILTER_POSITION_OPTIMIZATION_QP_SOLVER_GUROBI_H

#include "base_solver.h"

namespace gurobi
{
    class QPSolver: public BaseSolver
    {
    public:
        QPSolver(const OptimizerParam& param) : BaseSolver(param) {}

        bool solve(const double& initial_vel,
                   const double& initial_acc,
                   const double& ds,
                   const std::vector<double>& ref_vels,
                   const std::vector<double>& max_vels,
                   OutputInfo& output);

        bool solve(const double& initial_vel,
                   const double& initial_acc,
                   const double& ds,
                   const std::vector<double>& max_vels,
                   OutputInfo& output);

    };
}


#endif //FILTER_POSITION_OPTIMIZATION_QP_SOLVER_GUROBI_H
