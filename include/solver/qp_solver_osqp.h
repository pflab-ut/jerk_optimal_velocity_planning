#ifndef FILTER_POSITION_OPTIMIZATION_QP_SOLVER_OSQP_H
#define FILTER_POSITION_OPTIMIZATION_QP_SOLVER_OSQP_H

#include "base_solver.h"

namespace osqp
{
    class QPSolver : public BaseSolver
    {
    public:
        QPSolver(const OptimizerParam& param) : BaseSolver(param) {}

        bool solveSoft(const double& initial_vel,
                       const double& initial_acc,
                       const double& ds,
                       const std::vector<double>& ref_vels,
                       const std::vector<double>& max_vels,
                       OutputInfo& output);

        bool solveHard(const double& initial_vel,
                       const double& initial_acc,
                       const double& ds,
                       const std::vector<double>& ref_vels,
                       const std::vector<double>& max_vels,
                       OutputInfo& output);

        bool solveSoftPseudo(const double& initial_vel,
                             const double& initial_acc,
                             const double& ds,
                             const std::vector<double>& ref_vels,
                             const std::vector<double>& max_vels,
                             OutputInfo& output);

        bool solveHardPseudo(const double& initial_vel,
                             const double& initial_acc,
                             const double& ds,
                             const std::vector<double>& ref_vels,
                             const std::vector<double>& max_vels,
                             OutputInfo& output);
    };
}

#endif //FILTER_POSITION_OPTIMIZATION_QP_SOLVER_OSQP_H