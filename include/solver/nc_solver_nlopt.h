#ifndef FILTER_POSITION_OPTIMIZATION_NC_SOLVER_NLOPT_H
#define FILTER_POSITION_OPTIMIZATION_NC_SOLVER_NLOPT_H

#include "solver/base_solver.h"

namespace nlopt
{
    typedef struct
    {
        int dim_b_;
        int dim_a_;
        int dim_delta_;
        int dim_sigma_;
        int dim_gamma_;
        double over_v_weight_;
        double over_a_weight_;
        double over_j_weight_;
    }ObjectiveParameter;

    typedef struct
    {
        double ds_;
        int N_;
    }EConstParameter;

    typedef struct
    {
        int N_;
        std::vector<double> max_vels_;
    }VelConstParameter;

    typedef struct
    {
        int N_;
        double a_max_;
        double a_min_;
    }AccConstParameter;

    typedef struct
    {
        int N_;
        double ds_;
        double j_max_;
        double j_min_;
    }JerkConstParameter;

    //Nonconvex(NC) Solver
    class NCSolver : public BaseSolver
    {
    public:
        NCSolver(const OptimizerParam& param) : BaseSolver(param) {}

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

    private:
        static double computeObjective(const std::vector<double>& x,
                                       std::vector<double>& grad,
                                       void* parameter);

        static void computeEqualityConstraint(unsigned m,
                                              double* result,
                                              unsigned n,
                                              const double* x,
                                              double* grad,
                                              void* parameter);

        static void computeVelConstraint(unsigned m,
                                         double* result,
                                         unsigned n,
                                         const double* x,
                                         double* grad,
                                         void* parameter);

        static void computeAccConstraint(unsigned m,
                                         double* result,
                                         unsigned n,
                                         const double* x,
                                         double* grad,
                                         void* parameter);

        static void computeJerkConstraint(unsigned m,
                                          double* result,
                                          unsigned n,
                                          const double* x,
                                          double* grad,
                                          void* parameter);
    };
}

#endif //FILTER_POSITION_OPTIMIZATION_NC_SOLVER_NLOPT_H
