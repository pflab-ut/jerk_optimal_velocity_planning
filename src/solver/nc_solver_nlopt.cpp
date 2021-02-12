#include "solver/nc_solver_nlopt.h"

namespace nlopt
{
    bool NCSolver::solve(const double& initial_vel,
                         const double& initial_acc,
                         const double& ds,
                         const std::vector<double>& ref_vels,
                         const std::vector<double>& max_vels,
                         OutputInfo& output)
    {
        /*
         * x = [b[0], b[1], ..., b[N] | a[0], a[1], .... a[N] | delta[0], ..., delta[N]
         *      | sigma[0], sigma[1], ...., sigma[N] | gamma[0], gamma[1], ..., gamma[N] ]
         * b[i]: velocity^2
         * delta: 0 < b[i]-delta[i] < max_vel[i]*max_vel[i]
         * sigma: amin < a[i] - sigma[i] < amax
         * gamma: jerk_min < jerk[i]  < jerk_max
         */

        double tol = 1e-5;

        assert(ref_vels.size()==max_vels.size());
        int N = ref_vels.size();
        int dim = 2*N;
        const double amax = param_.max_accel;
        const double amin = param_.min_decel;
        const double jmax = param_.max_jerk;
        const double jmin = param_.min_jerk;
        const double over_j_weight = param_.over_j_weight;
        const double over_v_weight = param_.over_v_weight;
        const double over_a_weight = param_.over_a_weight;

        nlopt::opt opt(nlopt::LD_SLSQP, dim); //algorithm and dimension of the problem
        opt.set_maxeval(400000);

        //  1. Create Input Constraint
        std::vector<double> lb(dim); // Lower Bound
        std::vector<double> ub(dim); // Upper Bound
        for(int i=0; i<dim; ++i)
        {
            lb[i] = -HUGE_VAL;
            ub[i] =  HUGE_VAL;
        }

        // initial value
        lb[0] = initial_vel*initial_vel;
        ub[0] = initial_vel*initial_vel;
        lb[N] = initial_acc;
        ub[N] = initial_acc;

        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);

        // 2. Set Objective Function
        ObjectiveParameter param;
        param.dim_b_ = N;
        param.dim_a_ = N;
        param.dim_delta_ = N;
        param.dim_sigma_ = N;
        param.dim_gamma_ = N;
        param.over_v_weight_ = over_v_weight;
        param.over_a_weight_ = over_a_weight;
        param.over_j_weight_ = over_j_weight;
        opt.set_min_objective(computeObjective, &param);

        //3. Add Equality Constraint
        EConstParameter ec_param;
        ec_param.N_  = N;
        ec_param.ds_ = ds;
        std::vector<double> ec_tols(N-1, tol);
        opt.add_equality_mconstraint(computeEqualityConstraint, &ec_param, ec_tols);

        // 4. Add Velocity Constraint
        VelConstParameter vel_param;
        vel_param.max_vels_ = max_vels;
        vel_param.N_        = N;
        std::vector<double> vc_tols(2*N, tol);
        opt.add_inequality_mconstraint(computeVelConstraint, &vel_param, vc_tols);

        // 5. Add Acceleration Constraint
        AccConstParameter ac_param;
        ac_param.a_max_ = amax;
        ac_param.a_min_ = amin;
        ac_param.N_     = N;
        std::vector<double> ac_tols(2*N, tol);
        opt.add_inequality_mconstraint(computeAccConstraint, &ac_param, ac_tols);

        // 6. Add Jerk Constraint
        JerkConstParameter jc_param;
        jc_param.N_  = N;
        jc_param.ds_ = ds;
        jc_param.j_max_ = jmax;
        jc_param.j_min_ = jmin;
        std::vector<double> jc_tols(2*(N-1), tol);
        opt.add_inequality_mconstraint(computeJerkConstraint, &jc_param, jc_tols);

        // 7.Set x tolerance
        opt.set_xtol_rel(tol);

        // 8. Set Initial x value
        std::vector<double> x(dim, 0.0);
        x[0] = initial_vel*initial_vel;
        x[1] = x[0] + 2*initial_acc*ds; // b[1] = b[0] + 2*a[0]*ds
        x[N] = initial_acc;

        for(int i=2; i<N; ++i)
            x[i] = max_vels[i] * max_vels[i];

        // Solve the problem
        double min_f;
        try
        {
            nlopt::result result = opt.optimize(x, min_f);

            std::cout << "nlopt success" << std::endl;
            std::cout << "Minimum Value: " << min_f << std::endl;

            output.resize(N);
            for(int i=0; i<N; ++i)
            {
                output.velocity[i]     = std::sqrt(std::max(x[i], 0.0));
                output.acceleration[i] = x[i+N];
            }
            for(unsigned int i=0; i<N-1; ++i)
            {
                double a_current = output.acceleration[i];
                double a_next    = output.acceleration[i+1];
                output.jerk[i] = (a_next - a_current) * output.velocity[i] / ds;
            }
            output.jerk[N-1] = output.jerk[N-2];
        }
        catch(std::exception & e)
        {
            std::cout << "NLOPT Failed: " << e.what() << std::endl;
            return false;
        }

        return true;
    }

    bool NCSolver::solve(const double& initial_vel,
                         const double& initial_acc,
                         const double& ds,
                         const std::vector<double>& max_vels,
                         OutputInfo& output)
    {
/*
         * x = [b[0], b[1], ..., b[N] | a[0], a[1], .... a[N] ]
         * b[i]: velocity^2
         * delta: 0 < b[i] < max_vel[i]*max_vel[i]
         * sigma: amin < a[i] < amax
         * gamma: jerk_min < jerk[i]  < jerk_max
         */

        double tol = 1e-5;

        assert(max_vels.size()==max_vels.size());
        int N = max_vels.size();
        int dim = 2*N;
        const double amax = param_.max_accel;
        const double amin = param_.min_decel;
        const double jmax = param_.max_jerk;
        const double jmin = param_.min_jerk;

        nlopt::opt opt(nlopt::LD_SLSQP, dim); //algorithm and dimension of the problem
        opt.set_maxeval(400000);

        //  1. Create Input Constraint
        std::vector<double> lb(dim); // Lower Bound
        std::vector<double> ub(dim); // Upper Bound
        for(int i=0; i<N; ++i)
        {
            lb[i] = 0.0;
            ub[i] = max_vels[i]*max_vels[i];
        }

        for(int i=N; i<2*N; ++i)
        {
            lb[i] = amin;
            ub[i] = amax;
        }

        // initial value
        lb[0] = initial_vel*initial_vel;
        ub[0] = initial_vel*initial_vel;
        lb[N] = initial_acc;
        ub[N] = initial_acc;

        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);

        // 2. Set Objective Function
        ObjectiveParameter param;
        param.dim_b_ = N;
        param.dim_a_ = N;
        param.dim_delta_ = N;
        param.dim_sigma_ = N;
        param.dim_gamma_ = N;
        opt.set_min_objective(computeHardObjective, &param);

        //3. Add Equality Constraint
        EConstParameter ec_param;
        ec_param.N_  = N;
        ec_param.ds_ = ds;
        std::vector<double> ec_tols(N-1, tol);
        opt.add_equality_mconstraint(computeEqualityConstraint, &ec_param, ec_tols);

        // 4. Add Jerk Constraint
        JerkConstParameter jc_param;
        jc_param.N_  = N;
        jc_param.ds_ = ds;
        jc_param.j_max_ = jmax;
        jc_param.j_min_ = jmin;
        std::vector<double> jc_tols(2*(N-1), tol);
        opt.add_inequality_mconstraint(computeJerkHardConstraint, &jc_param, jc_tols);

        // 5.Set x tolerance
        opt.set_xtol_rel(tol);

        // 6. Set Initial x value
        std::vector<double> x(dim, 0.0);
        x[0] = initial_vel*initial_vel;
        x[N] = initial_acc;

        for(int i=1; i<N; ++i)
            x[i] = max_vels[i] * max_vels[i];

        // Solve the problem
        double min_f;
        try
        {
            nlopt::result result = opt.optimize(x, min_f);

            std::cout << "nlopt success" << std::endl;
            std::cout << "Minimum Value: " << min_f << std::endl;

            output.resize(N);
            for(int i=0; i<N; ++i)
            {
                output.velocity[i]     = std::sqrt(std::max(x[i], 0.0));
                output.acceleration[i] = x[i+N];
            }
            for(unsigned int i=0; i<N-1; ++i)
            {
                double a_current = output.acceleration[i];
                double a_next    = output.acceleration[i+1];
                output.jerk[i] = (a_next - a_current) * output.velocity[i] / ds;
            }
            output.jerk[N-1] = output.jerk[N-2];
        }
        catch(std::exception & e)
        {
            std::cout << "NLOPT Failed: " << e.what() << std::endl;
            return false;
        }

        return true;
    }

    double NCSolver::computeObjective(const std::vector<double>& x,
                                      std::vector<double>& grad,
                                      void* parameter)
    {
        ObjectiveParameter* param = reinterpret_cast<ObjectiveParameter*>(parameter);
        int N = param->dim_b_;
        double over_v_weight = param->over_v_weight_;
        double over_a_weight = param->over_a_weight_;
        double over_j_weight = param->over_j_weight_;

        if(!grad.empty())
        {
            for(int i=0; i<N; ++i)
                grad[i] = -1.0;

            for(int i=N; i<2*N; ++i)
                grad[i] = 0.0;

            for(int i=2*N; i<3*N; ++i)
                grad[i] = over_v_weight*x[i];

            for(int i=3*N; i<4*N; ++i)
                grad[i] = over_a_weight*x[i];

            for(int i=4*N; i<5*N; ++i)
                grad[i] = over_j_weight*x[i];
        }

        double cost = 0.0;
        for(int i=0; i<N; ++i)
            cost += -x[i] + over_v_weight*0.5*x[i+2*N]*x[i+2*N]
                          + over_a_weight*0.5*x[i+3*N]*x[i+3*N]
                          + over_j_weight*0.5*x[i+4*N]*x[i+4*N];

        return cost;
    }

    double NCSolver::computeHardObjective(const std::vector<double>& x,
                                          std::vector<double>& grad,
                                          void* parameter)
    {
        ObjectiveParameter* param = reinterpret_cast<ObjectiveParameter*>(parameter);
        int N = param->dim_b_;

        if(!grad.empty())
        {
            for(int i=0; i<N; ++i)
                grad[i] = -1.0;

            for(int i=N; i<2*N; ++i)
                grad[i] = 0.0;
        }

        double cost = 0.0;
        for(int i=0; i<N; ++i)
            cost += -x[i];

        return cost;
    }

    void NCSolver::computeEqualityConstraint(unsigned m,
                                             double* result,
                                             unsigned n,
                                             const double* x,
                                             double* grad,
                                             void* parameter)
    {
        EConstParameter * param = reinterpret_cast<EConstParameter *>(parameter);
        double ds = param->ds_;
        int N = param->N_;
        assert(N-1==m);

        // b' = 2a ... (b(i+1) - b(i)) = 2*a(i)*ds
        if(grad)
        {
            for(int i=0; i<m*n; ++i)
                grad[i] = 0.0;

            for(int i=0; i<m; ++i)
            {
                for(int j=0; j<N-1; ++j)
                {
                    if(i==j)
                    {
                        grad[i*n + j]     = -1.0; // dc_i/db_j
                        grad[i*n + j + 1] =  1.0; // dc_i/db_j+1
                        grad[i*n + j + N] = -2.0*ds; // dc_i/da_j
                    }
                }
            }
        }
        for(int i=0; i<m; ++i)
            result[i] = x[i+1] - x[i] - 2.0 * x[i+N] * ds;
    }

    void NCSolver::computeVelConstraint(unsigned int m,
                                        double *result,
                                        unsigned int n,
                                        const double *x,
                                        double *grad,
                                        void *parameter)
    {
        // 0 <= b[i]-delta[i] <= max_vel[i]*max_vel[i]

        VelConstParameter * param = reinterpret_cast<VelConstParameter *>(parameter);
        int N = param->N_;
        std::vector<double> max_vels = param->max_vels_;
        assert(2*N==m);

        if(grad)
        {
            for(int i=0; i<m*n; ++i)
                grad[i] = 0.0;

            for(int i=0; i<N; ++i) // for(int i=0; i<m; ++i)
            {
                for(int j=0; j<N; ++j)
                {
                    if(i==j)
                    {
                        grad[i*n + j]       =  1.0; // dc_i/db_i
                        grad[i*n + j + 2*N] = -1.0; // dc_i/d(delta_i)
                        grad[(i+N)*n + j]   = -1.0; // dc_(i+N)/db_i
                        grad[(i+N)*n + j + 2*N] =  1.0; // dc_(i+N)/d(delta_i)
                    }
                }
            }
        }

        for(int i=0; i<N; ++i)
        {
            // b[i] - delta[i] - vmax*vmax <= 0
            result[i]   =  x[i] - x[i+2*N] - max_vels[i]*max_vels[i];

            // 0 - b[i] + delta[i] <= 0
            result[i+N] = -x[i] + x[i+2*N];
        }
    }

    void NCSolver::computeAccConstraint(unsigned int m,
                                        double *result,
                                        unsigned int n,
                                        const double *x,
                                        double *grad,
                                        void *parameter)
    {
        AccConstParameter * param = reinterpret_cast<AccConstParameter *>(parameter);
        int N = param->N_;
        double amax = param->a_max_;
        double amin = param->a_min_;
        assert(2*N==m);

        if(grad)
        {
            for(int i=0; i<m*n; ++i)
                grad[i] = 0.0;

            for(int i=0; i<N; ++i) // for(int i=0; i<m; ++i)
            {
                for(int j=0; j<N; ++j)
                {
                    if(i==j)
                    {
                        grad[i*n + j + N]     =  1.0; // dc_i/da_i
                        grad[i*n + j + 3*N]   = -1.0; // dc_i/d(sigma_i)
                        grad[(i+N)*n + j + N]   = -1.0; // dc_(i+N)/da_i
                        grad[(i+N)*n + j + 3*N] =  1.0; // dc_(i+N)/d(sigma_i)
                    }
                }
            }
        }

        for(int i=0; i<N; ++i)
        {
            result[i]   = x[i+N] - x[i+3*N] - amax;
            result[i+N] = amin - x[i+N] + x[i+3*N];
        }
    }

    void NCSolver::computeJerkConstraint(unsigned int m,
                                         double *result,
                                         unsigned int n,
                                         const double *x,
                                         double *grad,
                                         void *parameter)
    {
        JerkConstParameter * param = reinterpret_cast<JerkConstParameter *>(parameter);
        int N = param->N_;
        double ds = param->ds_;
        double jmax = param->j_max_;
        double jmin = param->j_min_;
        assert(2*(N-1)==m);

        if(grad)
        {
            for(int i=0; i<m*n; ++i)
                grad[i] = 0.0;

            for(int i=0; i<N-1; ++i) // for(int i=0; i<m; ++i)
            {
                for(int j=0; j<N-1; ++j)
                {
                    if(i==j)
                    {
                        grad[i*n + j]         =  (x[i+N+1]-x[i+N])/(2*std::sqrt(std::max(x[i], 0.1))); // dc_i/db_i
                        grad[i*n + j + N]     =  -std::sqrt(x[i]); // dc_i/da_i
                        grad[i*n + j + N + 1] =   std::sqrt(x[i]); // dc_i/da_i+1
                        grad[i*n + j + 4*N]   =  -ds; // dc_i/d(gamma_i)

                        grad[(i+N-1)*n + j]   =  -(x[i+N+1]-x[i+N])/(2*std::sqrt(std::max(x[i], 0.1))); // dc_i/db_i
                        grad[(i+N-1)*n + j + N]     =  std::sqrt(x[i]); // dc_(i+N)/da_i
                        grad[(i+N-1)*n + j + N + 1] = -std::sqrt(x[i]); // dc_(i+N)/d(da_i+1)
                        grad[(i+N-1)*n + j + 4*N]   =  ds; // dc_(i+N)/d(gamma_i)
                    }
                }
            }
        }

        for(int i=0; i<N-1; ++i)
        {
            result[i]   = (x[i+N+1]-x[i+N])*std::sqrt(x[i]) - x[i+4*N]*ds - jmax*ds;
            result[i+N] = jmin*ds - (x[i+N+1]-x[i+N])*std::sqrt(x[i]) + x[i+4*N]*ds;
        }
    }

    void NCSolver::computeJerkHardConstraint(unsigned int m,
                                             double *result,
                                             unsigned int n,
                                             const double *x,
                                             double *grad,
                                             void *parameter)
    {
        JerkConstParameter * param = reinterpret_cast<JerkConstParameter *>(parameter);
        int N = param->N_;
        double ds = param->ds_;
        double jmax = param->j_max_;
        double jmin = param->j_min_;
        assert(2*(N-1)==m);

        if(grad)
        {
            for(int i=0; i<m*n; ++i)
                grad[i] = 0.0;

            for(int i=0; i<N-1; ++i) // for(int i=0; i<m; ++i)
            {
                for(int j=0; j<N-1; ++j)
                {
                    if(i==j)
                    {
                        grad[i*n + j]         =  (x[i+N+1]-x[i+N])/(2*ds*std::sqrt(std::max(x[i], 0.001))); // dc_i/db_i
                        grad[i*n + j + N]     =  -std::sqrt(x[i])/ds; // dc_i/da_i
                        grad[i*n + j + N + 1] =   std::sqrt(x[i])/ds; // dc_i/da_i+1

                        grad[(i+N-1)*n + j]   =  -(x[i+N+1]-x[i+N])/(2*std::sqrt(std::max(x[i], 0.001))); // dc_i/db_i
                        grad[(i+N-1)*n + j + N]     =  std::sqrt(x[i])/ds; // dc_(i+N)/da_i
                        grad[(i+N-1)*n + j + N + 1] = -std::sqrt(x[i])/ds; // dc_(i+N)/d(da_i+1)
                    }
                }
            }
        }

        for(int i=0; i<N-1; ++i)
        {
            result[i]   = (x[i+N+1]-x[i+N])*std::sqrt(x[i])/ds - jmax;
            result[i+N] = jmin - (x[i+N+1]-x[i+N])*std::sqrt(x[i])/ds;
        }
    }
}
