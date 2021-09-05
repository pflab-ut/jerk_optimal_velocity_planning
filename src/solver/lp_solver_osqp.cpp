#include "solver/lp_solver_osqp.h"

namespace osqp
{
    bool LPSolver::solveSoft(const double& initial_vel,
                             const double& initial_acc,
                             const double& ds,
                             const std::vector<double>& ref_vels,
                             const std::vector<double>& max_vels,
                             OutputInfo& output)
    {
        /*
         * x = [b[0], b[1], ..., b[N] | a[0], a[1], .... a[N]
         *      | pdelta[0], pdelta[1], ..., pdelta[N]
         *      | mdelta[0], mdelta[1], ..., mdelta[N]
         *      | psigma[0], psigma[1], ..., psigma[N]
         *      | msigma[0], msigma[1], ..., msigma[N]
         *      | pgamma[0], pgamma[1], ..., pgamma[N]
         *      | mgamma[0], mgamma[1], ..., mgamma[N]
         *      delta[i] = pdelta[i] - mdelta[i]
         *      sigma[i] = psigma[i] - msigma[i]
         *      gamma[i] = pgamma[i] - mgamma[i]
         *      |delta[i]| = pdelta[i] + mdelta[i]
         *      |sigma[i]| = psigma[i] + msigma[i]
         *      |gamma[i]| = pgamma[i] + mgamma[i]
         * b[i]: velocity^2
         * delta: 0 < b[i]-delta[i] < max_vel[i]*max_vel[i]
         * sigma: amin < a[i] - sigma[i] < amax
         * gamma: jerk_min/ref_vel[i] < pseudo_jerk[i] - gamma[i] < jerk_max/ref_vel[i]
         */

        // Create Solver
        osqp::OSQPInterface qp_solver_;
        qp_solver_.updateMaxIter(20000);
        qp_solver_.updateRhoInterval(0);  // 0 means automatic
        qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
        qp_solver_.updateEpsAbs(1.0e-8);  // def: 1.0e-4
        qp_solver_.updateVerbose(false);

        long N = static_cast<long>(ref_vels.size());
        const long var_size = 8*N;
        const long constraint_size = 10*N;
        const double amax = param_.max_accel;
        const double amin = param_.min_decel;
        const double jmax = param_.max_jerk;
        const double jmin = param_.min_jerk;
        const double over_j_weight = param_.over_j_weight;
        const double over_v_weight = param_.over_v_weight;
        const double over_a_weight = param_.over_a_weight;

        // Hessian
        Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(var_size, var_size);

        // Gradient
        std::vector<double> gradient(var_size, 0.0);
        for(long i=0; i<N; ++i)
            gradient[i] = -1.0;
        for(long i=2*N; i<4*N; ++i)
            gradient[i] = over_v_weight;
        for(long i=4*N; i<6*N; ++i)
            gradient[i] = over_a_weight;
        for(long i=6*N; i<8*N; ++i)
            gradient[i] = over_j_weight;

        // Constraint Matrix
        Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Zero(constraint_size, var_size);
        std::vector<double>lowerBound(constraint_size, 0.0);
        std::vector<double>upperBound(constraint_size, 0.0);
        long constraint_num = 0;

        // 0. Soft Constraint Variable
        for(long i=0; i<6*N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+2*N) = 1.0;
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = 1e100;
        }

        // 1. Velocity Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i) = 1.0; //b[i]
            constraint_matrix(constraint_num, i+2*N) = -1.0; // -pdelta[i]
            constraint_matrix(constraint_num, i+3*N) =  1.0; // mdelta[i]
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = max_vels[i] * max_vels[i];
        }

        // 2. Acceleration Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+N) = 1.0; //a[i]
            constraint_matrix(constraint_num, i+4*N) = -1.0; // -psigma[i]
            constraint_matrix(constraint_num, i+5*N) =  1.0; // msigma[i]
            lowerBound[constraint_num] = amin;
            upperBound[constraint_num] = amax;
        }

        //3. Jerk Constraint
        for(long i=0; i<N-1; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+N)   = -ref_vels[i]; // -a[i] * ref_vels[i]
            constraint_matrix(constraint_num, i+1+N) =  ref_vels[i]; // a[i+1] * ref_vels[i]
            constraint_matrix(constraint_num, i+6*N) = -ds; // -pgamma[i]
            constraint_matrix(constraint_num, i+7*N) =  ds; // mgamma[i]
            lowerBound[constraint_num] = jmin*ds;
            upperBound[constraint_num] = jmax*ds;
        }

        //4. Dynamic Constraint
        for(long i=0; i<N-1; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i) = -1.0; // -b[i]
            constraint_matrix(constraint_num, i+1) = 1.0; // b[i+1]
            constraint_matrix(constraint_num, i+N) = -2.0*ds; // a[i] * -2.0 * ds
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = 0.0;
        }

        //5. Initial Condition
        constraint_matrix(constraint_num, 0) = 1.0;
        lowerBound[constraint_num] = initial_vel*initial_vel;
        upperBound[constraint_num] = initial_vel*initial_vel;
        ++constraint_num;
        constraint_matrix(constraint_num, N) = 1.0;
        lowerBound[constraint_num] = initial_acc;
        upperBound[constraint_num] = initial_acc;
        ++constraint_num;
        assert(constraint_num == constraint_size);

        // solve the QP problem
        const auto result = qp_solver_.optimize(hessian, constraint_matrix, gradient, lowerBound, upperBound);

        const std::vector<double> optval = std::get<0>(result);

        output.resize(N);
        for(unsigned int i=0; i<N; ++i)
        {
            output.velocity[i] = std::sqrt(std::max(optval[i], 0.0));
            output.acceleration[i] = optval[i+N];
        }

        for(unsigned int i=0; i<N-1; ++i)
        {
            double a_current = optval[i+N];
            double a_next    = optval[i+N+1];
            output.jerk[i] = (a_next - a_current) * output.velocity[i] / ds;
        }
        output.jerk[N-1] = output.jerk[N-2];

        return true;
    }

    bool LPSolver::solveHard(const double& initial_vel,
                             const double& initial_acc,
                             const double& ds,
                             const std::vector<double>& ref_vels,
                             const std::vector<double>& max_vels,
                             OutputInfo& output)
    {
        /*
         * x = [b[0], b[1], ..., b[N] | a[0], a[1], .... a[N]]
         * b[i]: velocity^2
         * 0 < b[i] < max_vel[i]*max_vel[i]
         * amin < a[i] < amax
         * jerk_min/ref_vel[i] < pseudo_jerk[i] < jerk_max/ref_vel[i]
         */

        osqp::OSQPInterface qp_solver_;
        qp_solver_.updateMaxIter(20000);
        qp_solver_.updateRhoInterval(0);  // 0 means automatic
        qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
        qp_solver_.updateEpsAbs(1.0e-8);  // def: 1.0e-4
        qp_solver_.updateVerbose(false);

        long N = static_cast<long>(ref_vels.size());
        const long var_size = 2*N;
        const long constraint_size = 4*N;
        const double amax = param_.max_accel;
        const double amin = param_.min_decel;
        const double jmax = param_.max_jerk;
        const double jmin = param_.min_jerk;

        // Hessian
        Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(var_size, var_size);

        // Gradient
        std::vector<double> gradient(var_size, 0.0);
        for(long i=0; i<N; ++i)
            gradient[i] = -1.0;

        // Constraint Matrix
        Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Zero(constraint_size, var_size);
        std::vector<double> lowerBound(constraint_size, 0.0);
        std::vector<double> upperBound(constraint_size, 0.0);
        long constraint_num = 0;

        // 1. Velocity Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i) = 1.0; //b[i]
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = max_vels[i] * max_vels[i];
        }

        // 2. Acceleration Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+N) = 1.0; //a[i]
            lowerBound[constraint_num] = amin;
            upperBound[constraint_num] = amax;
        }

        //3. Jerk Constraint
        for(long i=0; i<N-1; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+N)   = -ref_vels[i]; // -a[i] * ref_vels[i]
            constraint_matrix(constraint_num, i+1+N) =  ref_vels[i]; // a[i+1] * ref_vels[i]
            lowerBound[constraint_num] = jmin*ds;
            upperBound[constraint_num] = jmax*ds;
        }

        //4. Dynamic Constraint
        for(long i=0; i<N-1; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+1) = 1.0; // b[i+1]
            constraint_matrix(constraint_num, i) = -1.0; // -b[i]
            constraint_matrix(constraint_num, i+N) = -2.0*ds; // a[i] * -2.0 * ds
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = 0.0;
        }

        //5. Initial Condition
        constraint_matrix(constraint_num, 0) = 1.0;
        lowerBound[constraint_num] = initial_vel*initial_vel;
        upperBound[constraint_num] = initial_vel*initial_vel;
        ++constraint_num;
        constraint_matrix(constraint_num, N) = 1.0;
        lowerBound[constraint_num] = initial_acc;
        upperBound[constraint_num] = initial_acc;
        ++constraint_num;
        assert(constraint_num == constraint_size);

        // solve the QP problem
        const auto result = qp_solver_.optimize(hessian, constraint_matrix, gradient, lowerBound, upperBound);

        const std::vector<double> optval = std::get<0>(result);
        output.resize(N);
        for(unsigned int i=0; i<N; ++i)
        {
            output.velocity[i] = std::sqrt(std::max(optval[i], 0.0));
            output.acceleration[i] = optval[i+N];
        }

        for(unsigned int i=0; i<N-1; ++i)
        {
            double a_current = optval[i+N];
            double a_next    = optval[i+N+1];
            output.jerk[i] = (a_next - a_current) * output.velocity[i] / ds;
        }
        output.jerk[N-1] = output.jerk[N-2];

        return true;
    }

    bool LPSolver::solveSoftPseudo(const double &initial_vel,
                                   const double &initial_acc,
                                   const double &ds,
                                   const std::vector<double> &ref_vels,
                                   const std::vector<double> &max_vels,
                                   OutputInfo &output)
    {
        std::cerr << "[Solver Error]: LP Solver cannot be applied to the pseudo-jerk problem" << std::endl;
        return false;
    }

    bool LPSolver::solveHardPseudo(const double &initial_vel,
                                   const double &initial_acc,
                                   const double &ds,
                                   const std::vector<double> &ref_vels,
                                   const std::vector<double> &max_vels,
                                   OutputInfo &output)

    {
        return solveSoftPseudo(initial_vel, initial_acc, ds, ref_vels, max_vels, output);
    }
}
