#include "solver/qp_solver_osqp.h"

namespace osqp
{
    bool QPSolver::solveSoft(const double& initial_vel,
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
        const long var_size = 5*N;
        const long constraint_size = 4*N;
        const double amax = param_.max_accel;
        const double amin = param_.min_decel;
        const double jmax = param_.max_jerk;
        const double jmin = param_.min_jerk;
        const double over_j_weight = param_.over_j_weight;
        const double over_v_weight = param_.over_v_weight;
        const double over_a_weight = param_.over_a_weight;

        // Hessian
        Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(var_size, var_size);
        for(int i=0; i<N; ++i)
        {
            hessian(i+2*N, i+2*N) = over_v_weight;
            hessian(i+3*N, i+3*N) = over_a_weight;
            hessian(i+4*N, i+4*N) = over_j_weight;
        }

        // Gradient
        std::vector<double> gradient(var_size, 0.0);
        for(long i=0; i<N; ++i)
            gradient[i] = -1.0;

        // Constraint Matrix
        Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Zero(constraint_size, var_size);
        std::vector<double>lowerBound(constraint_size, 0.0);
        std::vector<double>upperBound(constraint_size, 0.0);
        long constraint_num = 0;

        // 1. Velocity Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i) = 1.0; //b[i]
            constraint_matrix(constraint_num, i+2*N) = -1.0; // -delta[i]
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = max_vels[i] * max_vels[i];
        }

        // 2. Acceleration Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+N) = 1.0; //a[i]
            constraint_matrix(constraint_num, i+3*N) = -1.0; // -sigma[i]
            lowerBound[constraint_num] = amin;
            upperBound[constraint_num] = amax;
        }

        //3. Jerk Constraint
        for(long i=0; i<N-1; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+N)   = -ref_vels[i]; // -a[i] * ref_vels[i]
            constraint_matrix(constraint_num, i+1+N) =  ref_vels[i]; // a[i+1] * ref_vels[i]
            constraint_matrix(constraint_num, i+4*N) = -ds; // -pgamma[i]
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

    bool QPSolver::solveHard(const double& initial_vel,
                             const double& initial_acc,
                             const double& ds,
                             const std::vector<double>& ref_vels,
                             const std::vector<double>& max_vels,
                             OutputInfo& output)
    {
        std::cerr << "This has the same form of the LPSolver::SolveHard" << std::endl;
        return false;
    }

    bool QPSolver::solveSoftPseudo(const double& initial_vel,
                                   const double& initial_acc,
                                   const double& ds,
                                   const std::vector<double>& ref_vels,
                                   const std::vector<double>& max_vels,
                                   OutputInfo& output)
    {
        /*
         * x = [b0, b1, ..., bN, |  a0, a1, ..., aN, | delta0, delta1, ..., deltaN, | sigma0, sigma1, ..., sigmaN] in R^{4N}
         * b: velocity^2
         * a: acceleration
         * delta: 0 < bi < vmax^2 + delta
         * sigma: amin < ai - sigma < amax
         */

        // Create Solver
        osqp::OSQPInterface qp_solver_;
        qp_solver_.updateMaxIter(20000);
        qp_solver_.updateRhoInterval(0);  // 0 means automatic
        qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
        qp_solver_.updateEpsAbs(1.0e-8);  // def: 1.0e-4
        qp_solver_.updateVerbose(false);

        long N = static_cast<long>(ref_vels.size());
        const long var_size = 4*N;
        const long constraint_size = 3*N + 1;
        const double amax = param_.max_accel;
        const double amin = param_.min_decel;
        const double over_v_weight = param_.over_v_weight;
        const double over_a_weight = param_.over_a_weight;
        const double smooth_weight = param_.smooth_weight;

        // Hessian
        Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(var_size, var_size);
        for(int i=0; i<N; ++i)
        {
            hessian(i+2*N, i+2*N) = over_v_weight;
            hessian(i+3*N, i+3*N) = over_a_weight;
            if(i<N-1)
            {
                hessian(i+N, i+N) += smooth_weight/ds;
                hessian(i+1+N, i+1+N) += smooth_weight/ds;
                hessian(i+N, i+1+N) += -smooth_weight/ds;
                hessian(i+1+N, i+N) += -smooth_weight/ds;
            }
        }

        // Gradient
        std::vector<double> gradient(var_size, 0.0);
        for(long i=0; i<N; ++i)
            gradient[i] = -1.0;

        // Constraint Matrix
        Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Zero(constraint_size, var_size);
        std::vector<double>lowerBound(constraint_size, 0.0);
        std::vector<double>upperBound(constraint_size, 0.0);
        long constraint_num = 0;

        // 1. Velocity Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i) = 1.0; //b[i]
            constraint_matrix(constraint_num, i+2*N) = -1.0; // -delta[i]
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = max_vels[i] * max_vels[i];
        }

        // 2. Acceleration Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+N) = 1.0; //a[i]
            constraint_matrix(constraint_num, i+3*N) = -1.0; // -sigma[i]
            lowerBound[constraint_num] = amin;
            upperBound[constraint_num] = amax;
        }

        //3. Dynamic Constraint
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

    bool QPSolver::solveHardPseudo(const double& initial_vel,
                                   const double& initial_acc,
                                   const double& ds,
                                   const std::vector<double>& ref_vels,
                                   const std::vector<double>& max_vels,
                                   OutputInfo& output)
    {
        /*
         * x = [b0, b1, ..., bN, |  a0, a1, ..., aN] in R^{2N}
         * b: velocity^2
         * a: acceleration
         * sigma: amin < ai < amax
         */

        // Create Solver
        osqp::OSQPInterface qp_solver_;
        qp_solver_.updateMaxIter(40000);
        qp_solver_.updateRhoInterval(0);  // 0 means automatic
        qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
        qp_solver_.updateEpsAbs(1.0e-8);  // def: 1.0e-4
        qp_solver_.updateVerbose(false);

        long N = static_cast<long>(ref_vels.size());
        const long var_size = 2*N;
        const long constraint_size = 3*N + 1;
        const double amax = param_.max_accel;
        const double amin = param_.min_decel;
        const double smooth_weight = param_.smooth_weight;

        // Hessian
        Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(var_size, var_size);
        for(int i=0; i<N-1; ++i)
        {
            hessian(i+N, i+N) += smooth_weight/ds;
            hessian(i+1+N, i+1+N) += smooth_weight/ds;
            hessian(i+N, i+1+N) += -smooth_weight/ds;
            hessian(i+1+N, i+N) += -smooth_weight/ds;
        }

        // Gradient
        std::vector<double> gradient(var_size, 0.0);
        for(long i=0; i<N; ++i)
            gradient[i] = -1.0;

        // Constraint Matrix
        Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Zero(constraint_size, var_size);
        std::vector<double>lowerBound(constraint_size, 0.0);
        std::vector<double>upperBound(constraint_size, 0.0);
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

        //3. Dynamic Constraint
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
}
