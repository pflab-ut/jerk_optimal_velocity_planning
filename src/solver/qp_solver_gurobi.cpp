#include "solver/qp_solver_gurobi.h"

namespace gurobi
{
    bool QPSolver::solveSoft(const double& initial_vel,
                             const double& initial_acc,
                             const double& ds,
                             const std::vector<double>& ref_vels,
                             const std::vector<double>& max_vels,
                             OutputInfo& output)
    {
        try
        {
            /* Create Environment */
            GRBEnv env = GRBEnv();
            GRBModel model = GRBModel(env);
            model.set(GRB_DoubleParam_TimeLimit, 100.0);
            model.set(GRB_DoubleParam_IterationLimit, 40000);
            model.set(GRB_DoubleParam_FeasibilityTol, 1e-4);
            model.set(GRB_DoubleParam_BarConvTol, 1e-4);
            model.set(GRB_DoubleParam_OptimalityTol, 1e-4);
            model.set(GRB_IntParam_OutputFlag, 0);

            assert(ref_vels.size()==max_vels.size());
            int N = ref_vels.size();

            /*
             * x = [b[0], b[1], ..., b[N] | a[0], a[1], .... a[N] | delta[0], ..., delta[N]
             *      | sigma[0], sigma[1], ...., sigma[N] | gamma[0], gamma[1], ..., gamma[N] ]
             * b[i]: velocity^2
             * delta: 0 < b[i]-delta[i] < max_vel[i]*max_vel[i]
             * sigma: amin < a[i] - sigma[i] < amax
             * gamma: jerk_min/ref_vel[i] < pseudo_jerk[i] - gamma[i] < jerk_max/ref_vel[i]
             */

            std::vector<GRBVar> b(N);
            std::vector<GRBVar> a(N);
            std::vector<GRBVar> delta(N);
            std::vector<GRBVar> sigma(N);
            std::vector<GRBVar> gamma(N);

            for(int i=0; i<N; ++i)
            {
                b[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "b"+std::to_string(i));
                a[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "a"+std::to_string(i));
                delta[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "delta"+std::to_string(i));
                sigma[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "sigma"+std::to_string(i));
                gamma[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "gamma"+std::to_string(i));
            }

            const double amax = param_.max_accel;
            const double amin = param_.min_decel;
            const double jmax = param_.max_jerk;
            const double jmin = param_.min_jerk;
            const double over_j_weight = param_.over_j_weight;
            const double over_v_weight = param_.over_v_weight;
            const double over_a_weight = param_.over_a_weight;

            /**************************************************************/
            /**************************************************************/
            /**************** design objective function *******************/
            /**************************************************************/
            /**************************************************************/
            GRBLinExpr  Jl = 0.0;
            GRBQuadExpr Jq = 0.0;
            for(int i=0; i<N; ++i)
            {
                Jl += -b[i];
                Jq += over_v_weight*delta[i]*delta[i]+over_a_weight*sigma[i]*sigma[i]+over_j_weight*gamma[i]*gamma[i];
            }

            model.setObjective(Jl + Jq, GRB_MINIMIZE);

            /**************************************************************/
            /**************************************************************/
            /**************** design constraint matrix ********************/
            /**************************************************************/
            /**************************************************************/
            for(int i=0; i<N; ++i)
            {
                // 0 < b - delta < vmax^2
                model.addConstr(0 <= b[i]-delta[i], "blconstraint"+std::to_string(i));
                model.addConstr(b[i]-delta[i] <= max_vels[i]*max_vels[i], "buconstraint"+std::to_string(i));

                // amin < a - sigma < amax
                model.addConstr(amin <= a[i] - sigma[i], "alconstraint" + std::to_string(i));
                model.addConstr(a[i] - sigma[i] <= amax, "auconstraint" + std::to_string(i));
            }

            for(int i=0; i<N-1; ++i)
            {
                // Soft Constraint Jerk Limit: jerk_min < pseudo_jerk[i] * ref_vel[i] - gamma[i] < jerk_max
                // -> jerk_min * ds < (a[i+1] - a[i]) * ref_vel[i] - gamma[i] * ds < jerk_max * ds
                model.addConstr(jmin * ds <= (a[i+1] - a[i])*ref_vels[i] - gamma[i]*ds, "jlconstraint"+std::to_string(i));
                model.addConstr((a[i+1] - a[i])*ref_vels[i] - gamma[i]*ds <= jmax*ds, "juconstraint"+std::to_string(i));

                // b' = 2a ... (b(i+1) - b(i)) = 2a(i)*ds
                model.addConstr((b[i+1]-b[i]) == 2*a[i]*ds, "equality"+std::to_string(i));
            }

            // Initial Condition
            model.addConstr(b[0]==initial_vel*initial_vel, "v0");
            model.addConstr(a[0]==initial_acc, "a0");

            /**************************************************************/
            /**************************************************************/
            /********************** Optimize ******************************/
            /**************************************************************/
            /**************************************************************/
            model.optimize();

            output.resize(N);
            for(unsigned int i=0; i<N; ++i)
            {
                output.velocity[i] = std::sqrt(std::max(b[i].get(GRB_DoubleAttr_X), 0.0));
                output.acceleration[i] = a[i].get(GRB_DoubleAttr_X);
            }

            for(unsigned int i=0; i<N-1; ++i)
            {
                double a_current = output.acceleration[i];
                double a_next    = output.acceleration[i+1];
                output.jerk[i] = (a_next - a_current) * output.velocity[i] / ds;
            }
            output.jerk[N-1] = output.jerk[N-2];

            std::cout << "QP Runtime: " << model.get(GRB_DoubleAttr_Runtime)*1e3 << "[ms]" << std::endl;
        }
        catch(GRBException& e)
        {
            std::cout << "Error code = " << e.getErrorCode() << std::endl;
            std::cout << e.getMessage() << std::endl;
            return false;
        } catch(...)
        {
            std::cout << "Exception during optimization" << std::endl;
            return false;
        }

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
        try
        {
            /* Create Environment */
            GRBEnv env = GRBEnv();
            GRBModel model = GRBModel(env);
            model.set(GRB_DoubleParam_TimeLimit, 100.0);
            model.set(GRB_DoubleParam_IterationLimit, 40000);
            model.set(GRB_DoubleParam_FeasibilityTol, 1e-4);
            model.set(GRB_DoubleParam_BarConvTol, 1e-4);
            model.set(GRB_DoubleParam_OptimalityTol, 1e-4);
            model.set(GRB_IntParam_OutputFlag, 0);

            int N = max_vels.size();
            /*
             * x = [b0, b1, ..., bN, |  a0, a1, ..., aN, | delta0, delta1, ..., deltaN, | sigma0, sigma1, ..., sigmaN] in R^{4N}
             * b: velocity^2
             * a: acceleration
             * delta: 0 < bi < vmax^2 + delta
             * sigma: amin < ai - sigma < amax
             */

            std::vector<GRBVar> b(N);
            std::vector<GRBVar> a(N);
            std::vector<GRBVar> delta(N);
            std::vector<GRBVar> sigma(N);

            for(int i=0; i<N; ++i)
            {
                b[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "b"+std::to_string(i));
                a[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "a"+std::to_string(i));
                delta[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "delta"+std::to_string(i));
                sigma[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "sigma"+std::to_string(i));
            }

            const double amax = param_.max_accel;
            const double amin = param_.min_decel;
            const double smooth_weight = param_.smooth_weight;
            const double over_v_weight = param_.over_v_weight;
            const double over_a_weight = param_.over_a_weight;

            /**************************************************************/
            /**************************************************************/
            /**************** design objective function *******************/
            /**************************************************************/
            /**************************************************************/
            GRBLinExpr  Jl = 0.0;
            GRBQuadExpr Jq = 0.0;
            for(int i=0; i<N; ++i)
            {
                // |vmax^2 - b| -> minimize (-bi)
                Jl += -b[i];

                // Weight for soft constraint
                Jq += over_v_weight*delta[i]*delta[i] + over_a_weight*sigma[i]*sigma[i];

                // pseudo jerk: d(ai)/ds -> minimize weight * (a1 - a0)^2
                if(i<N-1)
                    Jq += (smooth_weight/ds)*(a[i+1]-a[i])*(a[i+1]-a[i]);
            }

            model.setObjective(Jl + Jq, GRB_MINIMIZE);

            /**************************************************************/
            /**************************************************************/
            /**************** design constraint matrix ********************/
            /**************************************************************/
            /**************************************************************/
            for(int i=0; i<N; ++i)
            {
                // 0 < b - delta < vmax^2
                model.addConstr(0 <= b[i]-delta[i], "blconstraint"+std::to_string(i));
                model.addConstr(b[i]-delta[i] <= max_vels[i]*max_vels[i], "buconstraint"+std::to_string(i));

                // amin < a - sigma < amax
                model.addConstr(amin <= a[i] - sigma[i], "alconstraint" + std::to_string(i));
                model.addConstr(a[i] - sigma[i] <= amax, "auconstraint" + std::to_string(i));
            }

            for(int i=0; i<N-1; ++i)
                // b' = 2a ... (b(i+1) - b(i)) / ds = 2a(i)
                model.addConstr((b[i+1]-b[i])/ds == 2*a[i], "equality"+std::to_string(i));

            // Initial Condition
            model.addConstr(b[0]==initial_vel*initial_vel, "v0");
            model.addConstr(a[0]==initial_acc, "a0");

            /**************************************************************/
            /**************************************************************/
            /********************** Optimize ******************************/
            /**************************************************************/
            /**************************************************************/
            model.optimize();

            output.resize(N);
            for(unsigned int i=0; i<N; ++i)
            {
                output.velocity[i] = std::sqrt(std::max(b[i].get(GRB_DoubleAttr_X), 0.0));
                output.acceleration[i] = a[i].get(GRB_DoubleAttr_X);
            }

            for(unsigned int i=0; i<N-1; ++i)
            {
                double a_current = output.acceleration[i];
                double a_next    = output.acceleration[i+1];
                output.jerk[i] = (a_next - a_current) * output.velocity[i] / ds;
            }
            output.jerk[N-1] = output.jerk[N-2];
            std::cout << "QP Runtime: " << model.get(GRB_DoubleAttr_Runtime)*1e3 << "[ms]" << std::endl;
        }
        catch(GRBException& e)
        {
            std::cout << "Error code = " << e.getErrorCode() << std::endl;
            std::cout << e.getMessage() << std::endl;
            return false;
        }
        catch(...)
        {
            std::cout << "Exception during optimization" << std::endl;
            return false;
        }

        return true;
    }

    bool QPSolver::solveHardPseudo(const double& initial_vel,
                                   const double& initial_acc,
                                   const double& ds,
                                   const std::vector<double>& ref_vels,
                                   const std::vector<double>& max_vels,
                                   OutputInfo& output)
    {
        try
        {
            /* Create Environment */
            GRBEnv env = GRBEnv();
            GRBModel model = GRBModel(env);
            model.set(GRB_DoubleParam_TimeLimit, 100.0);
            model.set(GRB_DoubleParam_IterationLimit, 40000);
            model.set(GRB_DoubleParam_FeasibilityTol, 1e-4);
            model.set(GRB_DoubleParam_BarConvTol, 1e-4);
            model.set(GRB_DoubleParam_OptimalityTol, 1e-4);
            model.set(GRB_IntParam_OutputFlag, 0);

            int N = max_vels.size();
            const double amax = param_.max_accel;
            const double amin = param_.min_decel;
            const double smooth_weight = param_.smooth_weight;

            /*
             * x = [b0, b1, ..., bN, |  a0, a1, ..., aN] in R^{2N}
             * b: velocity^2
             * a: acceleration
             * sigma: amin < ai < amax
             */
            std::vector<GRBVar> b(N);
            std::vector<GRBVar> a(N);
            b[0] = model.addVar(initial_vel*initial_vel, initial_vel*initial_vel, 0.0, GRB_CONTINUOUS, "b0");
            a[0] = model.addVar(initial_acc, initial_acc, 0.0, GRB_CONTINUOUS, "a0");
            for(int i=1; i<N; ++i)
            {
                b[i] = model.addVar(0.0, max_vels[i]*max_vels[i], 0.0, GRB_CONTINUOUS, "b"+std::to_string(i));
                a[i] = model.addVar(amin, amax, 0.0, GRB_CONTINUOUS, "a"+std::to_string(i));
            }

            /**************************************************************/
            /**************************************************************/
            /**************** design objective function *******************/
            /**************************************************************/
            /**************************************************************/
            GRBLinExpr  Jl = 0.0;
            GRBQuadExpr Jq = 0.0;
            for(int i=0; i<N; ++i)
            {
                // |vmax^2 - b| -> minimize (-bi)
                Jl += -b[i];

                // pseudo jerk: d(ai)/ds -> minimize weight * (a1 - a0)^2
                if(i<N-1)
                    Jq += (smooth_weight/ds)*(a[i+1]-a[i])*(a[i+1]-a[i]);
            }

            model.setObjective(Jl + Jq, GRB_MINIMIZE);

            /**************************************************************/
            /**************************************************************/
            /**************** design constraint matrix ********************/
            /**************************************************************/
            /**************************************************************/
            for(int i=0; i<N-1; ++i)
                // b' = 2a ... (b(i+1) - b(i)) / ds = 2a(i)
                model.addConstr((b[i+1]-b[i])/ds == 2*a[i], "equality"+std::to_string(i));

            /**************************************************************/
            /**************************************************************/
            /********************** Optimize ******************************/
            /**************************************************************/
            /**************************************************************/
            model.optimize();

            output.resize(N);
            for(unsigned int i=0; i<N; ++i)
            {
                output.velocity[i] = std::sqrt(std::max(b[i].get(GRB_DoubleAttr_X), 0.0));
                output.acceleration[i] = a[i].get(GRB_DoubleAttr_X);
            }

            for(unsigned int i=0; i<N-1; ++i)
            {
                double a_current = output.acceleration[i];
                double a_next    = output.acceleration[i+1];
                output.jerk[i] = (a_next - a_current) * output.velocity[i] / ds;
            }
            output.jerk[N-1] = output.jerk[N-2];
            std::cout << "QP Runtime: " << model.get(GRB_DoubleAttr_Runtime)*1e3 << "[ms]" << std::endl;
        }
        catch(GRBException& e)
        {
            std::cout << "Error code = " << e.getErrorCode() << std::endl;
            std::cout << e.getMessage() << std::endl;
            return false;
        }
        catch(...)
        {
            std::cout << "Exception during optimization" << std::endl;
            return false;
        }

        return true;
    }
}

