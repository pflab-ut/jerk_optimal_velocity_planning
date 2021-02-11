#include "solver/lp_solver_gurobi.h"

namespace gurobi
{
    bool LPSolver::solve(const double& initial_vel,
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
            //model.set(GRB_IntParam_Method, 2);

            assert(ref_vels.size()==max_vels.size());
            int N = ref_vels.size();

            /*
             * x = [b[0], b[1], ..., b[N] | a[0], a[1], .... a[N] | delta[0], ..., delta[N]
             *      | sigma[0], sigma[1], ...., sigma[N] | gamma[0], gamma[1], ..., gamma[N]
             *      | abs_delta[0], abs_delta[1], ..., abs_delta[N] | abs_sigma[0], ..., abs_sigma[N]
             *      | abs_gamma[0], abs_gamma[1], ..., abs_gamma[N] ]
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
            std::vector<GRBVar> abs_delta(N);
            std::vector<GRBVar> abs_sigma(N);
            std::vector<GRBVar> abs_gamma(N);

            for(int i=0; i<N; ++i)
            {
                b[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "b"+std::to_string(i));
                a[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "a"+std::to_string(i));
                delta[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "delta"+std::to_string(i));
                sigma[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "sigma"+std::to_string(i));
                gamma[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "gamma"+std::to_string(i));
                abs_delta[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "abs_delta"+std::to_string(i));
                abs_sigma[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "abs_sigma"+std::to_string(i));
                abs_gamma[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "abs_gamma"+std::to_string(i));
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
            for(int i=0; i<N; ++i)
                Jl += -b[i] + over_v_weight*abs_delta[i] + over_a_weight*abs_sigma[i] + over_j_weight*abs_gamma[i];

            model.setObjective(Jl, GRB_MINIMIZE);

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

                // b' = 2a ... (b(i+1) - b(i)) / ds = 2a(i)
                model.addConstr((b[i+1]-b[i]) == 2*a[i]*ds, "equality"+std::to_string(i));
            }

            // Initial Condition
            model.addConstr(b[0]==initial_vel*initial_vel, "v0");
            model.addConstr(a[0]==initial_acc, "a0");

            // Absolute Value Constraint
            for(int i=0; i<N; ++i)
            {
                // abs_delta[i] = |delta[i]|
                model.addGenConstrAbs(abs_delta[i], delta[i]);

                // abs_sigma[i] = |sigma[i]|
                model.addGenConstrAbs(abs_sigma[i], sigma[i]);

                // abs_gamma[i] = |gamma[i]|
                model.addGenConstrAbs(abs_gamma[i], gamma[i]);
            }

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

    bool LPSolver::solve(const double& initial_vel,
                         const double& initial_acc,
                         const double& ds,
                         const std::vector<double>& max_vels,
                         OutputInfo& output)
    {
        return true;
    }
}