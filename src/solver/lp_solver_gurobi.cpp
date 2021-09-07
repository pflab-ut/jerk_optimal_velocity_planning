#include "solver/lp_solver_gurobi.h"

namespace gurobi
{
    bool LPSolver::solveSoft(const double& initial_vel,
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
             *      | pdelta[0], pdelta[1], ..., pdelta[N]
             *      | psigma[0], psigma[1], ..., psigma[N]
             *      | pgamma[0], pgamma[1], ..., pgamma[N]
             *      | mdelta[0], mdelta[1], ..., mdelta[N]
             *      | msigma[0], msigma[1], ..., msigma[N]
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

            std::vector<GRBVar> b(N);
            std::vector<GRBVar> a(N);
            std::vector<GRBVar> pdelta(N); // positive delta
            std::vector<GRBVar> psigma(N); // positive sigma
            std::vector<GRBVar> pgamma(N); // positive gamma
            std::vector<GRBVar> mdelta(N); // minus delta
            std::vector<GRBVar> msigma(N); // minus sigma
            std::vector<GRBVar> mgamma(N); // minus gamma

            for(int i=0; i<N; ++i)
            {
                b[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "b"+std::to_string(i));
                a[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "a"+std::to_string(i));
                pdelta[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "positive_delta"+std::to_string(i));
                psigma[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "positive_sigma"+std::to_string(i));
                pgamma[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "positive_gamma"+std::to_string(i));
                mdelta[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "minus_delta"+std::to_string(i));
                msigma[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "minus_sigma"+std::to_string(i));
                mgamma[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "minus_gamma"+std::to_string(i));
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
            {
                double vmax = std::max(max_vels[i], 0.1);
                Jl += -b[i]/(vmax*vmax) + over_v_weight*(pdelta[i]+mdelta[i])
                      + over_a_weight*(psigma[i]+msigma[i])
                      + over_j_weight*(pgamma[i]+mgamma[i]);
            }

            model.setObjective(Jl, GRB_MINIMIZE);

            /**************************************************************/
            /**************************************************************/
            /**************** design constraint matrix ********************/
            /**************************************************************/
            /**************************************************************/
            for(int i=0; i<N; ++i)
            {
                // 0 < b - delta < vmax^2
                model.addConstr(0 <= b[i]-(pdelta[i]-mdelta[i]), "blconstraint"+std::to_string(i));
                model.addConstr(b[i]-(pdelta[i]-mdelta[i]) <= max_vels[i]*max_vels[i], "buconstraint"+std::to_string(i));

                // amin < a - sigma < amax
                model.addConstr(amin <= a[i] - (psigma[i]-msigma[i]), "alconstraint" + std::to_string(i));
                model.addConstr(a[i] - (psigma[i]-msigma[i]) <= amax, "auconstraint" + std::to_string(i));
            }

            for(int i=0; i<N-1; ++i)
            {
                // Soft Constraint Jerk Limit: jerk_min < pseudo_jerk[i] * ref_vel[i] - gamma[i] < jerk_max
                // -> jerk_min * ds < (a[i+1] - a[i]) * ref_vel[i] - gamma[i] * ds < jerk_max * ds
                model.addConstr(jmin * ds <= (a[i+1] - a[i])*ref_vels[i] - (pgamma[i]-mgamma[i])*ds, "jlconstraint"+std::to_string(i));
                model.addConstr((a[i+1] - a[i])*ref_vels[i] - (pgamma[i]-mgamma[i])*ds <= jmax*ds, "juconstraint"+std::to_string(i));

                // b' = 2a ... (b(i+1) - b(i)) / ds = 2a(i)
                model.addConstr((b[i+1]-b[i]) == 2*a[i]*ds, "equality"+std::to_string(i));
            }

            // Initial Condition
            model.addConstr(b[0]==initial_vel*initial_vel, "initial_velocity");
            model.addConstr(a[0]==initial_acc, "initial_acc");
            model.addConstr(a[N-1]==0.0, "terminal_acc");

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

            std::cout << "LP Runtime: " << model.get(GRB_DoubleAttr_Runtime)*1e3 << "[ms]" << std::endl;
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

    bool LPSolver::solveHard(const double& initial_vel,
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
            const double amax = param_.max_accel;
            const double amin = param_.min_decel;
            const double jmax = param_.max_jerk;
            const double jmin = param_.min_jerk;

            /*
             * x = [b[0], b[1], ..., b[N] | a[0], a[1], .... a[N]]
             * b[i]: velocity^2
             * 0 < b[i] < max_vel[i]*max_vel[i]
             * amin < a[i] < amax
             * jerk_min/ref_vel[i] < pseudo_jerk[i] < jerk_max/ref_vel[i]
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
            a[N-1] = model.addVar(0.0, 0.0, 0.0, GRB_CONTINUOUS, "a"+std::to_string(N-1));

            /**************************************************************/
            /**************************************************************/
            /**************** design objective function *******************/
            /**************************************************************/
            /**************************************************************/
            GRBLinExpr  Jl = 0.0;
            for(int i=0; i<N; ++i)
            {
                double vmax = std::max(max_vels[i], 0.1);
                Jl += -b[i]/(vmax*vmax);
            }

            model.setObjective(Jl, GRB_MINIMIZE);

            /**************************************************************/
            /**************************************************************/
            /**************** design constraint matrix ********************/
            /**************************************************************/
            /**************************************************************/
            for(int i=0; i<N-1; ++i)
            {
                // Soft Constraint Jerk Limit: jerk_min < pseudo_jerk[i] * ref_vel[i] < jerk_max
                // -> jerk_min * ds < (a[i+1] - a[i]) * ref_vel[i] < jerk_max * ds
                model.addConstr(jmin * ds <= (a[i+1] - a[i])*ref_vels[i], "jlconstraint"+std::to_string(i));
                model.addConstr((a[i+1] - a[i])*ref_vels[i] <= jmax * ds, "juconstraint"+std::to_string(i));

                // b' = 2a ... (b(i+1) - b(i)) / ds = 2a(i)
                model.addConstr((b[i+1]-b[i]) == 2*a[i]*ds, "equality"+std::to_string(i));
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

            std::cout << "LP Runtime: " << model.get(GRB_DoubleAttr_Runtime)*1e3 << "[ms]" << std::endl;
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