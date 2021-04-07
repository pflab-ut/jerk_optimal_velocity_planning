#include <iostream>
#include <vector>
#include <iomanip>
#include <chrono>
#include "interpolate.h"
#include "filter.h"
#include "optimizer.h"
#include "utils.h"
#include "obstacle.h"
#include "scenario_generator.h"

int main()
{
    const std::string current_dir = std::string(RESULT_DIR);
    std::cout << current_dir << std::endl;

    ScenarioGenerator::ScenarioNumber num = ScenarioGenerator::Stop;
    ScenarioGenerator generator;

    ScenarioGenerator::ScenarioData data;
    generator.generate(num, data);

    /***************************************************/
    /********** Obstacle Filter Velocity ***************/
    /***************************************************/
    Filter vel_filter;
    std::vector<double> obs_filtered_vels;
    std::vector<double> obs_filtered_times;

    std::chrono::system_clock::time_point  start_obs, end_obs;
    start_obs = std::chrono::system_clock::now();
    vel_filter.obstacleVelocityLimitFilter(data.v0_, data.positions_, data.max_velocities_, data.obs_,
                                           obs_filtered_vels, obs_filtered_times);

    end_obs = std::chrono::system_clock::now();
    double elapsed_obs = std::chrono::duration_cast<std::chrono::nanoseconds>(end_obs - start_obs).count();
    std::cout << "Obstacle Calulation Time: " << elapsed_obs /1000000 << "[ms]" << std::endl;

    std::string obs_filename = current_dir + "/result/obs.csv";
    Utils::outputObsToFile(obs_filename, data.obs_);

    /***************************************************/
    /************* Jerk Filter Velocity ****************/
    /***************************************************/
    std::vector<double> jerk_filtered_vels;
    std::vector<double> jerk_filtered_accs;

    std::chrono::system_clock::time_point  start_filter, end_filter;
    start_filter = std::chrono::system_clock::now();

    vel_filter.smoothVelocity(data.ds_, data.v0_, data.a0_, data.max_acc_, data.max_jerk_, obs_filtered_vels, jerk_filtered_vels, jerk_filtered_accs);

    end_filter = std::chrono::system_clock::now();
    double elapsed_filter = std::chrono::duration_cast<std::chrono::nanoseconds>(end_filter-start_filter).count();
    std::cout << "Filter Calulation Time: " << elapsed_filter/1000000 << "[ms]" << std::endl;

    /***************************************************/
    /***************** LP Parameter ********************/
    /***************************************************/
    BaseSolver::OptimizerParam param{};
    param.max_accel = 1.0;
    param.min_decel = -1.0;
    param.max_jerk = 0.8;
    param.min_jerk = -0.8;
    param.over_j_weight = 1000;
    param.over_a_weight = 1000;
    param.over_v_weight = 1000;
    param.smooth_weight = 500.0;
    bool is_hard = true;

    /***************************************************/
    /*************** LP Optimization *******************/
    /***************************************************/
    Optimizer lp_optimizer(Optimizer::OptimizerSolver::GUROBI_LP, param);
    BaseSolver::OutputInfo lp_output;
    lp_output.position = data.positions_;

    std::chrono::system_clock::time_point lp_start, lp_end;
    lp_start = std::chrono::system_clock::now();

    bool lp_result = lp_optimizer.solve(is_hard, data.v0_, data.a0_, data.ds_, jerk_filtered_vels, jerk_filtered_vels, lp_output);

    lp_end = std::chrono::system_clock::now();
    double lp_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(lp_end-lp_start).count();

    if(!lp_result)
    {
        std::cerr << "LP Solver has Error" << std::endl;
        return -1;
    }

    std::cout << "LP Solver Calculation Time: " << lp_elapsed << "[ms]" << std::endl;

    /***************************************************/
    /********** QP Optimization(Pseudo-Jerk) ***********/
    /***************************************************/
    Optimizer qp_optimizer(Optimizer::OptimizerSolver::GUROBI_QP, param);
    BaseSolver::OutputInfo qp_output;
    qp_output.position = data.positions_;

    std::chrono::system_clock::time_point qp_start, qp_end;
    qp_start = std::chrono::system_clock::now();

    bool qp_result = qp_optimizer.solvePseudo(is_hard, data.v0_, data.a0_, data.ds_, obs_filtered_vels, obs_filtered_vels, qp_output);

    qp_end = std::chrono::system_clock::now();
    double qp_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(qp_end-qp_start).count();

    if(!qp_result)
    {
        std::cerr << "QP Solver has Error" << std::endl;
        return -1;
    }

    std::cout << "QP Solver Calculation Time: " << qp_elapsed << "[ms]" << std::endl;

    /***************************************************/
    /************* Non-Convex Optimization *************/
    /***************************************************/
    Optimizer nc_optimizer(Optimizer::OptimizerSolver::NLOPT_NC, param);
    BaseSolver::OutputInfo nc_output;
    nc_output.position = data.positions_;

    std::chrono::system_clock::time_point nc_start, nc_end;
    nc_start = std::chrono::system_clock::now();

    bool nc_result = nc_optimizer.solve(is_hard, data.v0_, data.a0_, data.ds_, obs_filtered_vels, obs_filtered_vels, nc_output);

    nc_end = std::chrono::system_clock::now();
    double nc_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(nc_end-nc_start).count();

    if(!nc_result)
    {
        std::cerr << "Non-Convex Solver has Error" << std::endl;
        return -1;
    }

    std::cout << "Non-Convex Solver Calculation Time: " << nc_elapsed << "[ms]" << std::endl;

    std::string filename = current_dir + "/result/optimization_result.csv";
    Utils::outputToFile(filename, data.positions_,
                        data.max_velocities_, obs_filtered_vels, obs_filtered_times, jerk_filtered_vels,
                        lp_output, qp_output, nc_output);

    return 0;
}
