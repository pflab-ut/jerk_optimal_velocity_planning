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
    ScenarioGenerator::ScenarioNumber num = ScenarioGenerator::Normal;
    ScenarioGenerator generator;

    ScenarioGenerator::ScenarioData data;
    generator.generate(num, data);

    /***************************************************/
    /********** Obstacle Filter Velocity ***************/
    /***************************************************/
    Filter vel_filter;
    std::vector<double> obs_filtered_vels;

    std::chrono::system_clock::time_point  start_obs, end_obs;
    start_obs = std::chrono::system_clock::now();
    vel_filter.obstacleVelocityLimitFilter(data.v0_, data.positions_, data.max_velocities_, data.obs_, obs_filtered_vels);

    end_obs = std::chrono::system_clock::now();
    double elapsed_obs = std::chrono::duration_cast<std::chrono::nanoseconds>(end_obs - start_obs).count();
    std::cout << "Obstacle Calulation Time: " << elapsed_obs /1000000 << "[ms]" << std::endl;

    std::string obs_filtered_filename = "../result/filter_qp/obs_filtered.csv";
    Utils::outputVelocityToFile(obs_filtered_filename, data.positions_, data.max_velocities_, obs_filtered_vels);

    std::string st_filename = "../result/filter_qp/st_graph.csv";
    Utils::outputSTToFile(st_filename, data.positions_, data.max_velocities_, obs_filtered_vels, data.obs_);

    /***************************************************/
    /*************** Filter Velocity *******************/
    /***************************************************/
    std::vector<double> filtered_vel;
    std::vector<double> filtered_acc;

    std::chrono::system_clock::time_point  start_filter, end_filter;
    start_filter = std::chrono::system_clock::now();

    vel_filter.smoothVelocity(data.ds_, data.v0_, data.a0_, data.max_acc_, data.max_jerk_, obs_filtered_vels, filtered_vel, filtered_acc);
    //vel_filter.smoothVelocity(ds, initial_vel, initial_acc, max_acc, jerk_acc, filtered_vel, filtered_vel, filtered_acc);

    end_filter = std::chrono::system_clock::now();
    double elapsed_filter = std::chrono::duration_cast<std::chrono::nanoseconds>(end_filter-start_filter).count();
    std::cout << "Filter Calulation Time: " << elapsed_filter/1000000 << "[ms]" << std::endl;

    /***************************************************/
    /*************** QP Optimization +******************/
    /***************************************************/
    BaseSolver::OptimizerParam param{};
    param.max_accel = 1.0;
    param.min_decel = -1.0;
    param.max_jerk = 0.8;
    param.min_jerk = -0.8;
    param.over_j_weight = 1000;
    param.over_a_weight = 1000;
    param.over_v_weight = 1000;

    Optimizer optimizer(Optimizer::OptimizerSolver::GUROBI_LP, param);
    bool is_hard = true;
    BaseSolver::OutputInfo output;

    std::chrono::system_clock::time_point  start, end;
    start = std::chrono::system_clock::now();

    bool result = optimizer.solve(is_hard, data.v0_, data.a0_, data.ds_, filtered_vel, filtered_vel, output);

    end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    std::cout << "Calulation Time: " << elapsed << "[ms]" << std::endl;

    if(result)
    {
        std::string qp_filename = "../result/filter_qp/qp_result.csv";
        std::string velocity_filename = "../result/filter_qp/reference_velocity.csv";
        Utils::outputVelocityToFile(velocity_filename, data.positions_, obs_filtered_vels, filtered_vel, filtered_acc);
        Utils::outputResultToFile(qp_filename, data.positions_, output.velocity, output.acceleration, output.jerk);
    }
    else
        std::cerr << "Solver Failure" << std::endl;

    return 0;
}
