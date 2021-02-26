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
    vel_filter.obstacleVelocityLimitFilter(data.v0_, data.positions_, data.max_velocities_, data.obs_, obs_filtered_vels);

    std::string obs_filtered_filename = "../result/nonconvex_jerk/obs_filtered.csv";
    Utils::outputVelocityToFile(obs_filtered_filename, data.positions_, data.max_velocities_, obs_filtered_vels);

    std::string st_filename = "../result/nonconvex_jerk/st_graph.csv";
    Utils::outputSTToFile(st_filename, data.positions_, data.max_velocities_, obs_filtered_vels, data.obs_);

    /***************************************************/
    /*************** Filter Velocity *******************/
    /***************************************************/
    std::vector<double> filtered_vel;
    std::vector<double> filtered_acc;
    vel_filter.smoothVelocity(data.ds_, data.v0_, data.a0_, data.max_acc_, data.max_jerk_, obs_filtered_vels, filtered_vel, filtered_acc);

    /***************************************************/
    /*************** Nonconvex Optimization +***********/
    /***************************************************/
    BaseSolver::OptimizerParam param{};
    param.max_accel = 1.0;
    param.min_decel = -1.0;
    param.max_jerk = 0.8;
    param.min_jerk = -0.8;
    param.over_v_weight = 1000;
    param.over_a_weight = 1000;
    param.over_j_weight = 1000;

    Optimizer optimizer(Optimizer::OptimizerSolver::NLOPT_NC, param);
    BaseSolver::OutputInfo output;
    bool is_hard = true;

    std::chrono::system_clock::time_point  start, end;
    start = std::chrono::system_clock::now();

    bool result = optimizer.solve(is_hard, data.v0_, data.a0_, data.ds_, obs_filtered_vels, obs_filtered_vels, output);

    end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    std::cout << "Calulation Time: " << elapsed << "[ms]" << std::endl;

    if(result)
    {
        std::string nc_filename = "../result/nonconvex_jerk/nc_result.csv";
        std::string velocity_filename = "../result/nonconvex_jerk/reference_velocity.csv";
        Utils::outputVelocityToFile(velocity_filename, data.positions_, obs_filtered_vels, obs_filtered_vels, obs_filtered_vels);
        Utils::outputResultToFile(nc_filename, data.positions_, output.velocity, output.acceleration, output.jerk);
    }
    else
        std::cerr << "Solver Failure" << std::endl;


    return 0;
};
