#include <iostream>
#include <vector>
#include <iomanip>
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

    std::string obs_filtered_filename = "../result/pseudo_jerk/obs_filtered.csv";
    Utils::outputVelocityToFile(obs_filtered_filename, data.positions_, data.max_velocities_, obs_filtered_vels);

    std::string st_filename = "../result/pseudo_jerk/st_graph.csv";
    Utils::outputSTToFile(st_filename, data.positions_, data.max_velocities_, obs_filtered_vels, data.obs_);

    /***************************************************/
    /*************** QP Optimization +******************/
    /***************************************************/
    BaseSolver::OptimizerParam param{};
    param.max_accel = 1.0;
    param.min_decel = -1.0;
    param.max_jerk = 0.8;
    param.min_jerk = -0.8;
    param.smooth_weight = 500.0;
    param.over_j_weight = 1000;
    param.over_a_weight = 1000;
    param.over_v_weight = 1000;
    Optimizer optimizer(Optimizer::OptimizerSolver::GUROBI_QP, param);
    BaseSolver::OutputInfo output;
    bool is_hard = true;

    bool result = optimizer.solvePseudo(is_hard, data.v0_, data.a0_, data.ds_, obs_filtered_vels, obs_filtered_vels, output);

    if(result)
    {
        std::string qp_filename = "../result/pseudo_jerk/qp_result.csv";
        Utils::outputResultToFile(qp_filename, data.positions_, output.velocity, output.acceleration, output.jerk, obs_filtered_vels);
    }
    else
        std::cerr << "[Solver Error]: Solver has some error" << std::endl;

    return 0;
}