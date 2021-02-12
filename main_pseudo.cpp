#include <iostream>
#include <vector>
#include <iomanip>
#include "interpolate.h"
#include "filter.h"
#include "optimizer.h"
#include "utils.h"
#include "obstacle.h"

int main()
{
    /***************************************************/
    /*************** Velocity Profile ******************/
    /***************************************************/
    const int N = 300;
    const double initial_vel = 0.5;
    const double initial_acc = 0.0;
    const double ds = 0.1;
    const double max_acc= 1.0;
    const double jerk_acc = 0.8;

    std::vector<double> position(N, 0.0);
    std::vector<double> original_vel(N, 0.0);

    // position
    for(int i=0; i<N; ++i)
        position[i] = i*ds;

    for(int i=0; i<100; ++i)
        original_vel[i] = 3.0;
    for(int i=100; i<200; ++i)
        original_vel[i] = 5.0;
    for(int i=200; i<N; ++i)
        original_vel[i] = 4.0;
    original_vel.back() = 0.0;

    /***************************************************/
    /******************* Obstacle **********************/
    /***************************************************/
    const int obs_size = 50;
    const double obs_v = 2.0;
    const double dt = 0.1;
    const double s0 = 10.0;
    const double t0 = 2.0;
    Obstacle obs(obs_size, obs_v, dt, s0, t0);

    /***************************************************/
    /********** Obstacle Filter Velocity ***************/
    /***************************************************/
    Filter vel_filter;
    std::vector<double> obs_filtered_vels;
    vel_filter.obstacleVelocityLimitFilter(initial_vel, position, original_vel, obs, obs_filtered_vels);

    std::string obs_filtered_filename = "../result/pseudo_jerk/obs_filtered.csv";
    Utils::outputVelocityToFile(obs_filtered_filename, position, original_vel, obs_filtered_vels);

    std::string st_filename = "../result/pseudo_jerk/st_graph.csv";
    Utils::outputSTToFile(st_filename, position, original_vel, obs_filtered_vels, obs);

    /***************************************************/
    /*************** QP Optimization +******************/
    /***************************************************/
    BaseSolver::OptimizerParam param{};
    param.max_accel = 1.0;
    param.min_decel = -1.0;
    param.max_jerk = 0.8;
    param.min_jerk = -0.8;
    param.smooth_weight = 100.0;
    param.over_j_weight = 1000;
    param.over_a_weight = 1000;
    param.over_v_weight = 1000;
    Optimizer optimizer(Optimizer::OptimizerSolver::GUROBI_QP, param);
    BaseSolver::OutputInfo output;
    bool is_hard = true;

    bool result = optimizer.solvePseudo(is_hard, initial_vel, initial_acc, ds, obs_filtered_vels, obs_filtered_vels, output);

    if(result)
    {
        std::string qp_filename = "../result/pseudo_jerk/qp_result.csv";
        Utils::outputResultToFile(qp_filename, position, output.velocity, output.acceleration, output.jerk, obs_filtered_vels);
    }
    else
        std::cerr << "[Solver Error]: Solver has some error" << std::endl;

    return 0;
}