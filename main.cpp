#include <iostream>
#include <vector>
#include <iomanip>
#include <chrono>
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

    std::string obs_filtered_filename = "../result/filter_qp/obs_filtered.csv";
    Utils::outputVelocityToFile(obs_filtered_filename, position, original_vel, obs_filtered_vels);

    std::string st_filename = "../result/filter_qp/st_graph.csv";
    Utils::outputSTToFile(st_filename, position, original_vel, obs_filtered_vels, obs);

    /***************************************************/
    /*************** Filter Velocity *******************/
    /***************************************************/
    std::vector<double> filtered_vel;
    std::vector<double> filtered_acc;
    vel_filter.smoothVelocity(ds, initial_vel, initial_acc, max_acc, jerk_acc, obs_filtered_vels, filtered_vel, filtered_acc);
    //vel_filter.smoothVelocity(ds, initial_vel, initial_acc, max_acc, jerk_acc, filtered_vel, filtered_vel, filtered_acc);

    for(int i=0; i<original_vel.size(); ++i)
        std::cout << std::fixed << "s[" << i << "]" << std::setprecision(1) << position[i]
                  << "   v[" << i << "]: " << std::setprecision(3) << original_vel[i]
                  << "   Filtered Velocity: " << std::setprecision(3) << filtered_vel[i]
                  << "   Acc: " << std::setprecision(5) << filtered_acc[i] << std::endl;

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

    bool result = optimizer.solve(is_hard, initial_vel, initial_acc, ds, filtered_vel, filtered_vel, output);

    end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    std::cout << "Calulation Time: " << elapsed << "[ms]" << std::endl;

    if(result)
    {
        /*
        for(int i=0; i<original_vel.size(); ++i)
            std::cout << std::fixed << "s[" << i << "]" << std::setprecision(1) << position[i]
                      << "   v[" << i << "]: " << std::setprecision(3) << original_vel[i]
                      << "   Filtered Velocity: " << std::setprecision(3) << filtered_vel[i]
                      << "   qp_velocity: " << std::setprecision(5) << output.velocity[i]
                      << "   filtered_acceleration: " << std::setprecision(5) << filtered_acc[i]
                      << "   qp_acceleration: " << std::setprecision(5) << output.acceleration[i]
                      << "   qp_jerk: " << std::setprecision(5) << output.jerk[i] << std::endl;
         */

        std::string qp_filename = "../result/filter_qp/qp_result.csv";
        std::string velocity_filename = "../result/filter_qp/reference_velocity.csv";
        //Utils::outputVelocityToFile(velocity_filename, position, original_vel, filtered_vel, filtered_acc);
        Utils::outputVelocityToFile(velocity_filename, position, obs_filtered_vels, filtered_vel, filtered_acc);
        Utils::outputResultToFile(qp_filename, position, output.velocity, output.acceleration, output.jerk);
    }
    else
        std::cerr << "Solver Failure" << std::endl;

    return 0;
}
