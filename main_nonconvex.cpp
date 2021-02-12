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
    const int N = 100;
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

    for(int i=0; i<30; ++i)
        original_vel[i] = 2.0;
    for(int i=30; i<80; ++i)
        original_vel[i] = 3.0;
    for(int i=80; i<N; ++i)
        original_vel[i] = 1.0;
    original_vel.back() = 0.0;

    /***************************************************/
    /*************** Filter Velocity *******************/
    /***************************************************/
    Filter vel_filter;
    std::vector<double> filtered_vel;
    std::vector<double> filtered_acc;
    vel_filter.smoothVelocity(ds, initial_vel, initial_acc, max_acc, jerk_acc, original_vel, filtered_vel, filtered_acc);

    /***************************************************/
    /*************** QP Optimization +******************/
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

    std::chrono::system_clock::time_point  start, end;
    start = std::chrono::system_clock::now();

    optimizer.solve(initial_vel, initial_acc, ds, filtered_vel, filtered_vel, output);

    end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    std::cout << "Calulation Time: " << elapsed << "[ms]" << std::endl;

    for(int i=0; i<original_vel.size(); ++i)
        std::cout << std::fixed << "s[" << i << "]" << std::setprecision(1) << position[i]
                  << "   v[" << i << "]: " << std::setprecision(3) << original_vel[i]
                  << "   Filtered Velocity: " << std::setprecision(3) << filtered_vel[i]
                  << "   nc_velocity: " << std::setprecision(5) << output.velocity[i]
                  << "   nc_acceleration: " << std::setprecision(5) << output.acceleration[i]
                  << "   nc_jerk: " << std::setprecision(5) << output.jerk[i] << std::endl;

    std::string nc_filename = "../result/nonconvex_jerk/nc_result.csv";
    std::string velocity_filename = "../result/nonconvex_jerk/reference_velocity.csv";
    Utils::outputVelocityToFile(velocity_filename, position, original_vel, filtered_vel, filtered_acc);
    Utils::outputResultToFile(nc_filename, position, output.velocity, output.acceleration, output.jerk);

    return 0;
};
