#include <iostream>
#include <vector>
#include <iomanip>
#include "interpolate.h"
#include "filter.h"
#include "qp_interface.h"
#include "utils.h"

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
    /*************** Filter Velocity *******************/
    /***************************************************/
    std::vector<double> filtered_vel;
    std::vector<double> filtered_acc;
    Filter vel_filter;
    vel_filter.smoothVelocity(ds, initial_vel, initial_acc, max_acc, jerk_acc, original_vel, filtered_vel, filtered_acc);

    for(int i=0; i<original_vel.size(); ++i)
        std::cout << std::fixed << "s[" << i << "]" << std::setprecision(1) << position[i]
                  << "   v[" << i << "]: " << std::setprecision(3) << original_vel[i]
                  << "   Filtered Velocity: " << std::setprecision(3) << filtered_vel[i]
                  << "   Acc: " << std::setprecision(5) << filtered_acc[i] << std::endl;

    /***************************************************/
    /*************** QP Optimization +******************/
    /***************************************************/
    QPOptimizer::OptimizerParam param{};
    param.max_accel = 1.0;
    param.min_decel = -1.0;
    param.max_jerk = 0.8;
    param.min_jerk = -0.8;
    param.smooth_weight = 0.0;
    param.over_j_weight = 1000;
    param.over_a_weight = 1000;
    param.over_v_weight = 1000;
    QPOptimizer qp_optimizer(param);

    QPOptimizer::QPOutputInfo qp_output;
    //qp_optimizer.solve(initial_vel, initial_acc, ds, filtered_vel, original_vel, filtered_acc, qp_output);
    qp_optimizer.solve(initial_vel, initial_acc, ds, filtered_vel, filtered_vel, filtered_acc, qp_output);

    for(int i=0; i<original_vel.size(); ++i)
        std::cout << std::fixed << "s[" << i << "]" << std::setprecision(1) << position[i]
                  << "   v[" << i << "]: " << std::setprecision(3) << original_vel[i]
                  << "   Filtered Velocity: " << std::setprecision(3) << filtered_vel[i]
                  << "   qp_velocity: " << std::setprecision(5) << qp_output.qp_velocity[i]
                  << "   filtered_acceleration: " << std::setprecision(5) << filtered_acc[i]
                  << "   qp_acceleration: " << std::setprecision(5) << qp_output.qp_acceleration[i]
                  << "   qp_jerk: " << std::setprecision(5) << qp_output.qp_jerk[i] << std::endl;

    std::string qp_filename = "../result/qp_result.csv";
    std::string velocity_filename = "../result/reference_velocity.csv";
    Utils::outputVelocityToFile(velocity_filename, position, original_vel, filtered_vel, filtered_acc);
    Utils::outputResultToFile(qp_filename, position, qp_output.qp_velocity, qp_output.qp_acceleration, qp_output.qp_jerk);

    return 0;
}
