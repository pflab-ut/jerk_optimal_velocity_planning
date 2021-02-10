#ifndef FILTER_POSITION_OPTIMIZATION_QP_INTERFACE_H
#define FILTER_POSITION_OPTIMIZATION_QP_INTERFACE_H

#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>
#include <chrono>
#include "solver_interface/osqp_interface.h"
#include "interpolate.h"
#include "osqp.h"

class QPOptimizer
{
public:
    struct OptimizerParam
    {
        double max_accel;
        double min_decel;
        double max_jerk;
        double min_jerk;
        double smooth_weight;
        double over_j_weight;
        double over_v_weight;
        double over_a_weight;
    };

    struct QPOutputInfo
    {
        std::vector<double> qp_time;
        std::vector<double> qp_velocity;
        std::vector<double> qp_acceleration;
        std::vector<double> qp_jerk;

        void reserve(const unsigned int& N)
        {
            qp_time.reserve(N);
            qp_velocity.reserve(N);
            qp_acceleration.reserve(N);
            qp_jerk.reserve(N);
        }

        void resize(const unsigned int& N)
        {
            qp_time.resize(N);
            qp_velocity.resize(N);
            qp_acceleration.resize(N);
            qp_jerk.resize(N);
        }
    };

    QPOptimizer(const OptimizerParam& param);

    void setParam(const OptimizerParam& param);

    bool solve(const double& initial_vel,
               const double& initial_acc,
               const double& ds,
               const std::vector<double>& ref_vels,
               const std::vector<double>& max_vels,
               QPOutputInfo& qp_output);

    bool solve(const double& initial_vel,
               const double& initial_acc,
               const double& ds,
               const std::vector<double>& max_vels,
               QPOutputInfo& qp_output);

private:
    OptimizerParam param_;
    osqp::OSQPInterface qp_solver_;
};

#endif //FILTER_POSITION_OPTIMIZATION_QP_INTERFACE_H
