#ifndef FILTER_POSITION_OPTIMIZATION_FILTER_H
#define FILTER_POSITION_OPTIMIZATION_FILTER_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "obstacle.h"
#include "interpolate.h"

class Filter
{
public:
    struct OutputInfo
    {
        std::vector<double> time;
        std::vector<double> position;
        std::vector<double> velocity;

        void reserve(const unsigned int& N)
        {
            time.reserve(N);
            velocity.reserve(N);
            position.reserve(N);
        }

        void resize(const unsigned int& N)
        {
            time.resize(N);
            velocity.resize(N);
            position.reserve(N);
        }
    };
    Filter() = default;
    ~Filter() = default;

    void smoothVelocity(const double& ds,
                        const double& initial_vel,
                        const double& initial_acc,
                        const double& max_acc,
                        const double& jerk_acc,
                        const std::vector<double>& original_vel,
                        std::vector<double>& filtered_vel,
                        std::vector<double>& filtered_acc);

    void mergeFilteredVelocity(const std::vector<double>& forward_vels,
                               const std::vector<double>& backeard_vels,
                               std::vector<double>& merged_vels);

    bool obstacleVelocityLimitFilter(const double& initial_vel,
                                     const std::vector<double>& input_arclength,
                                     const std::vector<double>& max_vels,
                                     const Obstacle& obstacle,
                                     Filter::OutputInfo& output_data);
};

#endif //FILTER_POSITION_OPTIMIZATION_FILTER_H
