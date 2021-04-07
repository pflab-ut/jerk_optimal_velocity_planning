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
                                     std::vector<double>& filtered_vels,
                                     std::vector<double>& filtered_time);
};

#endif //FILTER_POSITION_OPTIMIZATION_FILTER_H
