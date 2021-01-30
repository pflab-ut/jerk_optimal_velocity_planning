#ifndef FILTER_POSITION_OPTIMIZATION_FILTER_H
#define FILTER_POSITION_OPTIMIZATION_FILTER_H

#include <vector>
#include <cmath>

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
};

#endif //FILTER_POSITION_OPTIMIZATION_FILTER_H
