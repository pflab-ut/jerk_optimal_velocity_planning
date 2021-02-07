#ifndef FILTER_POSITION_OPTIMIZATION_OBSTACLE_H
#define FILTER_POSITION_OPTIMIZATION_OBSTACLE_H

#include <vector>
#include <cmath>

class Obstacle
{
public:
    Obstacle() = default;

    Obstacle(const int& N, const double v, const double& dt, const double& s0, const double& t0)
    {
        s_.resize(N);
        t_.resize(N);

        s_.front() = s0;
        t_.front() = t0;
        for(unsigned i=1; i<s_.size(); ++i)
        {
            s_[i] = s_[i-1] + v * dt;
            t_[i] = t_[i-1] + dt;
        }
    }

    std::vector<double> s_;
    std::vector<double> t_;
};

#endif //FILTER_POSITION_OPTIMIZATION_OBSTACLE_H
