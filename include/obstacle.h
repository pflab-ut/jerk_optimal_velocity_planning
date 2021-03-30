#ifndef FILTER_POSITION_OPTIMIZATION_OBSTACLE_H
#define FILTER_POSITION_OPTIMIZATION_OBSTACLE_H

#include <vector>
#include <cmath>

class Obstacle
{
public:
    Obstacle() = default;

    Obstacle(const int& N, const double& v, const double& dt, const double& s0, const double& t0)
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

        // Generate extend_s and extend_t
        double s_ini;
        double t_ini;
        if(std::fabs(v)<1e-6)
        {
            t_ini = 0.0;
            s_ini = s0;
        }
        else
        {
            s_ini = std::min(0.0, s0 - v*t0);
            t_ini = t0 - (s0-s_ini)/v;
        }

        int extend_N = N + static_cast<int>(std::fabs(t0-t_ini)/dt);
        extend_s_.reserve(extend_N);
        extend_t_.reserve(extend_N);
        extend_s_.push_back(s_ini);
        extend_t_.push_back(t_ini);
        for(int i=0; i<extend_N; ++i)
        {
            double s_next = extend_s_.back() + v*dt;
            double t_next = extend_t_.back() + dt;
            extend_s_.push_back(s_next);
            extend_t_.push_back(t_next);
        }
    }

    std::vector<double> s_;
    std::vector<double> t_;
    std::vector<double> extend_s_;
    std::vector<double> extend_t_;
};

#endif //FILTER_POSITION_OPTIMIZATION_OBSTACLE_H
