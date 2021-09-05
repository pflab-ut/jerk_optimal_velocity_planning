#ifndef FILTER_POSITION_OPTIMIZATION_OBSTACLE_H
#define FILTER_POSITION_OPTIMIZATION_OBSTACLE_H

#include <cassert>
#include <numeric>
#include <cmath>
#include <limits>
#include <iostream>

class Obstacle
{
public:
    Obstacle();
    Obstacle(const double& t_ini,
             const double& t_end,
             const double& s_ini,
             const double& s_end,
             const double& width,
             const double& length,
             const double& margin_s);
    ~Obstacle() = default;

    void calculateYIntercept();

    std::pair<double, double> t_;
    std::pair<double, double> s_;
    std::pair<double, double> t_actual_;
    std::pair<double, double> s_actual_;
    double width_;
    double length_;
    double vel_;
    double y_intercept_;
};

#endif //FILTER_POSITION_OPTIMIZATION_OBSTACLE_H
