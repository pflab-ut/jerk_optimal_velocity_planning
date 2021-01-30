#ifndef FILTER_POSITION_OPTIMIZATION_INTERPOLATE_H
#define FILTER_POSITION_OPTIMIZATION_INTERPOLATE_H

#include <cmath>
#include <iostream>
#include <vector>

class LinearInterpolate
{
public:
    LinearInterpolate(){};
    ~LinearInterpolate(){};
    static bool interpolate(
            const std::vector<double> & base_index, const std::vector<double> & base_value,
            const std::vector<double> & return_index, std::vector<double> & return_value);
};

class SplineInterpolate
{
    bool initialized_;
    std::vector<double> a_;  //!< @brief temporal vector for calculation
    std::vector<double> b_;  //!< @brief temporal vector for calculation
    std::vector<double> c_;  //!< @brief temporal vector for calculation
    std::vector<double> d_;  //!< @brief temporal vector for calculation

public:
    SplineInterpolate();
    SplineInterpolate(const std::vector<double> & x);
    ~SplineInterpolate();
    void generateSpline(const std::vector<double> & x);
    double getValue(const double & s);
    bool interpolate(
            const std::vector<double> & base_index, const std::vector<double> & base_value,
            const std::vector<double> & return_index, std::vector<double> & return_value);
    void getValueVector(const std::vector<double> & s_v, std::vector<double> & value_v);
};

#endif //FILTER_POSITION_OPTIMIZATION_INTERPOLATE_H
