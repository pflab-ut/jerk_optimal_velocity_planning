#ifndef FILTER_POSITION_OPTIMIZATION_UTILS_H
#define FILTER_POSITION_OPTIMIZATION_UTILS_H

#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <vector>
#include <chrono>
#include <string>
#include <fstream>
#include <cassert>

namespace Utils
{
    void outputVelocityToFile(const std::string& filename,
                              const std::vector<double>& position,
                              const std::vector<double>& original_velocity,
                              const std::vector<double>& filtered_velocity,
                              const std::vector<double>& filtered_acc);

    void outputResultToFile(const std::string& filename,
                            const std::vector<double>& position,
                            const std::vector<double>& qp_velocity,
                            const std::vector<double>& qp_acceleration,
                            const std::vector<double>& qp_jerk);
}

#endif //FILTER_POSITION_OPTIMIZATION_UTILS_H
