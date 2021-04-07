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
#include "obstacle.h"
#include "solver/base_solver.h"

namespace Utils
{
    void outputToFile(const std::string& filename,
                      const std::vector<double>& position,
                      const std::vector<double>& max_vels,
                      const std::vector<double>& obs_filtered_vels,
                      const std::vector<double>& obs_filtered_times,
                      const std::vector<double>& jerk_filtered_vels,
                      const BaseSolver::OutputInfo& lp_output,
                      const BaseSolver::OutputInfo& qp_output,
                      const BaseSolver::OutputInfo& nc_output);

    void outputObsToFile(const std::string& filename,
                         const Obstacle& obs);
}

#endif //FILTER_POSITION_OPTIMIZATION_UTILS_H
