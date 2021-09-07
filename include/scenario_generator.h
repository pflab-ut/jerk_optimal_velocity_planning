#ifndef FILTER_POSITION_OPTIMIZATION_SCENARIO_GENERATOR_H
#define FILTER_POSITION_OPTIMIZATION_SCENARIO_GENERATOR_H

#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <vector>
#include "obstacle.h"

class ScenarioGenerator
{
public:
    enum ScenarioNumber
    {
        Normal = 0,
        Accelerate = 1,
        Stop = 2,
        Wait = 3,
        ZeroMaximum = 4,
    };

    struct ScenarioData
    {
        // 1. Velocity
        int N_;
        double v0_;
        double a0_;
        double ds_;
        double max_acc_;
        double max_jerk_;
        double min_acc_;
        double min_jerk_;

        std::vector<double> positions_;
        std::vector<double> max_velocities_;

        // 2. Obstacles
        Obstacle obs_;
    };


    ScenarioGenerator() = default;
    ~ScenarioGenerator() = default;

    void generate(const ScenarioNumber& scenario_num, ScenarioData& scenario_data);
    void generateNormalScenario(ScenarioData& scenario_data);
    void generateAccelerateScenario(ScenarioData& scenario_data);
    void generateStopScenario(ScenarioData& scenario_data);
    void generateWaitScenario(ScenarioData& scenario_data);
    void generateZeroMaximumScenario(ScenarioData& scenario_data);
};

#endif //FILTER_POSITION_OPTIMIZATION_SCENARIO_GENERATOR_H
