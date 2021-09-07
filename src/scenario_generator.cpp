#include "scenario_generator.h"

void ScenarioGenerator::generate(const ScenarioNumber& scenario_num,
                                 ScenarioData& scenario_data)
{
    if(scenario_num==Accelerate)
        return generateAccelerateScenario(scenario_data);
    else if(scenario_num==Stop)
        return generateStopScenario(scenario_data);
    else if(scenario_num==Wait)
        return generateWaitScenario(scenario_data);
    else if(scenario_num == ZeroMaximum)
        return generateZeroMaximumScenario(scenario_data);
    else
        return generateNormalScenario(scenario_data);
}

void ScenarioGenerator::generateNormalScenario(ScenarioData &scenario_data)
{
    /***************************************************/
    /*************** Velocity Profile ******************/
    /***************************************************/
    scenario_data.N_        = 300;
    scenario_data.v0_       = 0.5;
    scenario_data.a0_       = 0.0;
    scenario_data.ds_       = 0.1;
    scenario_data.max_acc_  = 1.0;
    scenario_data.max_jerk_ = 0.8;
    scenario_data.min_acc_  = -1.0;
    scenario_data.min_jerk_ = -0.8;

    scenario_data.positions_      = std::vector<double>(scenario_data.N_, 0.0);
    scenario_data.max_velocities_ = std::vector<double>(scenario_data.N_, 0.0);

    // Position
    for(int i=0; i<scenario_data.N_; ++i)
        scenario_data.positions_[i] = i*scenario_data.ds_;

    // Maximum Velocities
    for(int i=0; i<100; ++i)
        scenario_data.max_velocities_[i] = 3.0;
    for(int i=100; i<200; ++i)
        scenario_data.max_velocities_[i] = 5.0;
    for(int i=200; i<scenario_data.N_; ++i)
        scenario_data.max_velocities_[i] = 4.0;
    scenario_data.max_velocities_.back() = 0.0;

    /***************************************************/
    /******************* Obstacle **********************/
    /***************************************************/
    const double obs_v = 2.0;
    const double t0 = 2.0;
    const double tN = 7.0;
    const double s0 = 10.0;
    const double sN = s0 + obs_v * (tN- t0);
    const double obs_width  = 2.0;
    const double obs_length = 4.0;
    const double margin_s = 2.0;
    scenario_data.obs_ = Obstacle(t0, tN, s0, sN, obs_width, obs_length, margin_s);
}

void ScenarioGenerator::generateAccelerateScenario(ScenarioData& scenario_data)
{
    /***************************************************/
    /*************** Velocity Profile ******************/
    /***************************************************/
    scenario_data.N_        = 300;
    scenario_data.v0_       = 0.5;
    scenario_data.a0_       = 0.0;
    scenario_data.ds_       = 0.1;
    scenario_data.max_acc_  = 1.0;
    scenario_data.max_jerk_ = 0.8;
    scenario_data.min_acc_  = -1.0;
    scenario_data.min_jerk_ = -0.8;

    scenario_data.positions_      = std::vector<double>(scenario_data.N_, 0.0);
    scenario_data.max_velocities_ = std::vector<double>(scenario_data.N_, 0.0);

    // Position
    for(int i=0; i<scenario_data.N_; ++i)
        scenario_data.positions_[i] = i*scenario_data.ds_;

    // Maximum Velocities
    for(int i=0; i<100; ++i)
        scenario_data.max_velocities_[i] = 3.0;
    for(int i=100; i<200; ++i)
        scenario_data.max_velocities_[i] = 5.0;
    for(int i=200; i<scenario_data.N_; ++i)
        scenario_data.max_velocities_[i] = 7.0;
    scenario_data.max_velocities_.back() = 0.0;

    /***************************************************/
    /******************* Obstacle **********************/
    /***************************************************/
    const double obs_v = 2.0;
    const double t0 = 2.0;
    const double tN = 7.0;
    const double s0 = 10.0;
    const double sN = s0 + obs_v * (tN- t0);
    const double obs_width  = 2.0;
    const double obs_length = 4.0;
    const double margin_s = 2.0;
    scenario_data.obs_ = Obstacle(t0, tN, s0, sN, obs_width, obs_length, margin_s);
}

void ScenarioGenerator::generateStopScenario(ScenarioData& scenario_data)
{
    /***************************************************/
    /*************** Velocity Profile ******************/
    /***************************************************/
    scenario_data.N_        = 300;
    scenario_data.v0_       = 0.5;
    scenario_data.a0_       = 0.0;
    scenario_data.ds_       = 0.1;
    scenario_data.max_acc_  = 1.0;
    scenario_data.max_jerk_ = 0.8;
    scenario_data.min_acc_  = -1.0;
    scenario_data.min_jerk_ = -0.8;

    scenario_data.positions_      = std::vector<double>(scenario_data.N_, 0.0);
    scenario_data.max_velocities_ = std::vector<double>(scenario_data.N_, 0.0);

    // Position
    for(int i=0; i<scenario_data.N_; ++i)
        scenario_data.positions_[i] = i*scenario_data.ds_;

    // Maximum Velocities
    for(int i=0; i<100; ++i)
        scenario_data.max_velocities_[i] = 3.0;
    for(int i=100; i<200; ++i)
        scenario_data.max_velocities_[i] = 5.0;
    for(int i=200; i<scenario_data.N_; ++i)
        scenario_data.max_velocities_[i] = 7.0;
    scenario_data.max_velocities_.back() = 0.0;

    /***************************************************/
    /******************* Obstacle **********************/
    /***************************************************/
    const double obs_v = 0.0;
    const double t0 = 3.0;
    const double tN = 13.0;
    const double s0 = 17.0;
    const double sN = s0 + obs_v * (tN- t0);
    const double obs_width  = 2.0;
    const double obs_length = 4.0;
    const double margin_s = 2.0;
    scenario_data.obs_ = Obstacle(t0, tN, s0, sN, obs_width, obs_length, margin_s);
}

void ScenarioGenerator::generateWaitScenario(ScenarioData& scenario_data)
{
    /***************************************************/
    /*************** Velocity Profile ******************/
    /***************************************************/
    scenario_data.N_        = 300;
    scenario_data.v0_       = 0.5;
    scenario_data.a0_       = 0.0;
    scenario_data.ds_       = 0.1;
    scenario_data.max_acc_  = 1.0;
    scenario_data.max_jerk_ = 0.8;
    scenario_data.min_acc_  = -1.0;
    scenario_data.min_jerk_ = -0.8;

    scenario_data.positions_      = std::vector<double>(scenario_data.N_, 0.0);
    scenario_data.max_velocities_ = std::vector<double>(scenario_data.N_, 0.0);

    // Position
    for(int i=0; i<scenario_data.N_; ++i)
        scenario_data.positions_[i] = i*scenario_data.ds_;

    // Maximum Velocities
    for(int i=0; i<100; ++i)
        scenario_data.max_velocities_[i] = 3.0;
    for(int i=100; i<200; ++i)
        scenario_data.max_velocities_[i] = 5.0;
    for(int i=200; i<scenario_data.N_; ++i)
        scenario_data.max_velocities_[i] = 7.0;
    scenario_data.max_velocities_.back() = 0.0;

    /***************************************************/
    /******************* Obstacle **********************/
    /***************************************************/
    const double obs_v = 1.0;
    const double t0 = 7.0;
    const double tN = 17.0;
    const double s0 = 15.0;
    const double sN = s0 + obs_v * (tN- t0);
    const double obs_width  = 2.0;
    const double obs_length = 4.0;
    const double margin_s = 2.0;
    scenario_data.obs_ = Obstacle(t0, tN, s0, sN, obs_width, obs_length, margin_s);
}

void ScenarioGenerator::generateZeroMaximumScenario(ScenarioData &scenario_data)
{
    /***************************************************/
    /*************** Velocity Profile ******************/
    /***************************************************/
    scenario_data.N_        = 300;
    scenario_data.v0_       = 0.5;
    scenario_data.a0_       = 0.0;
    scenario_data.ds_       = 0.1;
    scenario_data.max_acc_  = 1.0;
    scenario_data.max_jerk_ = 0.8;
    scenario_data.min_acc_  = -1.0;
    scenario_data.min_jerk_ = -0.8;

    scenario_data.positions_      = std::vector<double>(scenario_data.N_, 0.0);
    scenario_data.max_velocities_ = std::vector<double>(scenario_data.N_, 0.0);

    // Position
    for(int i=0; i<scenario_data.N_; ++i)
        scenario_data.positions_[i] = i*scenario_data.ds_;

    // Maximum Velocities
    for(int i=0; i<100; ++i)
        scenario_data.max_velocities_[i] = 3.0;
    for(int i=100; i<150; ++i)
        scenario_data.max_velocities_[i] = 5.0;
    scenario_data.max_velocities_[150] = 0.0;
    for(int i=151; i<200; ++i)
        scenario_data.max_velocities_[i] = 5.0;
    for(int i=200; i<scenario_data.N_; ++i)
        scenario_data.max_velocities_[i] = 4.0;
    scenario_data.max_velocities_.back() = 0.0;

    /***************************************************/
    /******************* Obstacle **********************/
    /***************************************************/
    const double obs_v = 0.0;
    const double t0 = 2.0;
    const double tN = 7.0;
    const double s0 = 20.0;
    const double sN = s0 + obs_v * (tN- t0);
    const double obs_width  = 2.0;
    const double obs_length = 4.0;
    const double margin_s = 2.0;
    scenario_data.obs_ = Obstacle(t0, tN, s0, sN, obs_width, obs_length, margin_s);
}
