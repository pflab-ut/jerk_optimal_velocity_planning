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
    const int obs_size = 50;
    const double obs_v = 2.0;
    const double dt = 0.1;
    const double s0 = 10.0;
    const double t0 = 2.0;
    scenario_data.obs_ = Obstacle(obs_size, obs_v, dt, s0, t0);
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
    const int obs_size = 50;
    const double obs_v = 2.0;
    const double dt = 0.1;
    const double s0 = 10.0;
    const double t0 = 2.0;
    scenario_data.obs_ = Obstacle(obs_size, obs_v, dt, s0, t0);
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
    const int obs_size = 100;
    const double obs_v = 0.0;
    const double dt = 0.1;
    const double s0 = 15.0;
    const double t0 = 3.0;
    scenario_data.obs_ = Obstacle(obs_size, obs_v, dt, s0, t0);
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
    const int obs_size = 100;
    const double obs_v = 1.0;
    const double dt = 0.1;
    const double s0 = 15.0;
    const double t0 = 7.0;
    scenario_data.obs_ = Obstacle(obs_size, obs_v, dt, s0, t0);
}
