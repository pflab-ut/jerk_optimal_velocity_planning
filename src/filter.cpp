#include "filter.h"

void Filter::smoothVelocity(const double& ds,
                            const double& initial_vel,
                            const double& initial_acc,
                            const double& max_acc,
                            const double& jerk_acc,
                            const std::vector<double>& original_vel,
                            std::vector<double>& filtered_vel,
                            std::vector<double>& filtered_acc)
{
    filtered_vel = std::vector<double>(original_vel.size());
    filtered_acc = std::vector<double>(original_vel.size());
    filtered_vel.front() = initial_vel;
    filtered_acc.front() = initial_acc;
    double current_vel = initial_vel;
    double current_acc = initial_acc;

    // Forward Filter
    for(unsigned int i=1; i<original_vel.size(); ++i)
    {
        double dt = 0.0;
        if(std::fabs(current_vel)<1e-6)
            dt = sqrt(2*ds/max_acc);
        else
            dt = ds/current_vel;

        current_acc = std::min(current_acc + jerk_acc*dt, max_acc);
        double next_vel = current_vel + current_acc * dt;
        if(next_vel > original_vel[i])
        {
            current_vel = original_vel[i];
            current_acc = 0.0;
        }
        else
            current_vel = next_vel;

        // Store Filtered Velocity
        filtered_vel[i] = current_vel;
        filtered_acc[i] = current_acc;
    }

    std::vector<double> forward_vels = filtered_vel;

    //3. Backward Filter
    filtered_vel.back() = original_vel.back();
    filtered_acc.back() = 0.0;
    current_vel = original_vel.back();
    current_acc = 0.0;
    for(int i=static_cast<int>(original_vel.size())-2; i>=0; --i)
    {
        double dt;
        if(std::fabs(current_vel)<1e-4)
            dt = sqrt(2*ds/max_acc);
        else
            dt = ds/current_vel;

        current_acc = std::min(current_acc + jerk_acc*dt, max_acc);
        double next_vel = current_vel + current_acc * dt;
        if(next_vel > filtered_vel[i])
        {
            current_vel = filtered_vel[i];
            current_acc = 0.0;
        }
        else
        {
            current_vel = next_vel;
            filtered_acc[i] = -current_acc;
        }

        // Store Filtered Velocity
        filtered_vel[i] = current_vel;
    }

    std::vector<double> backward_vels = filtered_vel;
    std::vector<double> merged_vels;
    mergeFilteredVelocity(forward_vels, backward_vels, merged_vels);
    filtered_vel = merged_vels;
}

void Filter::mergeFilteredVelocity(const std::vector<double> &forward_vels,
                                   const std::vector<double> &backeard_vels,
                                   std::vector<double> &merged_vels)
{
    double ep = 1e-5;
    double v0 = forward_vels.front();

    merged_vels.resize(forward_vels.size());

    size_t i = 0;
    if(backeard_vels.front() < v0 - 1e-6)
    {
        while(backeard_vels[i] < forward_vels[i] && i < merged_vels.size())
        {
            merged_vels[i] = forward_vels[i];
            ++i;
        }
    }

    for(; i<merged_vels.size(); ++i)
        merged_vels[i] = (forward_vels[i] < backeard_vels[i]) ? forward_vels[i] : backeard_vels[i];
}

