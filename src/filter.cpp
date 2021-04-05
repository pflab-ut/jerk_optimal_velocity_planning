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

bool Filter::obstacleVelocityLimitFilter(const double& initial_vel,
                                         const std::vector<double>& input_arclength,
                                         const std::vector<double>& max_vels,
                                         const Obstacle& obstacle,
                                         std::vector<double>& filtered_vels)
{
    filtered_vels = max_vels;
    // 1. Compute Intersection Time
    std::vector<double> intersection_time;
    std::vector<double> intersection_arclength;
    for(int i=0; i<obstacle.extend_s_.size(); ++i)
    {
        double s_obs = obstacle.extend_s_[i];
        double min_dist = std::numeric_limits<double>::max();
        double min_id   = -1;
        for(int j=0; j<input_arclength.size(); ++j)
        {
            double s_ego = input_arclength[j];
            double dist  = std::fabs(s_ego - s_obs);
            if(dist < min_dist)
            {
                min_dist = dist;
                min_id = j;
            }
        }

        if(min_id > 0 && min_dist<0.2)
        {
            intersection_time.push_back(obstacle.extend_t_[i]);
            intersection_arclength.push_back(input_arclength[min_id]);
        }
    }

    // 2. Find arclength_inner and interpolate intersection time
    std::vector<double> arclength_inner;
    std::vector<double> time_inner;
    size_t idx_cutout = 0;
    if(std::fabs(intersection_arclength.front()-intersection_arclength.back())<1e-6)
    {
        // When the obstacle stops
        arclength_inner = intersection_arclength;
        time_inner = intersection_time;
        auto it_cutout_arclength = std::find_if(input_arclength.begin(), input_arclength.end(), [&intersection_arclength](double x){ return x > intersection_arclength.back(); });
        idx_cutout = std::distance(input_arclength.begin(), it_cutout_arclength);
    }
    else
    {
        auto it_cutin_arclength = std::find_if(input_arclength.begin(), input_arclength.end(), [&intersection_arclength](double x){ return x >= intersection_arclength[0]; });
        auto it_cutout_arclength = std::find_if(input_arclength.begin(), input_arclength.end(), [&intersection_arclength](double x){ return x > intersection_arclength.back(); });
        for(auto it = it_cutin_arclength; it!=it_cutout_arclength; ++it)
            arclength_inner.push_back(*it);

        if(!LinearInterpolate::interpolate(intersection_arclength, intersection_time, arclength_inner, time_inner))
        {
            std::cout << "Interpolation Failed" << std::endl;
            return false;
        }

        idx_cutout = std::distance(input_arclength.begin(), it_cutout_arclength);
    }


    //4. Set Velocity Limits
    double t = input_arclength[1]/std::max(max_vels[0], 0.1);
    double range_s1 = 3.0; //temporary
    double range_s2 = 1.0; //temporary
    double range_t = 0.5; //temporary
    size_t interrputed_idx = input_arclength.size()-1;
    for(size_t i=1; i < input_arclength.size()-1; ++i)
    {
        // Manage Maximum Velocity here
        double v = 0.0;
        if(std::fabs(max_vels[i]<1e-3) || i > idx_cutout)
            v = max_vels[i];
        else
        {
            double ds = input_arclength[i+1] - input_arclength[i];
            double t_tmp = t + ds / max_vels[i];
            double nearest_s = range_s1;
            double nearest_t = range_t;
            size_t j_nearest = 0;

            // Find nearest Intersection point
            for(size_t j = 0; j < arclength_inner.size(); ++j)
            {
                double delta_s = std::fabs(arclength_inner[j] - input_arclength[i]);
                double delta_t = std::fabs(time_inner[j] - t_tmp);
                if(delta_s < nearest_s && delta_t < nearest_t)
                {
                    nearest_s = delta_s;
                    nearest_t = delta_t;
                    j_nearest = j;
                }
            }

            if(nearest_s < range_s1 && nearest_t < range_t)
            {
                if(nearest_s < range_s2)
                    v = (arclength_inner[j_nearest + 1] - arclength_inner[j_nearest]) / (time_inner[j_nearest + 1] - time_inner[j_nearest]);
                else
                    v = (arclength_inner.back() - input_arclength[i]) / (time_inner.back() - t);

                // If the velocity is less than 1e-6, then we move the time till the end of the obstacle's cutout time
                if(std::fabs(v)<1e-6)
                {
                    t = intersection_time.back() + range_t;
                    /*
                    interrputed_idx = i;
                    break;
                    */
                }
                else
                {
                    t = t + ds / v;
                }
            }
            else
            {
                v = max_vels[i];
                t = t_tmp;
            }
        }

        // Fill the calculated Value
        filtered_vels[i] = v;
    }
    filtered_vels.back() = 0.0;

    return true;
}
