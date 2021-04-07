#include "utils.h"

namespace Utils
{
    void outputObsToFile(const std::string& filename,
                         const Obstacle& obs)
    {
        double t0 = 0.0;

        std::ofstream writing_file;
        writing_file.open(filename, std::ios::out);
        writing_file << "obs_time" <<  "," << "obs_s" << std::endl;

        for(int i=0; i<obs.s_.size(); ++i)
            writing_file << obs.t_[i] << "," << obs.s_[i] << std::endl;

        writing_file.close();
    }

    void outputToFile(const std::string& filename,
                      const std::vector<double>& positions,
                      const std::vector<double>& max_vels,
                      const std::vector<double>& obs_filtered_vels,
                      const std::vector<double>& obs_filtered_times,
                      const std::vector<double>& jerk_filtered_vels,
                      const BaseSolver::OutputInfo& lp_output,
                      const BaseSolver::OutputInfo& qp_output,
                      const BaseSolver::OutputInfo& nc_output)
    {
        assert(positions.size() == max_vels.size());
        assert(positions.size() == obs_filtered_vels.size());
        assert(positions.size() == jerk_filtered_vels.size());

        // Calculate the time when travel with maximum time
        std::vector<double> max_times(positions.size(), 0.0);
        std::vector<double> jerk_filtered_times(positions.size(), 0.0);
        std::vector<double> obs_positions = positions;
        std::vector<double> jerk_positions = positions;
        for(int i=1; i<max_times.size(); ++i)
        {
            double ds = positions[i] - positions[i-1];
            double dt_max  = 0.1;
            double dt_jerk = 0.1;
            if(std::fabs(max_vels[i])>1e-6)
                dt_max = ds/max_vels[i];

            double dt_obs = obs_filtered_times[i] - obs_filtered_times[i-1];
            obs_positions[i] = obs_positions[i-1] + obs_filtered_vels[i-1] * dt_obs;

            if(std::fabs(jerk_filtered_vels[i])>1e-6)
                dt_jerk = ds/jerk_filtered_vels[i];
            else
                jerk_positions[i] = jerk_positions[i-1];

            max_times[i]           = max_times[i-1]           + dt_max;
            jerk_filtered_times[i] = jerk_filtered_times[i-1] + dt_jerk;
        }

        std::ofstream writing_file;
        writing_file.open(filename, std::ios::out);
        writing_file << "position" << "," << "obs_position" << "," << "jerk_position" << ","
                     << "lp_position" << "," << "qp_position" << "," << "nc_position" << ","
                     << "max_time" << "," << "obs_filtered_time" << "," << "jerk_filtered_time" << ","
                     << "max_velocity" << "," << "obs_filtered_velocity" << "," << "jerk_filtered_velocity" << ","
                     << "lp_time" <<  "," << "lp_velocity" << "," << "lp_acceleration" << "," << "lp_jerk" << ","
                     << "qp_time" <<  "," << "qp_velocity" << "," << "qp_acceleration" << "," << "qp_jerk" << ","
                     << "nc_time" <<  "," << "nc_velocity" << "," << "nc_acceleration" << "," << "nc_jerk" << ","
                     << std::endl;

        for(int i=0; i<positions.size(); ++i)
        {
            writing_file << positions[i] << "," << obs_positions[i] << "," << jerk_positions[i] << ","
                         << lp_output.position[i] << "," << qp_output.position[i] << "," << nc_output.position[i] << ","
                         << max_times[i] << "," << obs_filtered_times[i] << "," << jerk_filtered_times[i] << ","
                         << max_vels[i] << "," << obs_filtered_vels[i] << "," << jerk_filtered_vels[i] << ","
                         << lp_output.time[i] << "," << lp_output.velocity[i] << "," << lp_output.acceleration[i] << "," << lp_output.jerk[i] << ","
                         << qp_output.time[i] << "," << qp_output.velocity[i] << "," << qp_output.acceleration[i] << "," << qp_output.jerk[i] << ","
                         << nc_output.time[i] << "," << nc_output.velocity[i] << "," << nc_output.acceleration[i] << "," << nc_output.jerk[i] << ","
                         << std::endl;
        }

        writing_file.close();
    }
}