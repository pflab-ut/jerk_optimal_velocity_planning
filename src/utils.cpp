#include "utils.h"

namespace Utils
{
    void outputVelocityToFile(const std::string& filename,
                              const std::vector<double>& position,
                              const std::vector<double>& original_velocity,
                              const std::vector<double>& filtered_velocity)
    {
        assert(original_velocity.size() == filtered_velocity.size());
        std::ofstream writing_file;
        writing_file.open(filename, std::ios::out);
        writing_file << "position" << "," << "original_velocity" << "," << "filtered_velocity" << std::endl;
        for(int i=0; i<original_velocity.size(); ++i)
            writing_file << position[i] << "," << original_velocity[i] << "," << filtered_velocity[i] << std::endl;

        writing_file.close();
    }

    void outputVelocityToFile(const std::string& filename,
                              const std::vector<double>& position,
                              const std::vector<double>& original_velocity,
                              const std::vector<double>& filtered_velocity,
                              const std::vector<double>& filtered_acc)
    {
        assert(original_velocity.size() == filtered_velocity.size());
        std::ofstream writing_file;
        writing_file.open(filename, std::ios::out);
        writing_file << "position" << "," << "original_velocity" << "," << "filtered_velocity" << ","  << "filtered_acc" << std::endl;
        for(int i=0; i<original_velocity.size(); ++i)
            writing_file << position[i] << "," << original_velocity[i] << "," << filtered_velocity[i] << "," << filtered_acc[i] << std::endl;

        writing_file.close();
    }

    void outputResultToFile(const std::string& filename,
                            const std::vector<double>& position,
                            const std::vector<double>& qp_velocity,
                            const std::vector<double>& qp_acceleration,
                            const std::vector<double>& qp_jerk)
    {
        std::ofstream writing_file;
        writing_file.open(filename, std::ios::out);
        writing_file << "qp_position" <<  "," << "qp_velocity" << "," << "qp_acceleration" << "," << "qp_jerk" << std::endl;

        for(int i=0; i<qp_velocity.size(); ++i)
            writing_file << position[i] << "," << qp_velocity[i] << "," << qp_acceleration[i] << "," << qp_jerk[i] << std::endl;

        writing_file.close();
    }

    void outputSTToFile(const std::string& filename,
                        const std::vector<double>& position,
                        const std::vector<double>& original_vels,
                        const std::vector<double>& filtered_vels,
                        const Obstacle& obs)
    {
        assert(position.size() == original_vels.size());
        assert(position.size() == filtered_vels.size());
        std::vector<double> original_time(position.size());
        std::vector<double> filtered_time(position.size());
        double t0 = 0.0;
        original_time.front() = t0;
        filtered_time.front() = t0;

        for(int i=0; i<original_vels.size()-1; ++i)
        {
            double ds = position[i+1] - position[i];
            double dt = 0.0;
            if(original_vels[i] > 0.1)
                dt = ds / original_vels[i];
            original_time[i+1] = original_time[i] + dt;
        }

        for(int i=0; i<filtered_vels.size()-1; ++i)
        {
            double ds = position[i + 1] - position[i];
            double dt = 0.0;
            if (filtered_vels[i] > 0.1)
                dt = ds / filtered_vels[i];
            filtered_time[i + 1] = filtered_time[i] + dt;
        }

        std::ofstream writing_file;
        writing_file.open(filename, std::ios::out);
        writing_file << "position" << "," << "original_t" << "," << "filtered_t" << "," << "obs_time" <<  "," << "obs_s" << std::endl;

        for(int i=0; i<position.size(); ++i)
        {
            if(i<obs.s_.size())
                writing_file << position[i] << "," << original_time[i] << "," << filtered_time[i] << ","
                             << obs.t_[i] << "," << obs.s_[i] << std::endl;
            else
                writing_file << position[i] << "," << original_time[i] << "," << filtered_time[i] << std::endl;
        }
        writing_file.close();
    }
}