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
}