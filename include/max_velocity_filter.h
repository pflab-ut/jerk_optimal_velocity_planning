#ifndef FILTER_POSITION_OPTIMIZATION_MAX_VELOCITY_FILTER_H
#define FILTER_POSITION_OPTIMIZATION_MAX_VELOCITY_FILTER_H

#include <boost/assign/list_of.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometry.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include "obstacle.h"

enum class Category
{
    Ignore= -1,
    ZeroVelObstacle = 0,
    PosInterceptionObstacle = 1,
    NegLeftInterceptionObstacle = 2,
    NegRightInterceptionObstacle = 3,
};

namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> point_type;
typedef std::pair<double, double> TSPoint;
typedef std::pair<TSPoint, TSPoint> Line;

class MaximumVelocityFilter
{
public:
    struct OutputInfo
    {
        std::vector<double> time;
        std::vector<double> position;
        std::vector<double> velocity;

        void reserve(const unsigned int& N)
        {
            time.reserve(N);
            velocity.reserve(N);
            position.reserve(N);
        }

        void resize(const unsigned int& N)
        {
            time.resize(N);
            velocity.resize(N);
            position.resize(N);
        }
    };

    MaximumVelocityFilter(const double low_velocity_threshold);
    ~MaximumVelocityFilter() = default;

    void modifyMaximumVelocity(const std::vector<double>& positions,
                               const std::vector<double>& original_max_vels,
                               MaximumVelocityFilter::OutputInfo& output_data);

    void smoothVelocity(const double& ds, const double& v0, const double& a0,
                        const double& a_max, const double& j_max,
                        const double& a_min, const double& j_min,
                        const std::vector<double>& original_vel,
                        std::vector<double>& filtered_vel,
                        std::vector<double>& filtered_acc);

    void forwardJerkFilter(const double v0, const double a0, const double a_max, const double j_max,
                           const double ds,
                           const std::vector<double>& original_vel,
                           std::vector<double>& filtered_vel,
                           std::vector<double>& filtered_acc);

    void backwardJerkFilter(const double v0, const double a0, const double a_min, const double j_min,
                            const double ds,
                            const std::vector<double>& original_vel,
                            std::vector<double>& filtered_vel,
                            std::vector<double>& filtered_acc);

    Category categorizeObstacles(
            const std::vector<double> & times, const std::vector<double> & positions,
            const Obstacle & obstacle);

    void calcSafetyVelocity(
            const std::vector<double> & positions, const std::vector<double> & max_vels,
            const Obstacle & obstacle, const double v0, const double a0, const double a_decel,
            const double j_decel, const double margin_s2, std::vector<double> & updated_max_positions,
            std::vector<double> & updated_max_vels);

    void calcZeroVelObsVelocity(
            const std::vector<double> & times, const std::vector<double> & positions,
            const std::vector<double> & max_vels, const Obstacle & obstacle, const double & margin_s1,
            const double & margin_s2, std::vector<double> & updated_max_times,
            std::vector<double> & updated_max_positions, std::vector<double> & updated_max_vels);

    void calcPosInterceptObsVelocity(
            const std::vector<double> & times, const std::vector<double> & positions,
            const std::vector<double> & max_vels, const Obstacle & obstacle, const double & margin_s1,
            const double & margin_s2, std::vector<double> & opt_times, std::vector<double> & opt_positions,
            std::vector<double> & opt_vels);

    void calcNegRightInterceptObsVelocity(
            const std::vector<double> & times, const std::vector<double> & positions,
            const std::vector<double> & max_vels, const Obstacle & obstacle, const double & margin_s1,
            const double & margin_s2, std::vector<double> & opt_times, std::vector<double> & opt_positions,
            std::vector<double> & opt_vels);

    void calcNegLeftInterceptObsVelocity(
            const std::vector<double> & times, const std::vector<double> & positions,
            const std::vector<double> & max_vels, const Obstacle & obstacle, const double & margin_s1,
            const double & margin_s2, std::vector<double> & opt_times, std::vector<double> & opt_positions,
            std::vector<double> & opt_vels);

private:
    void calcFirstStage(
            const std::vector<double> & positions, const std::vector<double> & max_vels,
            const Line & obstacle_line, const double & margin_s1, const double & obs_vel, double & t_to_t1,
            double & s_to_s1, int & s1_position_id, std::vector<double> & opt_times,
            std::vector<double> & opt_positions, std::vector<double> & opt_vels);

    void calcSecondStage(
            const std::vector<double> & positions, const std::vector<double> & max_vels,
            const Line & obstacle_line, const double & margin_s2, const double & obstacle_vel,
            const double & obstacle_offset, const double & v_candidate, const double & t_to_t1,
            const double & s_to_s1, const int & s1_position_id, double & t1_to_t2, double & s1_to_s2,
            int & s2_position_id, std::vector<double> & opt_times, std::vector<double> & opt_positions,
            std::vector<double> & opt_vels);

    void calcFinalStage(
            const std::vector<double> & positions, const std::vector<double> & max_vels,
            const Line & obstacle_line, const double & obstacle_vel, const double & t1_to_t2,
            const double & s1_to_s2, const int & s2_position_id, std::vector<double> & opt_times,
            std::vector<double> & opt_positions, std::vector<double> & opt_vels);

    bool checkIntersect(
            const std::pair<double, double> & line1_ini, const std::pair<double, double> & line1_end,
            const std::pair<double, double> & line2_ini, const std::pair<double, double> & line2_end);

    bool calcIntersectionPoint(
            const std::pair<double, double> & a, const std::pair<double, double> & b,
            const std::pair<double, double> & c, const std::pair<double, double> & d,
            std::pair<double, double> & result);

    double calcCross(const std::pair<double, double> & a, const std::pair<double, double> & b);

    bool checkSide(
            const std::pair<double, double> & point, const std::pair<double, double> & line_front,
            const std::pair<double, double> & line_back);

    bool checkIsLeft(
            const std::vector<double> & times, const std::vector<double> & positions,
            const Obstacle & obstacle);
    bool checkIsLeft(
            const std::vector<double> & times, const std::vector<double> & positions,
            const Line & obs_line);

    void mergeFilteredVelocity(const std::vector<double>& forward_vels,
                               const std::vector<double>& backeard_vels,
                               std::vector<double>& merged_vels);

    const double low_velocity_threshold_;
};

#endif //FILTER_POSITION_OPTIMIZATION_MAX_VELOCITY_FILTER_H
