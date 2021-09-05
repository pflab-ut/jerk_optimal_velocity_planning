#include "max_velocity_filter.h"

MaximumVelocityFilter::MaximumVelocityFilter(const double low_velocity_threshold)
        : low_velocity_threshold_(low_velocity_threshold)
{
}

void MaximumVelocityFilter::modifyMaximumVelocity(const std::vector<double>& positions,
                                                  const std::vector<double>& original_max_vels,
                                                  MaximumVelocityFilter::OutputInfo& output_data)
{
    output_data.resize(original_max_vels.size());
    output_data.velocity.front() = original_max_vels.front();
    output_data.position.front() = 0.0;
    output_data.time.front()     = 0.0;
    unsigned int interrupt_id = original_max_vels.size() - 1;

    double t = 0.0;
    for(unsigned int i=0; i<original_max_vels.size()-1; ++i)
    {
        if(std::fabs(original_max_vels[i]) < 1e-6)
        {
            interrupt_id = i;
            break;
        }
        else
        {
            double dt = (positions[i+1] - positions[i]) / original_max_vels[i];
            t += dt;

            output_data.position[i] = positions[i];
            output_data.velocity[i] = original_max_vels[i];
            output_data.time[i+1] = t;
        }
    }

    for(unsigned int i=interrupt_id; i<original_max_vels.size(); ++i)
    {
        output_data.position[i] = positions[interrupt_id];
        output_data.velocity[i] = 0.0;

        if(i<original_max_vels.size()-1)
            output_data.time[i+1] = output_data.time[i] + 0.1;
        else
            output_data.time[i+1] = output_data.time[i];
    }

    return;
}

void MaximumVelocityFilter::smoothVelocity(const double& ds,
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

Category MaximumVelocityFilter::categorizeObstacles(
        const std::vector<double> & times, const std::vector<double> & positions,
        const Obstacle & obstacle)
{
    std::pair<double, double> obs_ini = std::make_pair(obstacle.t_.first, obstacle.s_.first);
    std::pair<double, double> obs_end = std::make_pair(obstacle.t_.second, obstacle.s_.second);

    // Step0. Check intersection and side for the obstacle (Ignore the obstacle that lies is in the left side of the max velocity)
    for (size_t pos_id = 0; pos_id < positions.size() - 1; ++pos_id) {
        TSPoint pos_ini = std::make_pair(times[pos_id], positions[pos_id]);
        TSPoint pos_end = std::make_pair(times[pos_id + 1], positions[pos_id + 1]);
        TSPoint collision_point;
        bool is_intersect = calcIntersectionPoint(pos_ini, pos_end, obs_ini, obs_end, collision_point);
        if (is_intersect) {
            if (obstacle.vel_ < low_velocity_threshold_)
                return Category::ZeroVelObstacle;
            else if (obstacle.y_intercept_ > 0.0)
                return Category::PosInterceptionObstacle;
            else {
                constexpr double epsilon = 1e-3;
                double collision_t = std::max(collision_point.first - epsilon, obstacle.t_.first);
                double collision_s = obstacle.s_.first + obstacle.vel_ * (collision_t - obstacle.t_.first);
                Line obs_line = std::make_pair(
                        std::make_pair(obstacle.t_.first, obstacle.s_.first),
                        std::make_pair(collision_t, collision_s));
                if (checkIsLeft(times, positions, obs_line))
                    return Category::NegLeftInterceptionObstacle;
                else
                    return Category::NegRightInterceptionObstacle;
            }
        }
    }

    if (checkIsLeft(times, positions, obstacle)) {
        // Ignore obstacle that is in the left side of the maximum velocity profile
        return Category::Ignore;
    }

    // The remainder of the obstacle is in the right side of the maximum velocity profile, which does not have intersects
    // Step1. Check if the obstacle's y intercept is larger than zero or not
    if (obstacle.vel_ < low_velocity_threshold_)
        return Category::ZeroVelObstacle;
    else if (obstacle.y_intercept_ > 0.0)
        return Category::PosInterceptionObstacle;
    else
        return Category::NegRightInterceptionObstacle;
}

bool MaximumVelocityFilter::checkIntersect(
        const std::pair<double, double> & line1_ini, const std::pair<double, double> & line1_end,
        const std::pair<double, double> & line2_ini, const std::pair<double, double> & line2_end)
{
    double ax = line1_ini.first;
    double ay = line1_ini.second;
    double bx = line1_end.first;
    double by = line1_end.second;
    double cx = line2_ini.first;
    double cy = line2_ini.second;
    double dx = line2_end.first;
    double dy = line2_end.second;

    double s = (ax - bx) * (cy - ay) - (ay - by) * (cx - ax);
    double t = (ax - bx) * (dy - ay) - (ay - by) * (dx - ax);
    if (s * t <= 0) {
        s = (cx - dx) * (ay - cy) - (cy - dy) * (ax - cx);
        t = (cx - dx) * (by - cy) - (cy - dy) * (bx - cx);
        if (s * t <= 0) return true;
    }

    return false;
}

double MaximumVelocityFilter::calcCross(
        const std::pair<double, double> & a, const std::pair<double, double> & b)
{
    return a.first * b.second - a.second * b.first;
}

bool MaximumVelocityFilter::calcIntersectionPoint(
        const std::pair<double, double> & a, const std::pair<double, double> & b,
        const std::pair<double, double> & c, const std::pair<double, double> & d,
        std::pair<double, double> & result)
{
    std::pair<double, double> tmp1 = std::make_pair(b.first - a.first, b.second - a.second);
    std::pair<double, double> tmp2 = std::make_pair(d.first - c.first, d.second - c.second);
    double deno = calcCross(tmp1, tmp2);
    if (std::abs(deno) < 1e-6) return false;

    std::pair<double, double> ca = std::make_pair(c.first - a.first, c.second - a.second);
    std::pair<double, double> dc = std::make_pair(d.first - c.first, d.second - c.second);
    std::pair<double, double> ba = std::make_pair(b.first - a.first, b.second - a.second);
    std::pair<double, double> ac = std::make_pair(a.first - c.first, a.second - c.second);
    double s = calcCross(ca, dc) / deno;
    double t = calcCross(ba, ac) / deno;
    if (s < 0.0 || 1.0 < s || t < 0.0 || 1.0 < t) return false;

    result.first = a.first + s * (b.first - a.first);
    result.second = a.second + s * (b.second - a.second);

    return true;
}

bool MaximumVelocityFilter::checkSide(
        const std::pair<double, double> & point, const std::pair<double, double> & line_first,
        const std::pair<double, double> & line_back)
{
    double value = point.first * (line_first.second - line_back.second) +
                   line_first.first * (line_back.second - point.second) +
                   line_back.first * (point.second - line_first.second);

    // If point is in the right side, value should be less than 0.0;
    return value < 0.0;
}

bool MaximumVelocityFilter::checkIsLeft(
        const std::vector<double> & times, const std::vector<double> & positions,
        const Obstacle & obstacle)
{
    bg::model::polygon<point_type> polygon;
    for (size_t i = 0; i < positions.size(); ++i)
        polygon.outer().push_back(point_type(times[i], positions[i]));
    for (int i = positions.size() - 2; i >= 0; --i)
        polygon.outer().push_back(point_type(times[i], positions.back()));
    polygon.outer().push_back(point_type(times[0], positions[0]));

    bg::model::linestring<point_type> line = boost::assign::list_of<point_type>(
            obstacle.t_.first, obstacle.s_.first)(obstacle.t_.second, obstacle.s_.second);

    return bg::within(
            line, polygon);  // If obstacle is in the left side, this function will return true
}

bool MaximumVelocityFilter::checkIsLeft(
        const std::vector<double> & times, const std::vector<double> & positions, const Line & obs_line)
{
    bg::model::polygon<point_type> polygon;
    for (size_t i = 0; i < positions.size(); ++i)
        polygon.outer().push_back(point_type(times[i], positions[i]));
    for (int i = positions.size() - 2; i >= 0; --i)
        polygon.outer().push_back(point_type(times[i], positions.back()));
    polygon.outer().push_back(point_type(times[0], positions[0]));

    bg::model::linestring<point_type> line = boost::assign::list_of<point_type>(
            obs_line.first.first, obs_line.first.second)(obs_line.second.first, obs_line.second.second);

    return bg::within(
            line, polygon);  // If obstacle is in the left side, this function will return true
}

void MaximumVelocityFilter::calcFirstStage(
        const std::vector<double> & positions, const std::vector<double> & max_vels,
        const Line & obstacle_line, const double & margin_s1, const double & obs_vel, double & t_to_t1,
        double & s_to_s1, int & s1_position_id, std::vector<double> & opt_times,
        std::vector<double> & opt_positions, std::vector<double> & opt_vels)
{
    s1_position_id = -1;
    t_to_t1 = 0.0;
    s_to_s1 = 0.0;
    opt_times.push_back(t_to_t1);
    opt_positions.push_back(s_to_s1);

    for (size_t i = 0; i < max_vels.size() - 1; ++i) {
        double v = max_vels[i];
        double ds = positions[i + 1] - positions[i];
        double tmp_t = t_to_t1 + ds / v;
        double tmp_s = s_to_s1 + ds;

        // Check which index cross the first collision point
        double obs_s_at_t =
                obstacle_line.first.second + obs_vel * std::max(tmp_t - obstacle_line.first.first, 0.0);
        //if(obstacle_line.first.second - tmp_s < margin_s1)
        if (obs_s_at_t - tmp_s < margin_s1) {
            s1_position_id = i;
            break;
        } else {
            opt_times.push_back(tmp_t);
            opt_positions.push_back(tmp_s);
            opt_vels.push_back(v);
            t_to_t1 = tmp_t;
            s_to_s1 = tmp_s;
        }
    }
}

void MaximumVelocityFilter::calcSecondStage(
        const std::vector<double> & positions, const std::vector<double> & max_vels,
        const Line & obstacle_line, const double & margin_s2, const double & obstacle_vel,
        const double & obstacle_offset, const double & v_candidate, const double & t_to_t1,
        const double & s_to_s1, const int & s1_position_id, double & t1_to_t2, double & s1_to_s2,
        int & s2_position_id, std::vector<double> & opt_times, std::vector<double> & opt_positions,
        std::vector<double> & opt_vels)
{
    s2_position_id = -1;
    t1_to_t2 = t_to_t1;
    s1_to_s2 = s_to_s1;

    for (int i = s1_position_id; i < static_cast<int>(max_vels.size()) - 1; ++i) {
        double v = std::min(v_candidate, max_vels[i]);
        double ds = positions[i + 1] - positions[i];
        double tmp_t = t1_to_t2 + ds / v;
        double tmp_s = s1_to_s2 + ds;
        double obs_s_at_t = obstacle_offset + obstacle_vel * (tmp_t - obstacle_line.first.first);

        // Check if the new point will exceed the margin_s2
        if (obs_s_at_t - tmp_s < margin_s2) {
            s2_position_id = i;
            break;
        } else {
            opt_times.push_back(tmp_t);
            opt_positions.push_back(tmp_s);
            opt_vels.push_back(v);
            t1_to_t2 = tmp_t;
            s1_to_s2 = tmp_s;
        }
    }
}

void MaximumVelocityFilter::calcFinalStage(
        const std::vector<double> & positions, const std::vector<double> & max_vels,
        const Line & obstacle_line, const double & obstacle_vel, const double & t1_to_t2,
        const double & s1_to_s2, const int & s2_position_id, std::vector<double> & opt_times,
        std::vector<double> & opt_positions, std::vector<double> & opt_vels)
{
    int obs_end_id = -1;
    double t2_to_t3 = t1_to_t2;
    double s2_to_s3 = s1_to_s2;
    assert(t2_to_t3 <= obstacle_line.second.first);
    for (int i = s2_position_id; i < static_cast<int>(max_vels.size()) - 1; ++i) {
        double v2 = std::min(obstacle_vel, max_vels[i]);
        double ds = positions[i + 1] - positions[i];
        double tmp_t = 0.0;
        double tmp_s = s2_to_s3 + ds;
        if (t2_to_t3 < obstacle_line.second.first && obstacle_line.second.first < t2_to_t3 + ds / v2) {
            double dt1 = obstacle_line.second.first - t2_to_t3;
            double ds1 = v2 * dt1;
            double ds2 = ds - ds1;
            assert(ds2 >= 0.0);
            double dt2 = ds2 / max_vels[i];
            tmp_t = t2_to_t3 + dt1 + dt2;
        } else {
            tmp_t = t2_to_t3 + ds / v2;
        }

        opt_times.push_back(tmp_t);
        opt_positions.push_back(tmp_s);
        opt_vels.push_back(v2);
        t2_to_t3 = tmp_t;
        s2_to_s3 = tmp_s;

        // Check if the new point will exceed the margin_s2
        if (
                obstacle_line.second.first <=
                t2_to_t3)  // If we pass the obstacle's end, we end the calculation
        {
            obs_end_id = i + 1;
            break;
        }
    }

    for (int i = obs_end_id; i < static_cast<int>(max_vels.size()) - 1; ++i) {
        double v = max_vels[i];
        double ds = positions[i + 1] - positions[i];
        opt_times.push_back(opt_times.back() + ds / v);
        opt_positions.push_back(opt_positions.back() + ds);
        opt_vels.push_back(v);
    }
    opt_vels.push_back(0.0);
}

void MaximumVelocityFilter::calcSafetyVelocity(
        const std::vector<double> & positions, const std::vector<double> & max_vels,
        const Obstacle & obstacle, const double v0, const double a0, const double a_decel,
        const double j_decel, const double margin_s2, std::vector<double> & updated_max_positions,
        std::vector<double> & updated_max_vels)
{
    const double ZERO_VEL_THRESHOLD = 1e-3;
    const double v0_obs = obstacle.vel_;
    const double s0_obs = obstacle.s_.first;

    updated_max_positions.resize(positions.size());
    updated_max_vels.resize(max_vels.size());

    // Step1. Consider Jerk
    double t0_decel = std::max((a_decel - a0) / j_decel, 0.0);
    double v1 = v0 + a0 * t0_decel + 0.5 * j_decel * t0_decel * t0_decel;
    if (v1 < 0.0) {
        v1 = 0.0;
        t0_decel = (-a0 + std::sqrt(a0 * a0 - 2 * j_decel * v0)) / std::fabs(j_decel);
        assert(t0_decel >= 0.0);
    }

    const double s1 = std::max(v0 * t0_decel + 0.5 * a0 * t0_decel * t0_decel +
                               (1.0 / 6.0) * j_decel * t0_decel * t0_decel * t0_decel, 0.0);
    const double s1_obs = v0_obs * t0_decel + s0_obs;
    const double v1_obs = v0_obs;

    // Step2. Check if deceleration is applicable
    double t1_decel = 0.0;
    if (v1 < ZERO_VEL_THRESHOLD) {
        t1_decel = 0.0;
    } else {
        const double discriminant =
                4 * std::pow(v1 - v1_obs, 2) - 8 * a_decel * (margin_s2 - s1_obs + s1);
        assert(discriminant > 0.0);
        t1_decel = std::min(
                (-2 * (v1 - v1_obs) - std::sqrt(discriminant)) / (2 * a_decel), v1 / std::fabs(a_decel));
    }

    // Step3. Store Result
    if (t1_decel < 0.0) {
        std::cerr << "Deceleration Duration is negative: Emergency Stop" << std::endl;
    } else {
        double s2 = std::max(s1 + v1 * t1_decel + 0.5 * a_decel * t1_decel * t1_decel, 0.0);
        double v2 = v1 + a_decel * t1_decel;
        std::cerr << "v0: " << v0  << "  v1: " << v1 << "  v2: " << v2 << std::endl;
        std::cerr << "s1: " << s1  << "  s2: " << s2 << std::endl;
        std::cerr << "t0_decel: " << t0_decel << "  t1_decel: " << t1_decel << std::endl;

        updated_max_positions.front() = 0.0;
        updated_max_vels.front() = v0;

        // First Stage(Jerk Deceleration)
        size_t first_break_id = positions.size();
        for (size_t i = 1; i < positions.size(); ++i) {
            const double ds = positions[i] - positions[i - 1];
            updated_max_positions[i] = positions[i];
            updated_max_vels[i] = v0;

            if (std::fabs(updated_max_positions[i] - s1) < ds || updated_max_positions[i] > s1) {
                updated_max_vels[i] = v1;
                first_break_id = i + 1;
                break;
            }
        }

        // Second Stage (Constant Acceleration Deceleration)
        size_t second_break_id = positions.size();
        for (size_t i = first_break_id; i < positions.size(); ++i) {
            const double ds = positions[i] - positions[i - 1];
            updated_max_positions[i] = positions[i];
            updated_max_vels[i] =
                    std::sqrt(std::max(updated_max_vels[i - 1] + 2 * a_decel * ds, v2 * v2));

            if (updated_max_positions[i] >= s2) {
                second_break_id = i + 1;
                break;
            }
        }

        // Final Stage
        for (size_t i = second_break_id; i < positions.size(); ++i) {
            updated_max_positions[i] = positions[i];
            updated_max_vels[i] = std::min(v1_obs, max_vels[i]);
        }
        updated_max_vels.back() = 0.0;
    }
}

void MaximumVelocityFilter::calcZeroVelObsVelocity(
        const std::vector<double> & times, const std::vector<double> & positions,
        const std::vector<double> & max_vels, const Obstacle & obstacle, const double & margin_s1,
        const double & margin_s2, std::vector<double> & opt_times, std::vector<double> & opt_positions,
        std::vector<double> & opt_vels)
{
    assert(times.size() == positions.size());
    assert(times.size() == max_vels.size());

    opt_times.reserve(times.size());
    opt_positions.reserve(positions.size());
    opt_vels.reserve(max_vels.size());

    // Step1. Create Obstacle t-s point
    TSPoint obs_ini = std::make_pair(0.0, obstacle.y_intercept_);
    TSPoint obs_end = std::make_pair(obstacle.t_.second, obstacle.s_.second);

    bool is_intersect = false;
    TSPoint collision_point;
    for (size_t id = 0; id < positions.size() - 1; ++id) {
        TSPoint pos_ini = std::make_pair(times[id], positions[id]);
        TSPoint pos_end = std::make_pair(times[id + 1], positions[id + 1]);
        is_intersect = calcIntersectionPoint(pos_ini, pos_end, obs_ini, obs_end, collision_point);
        if (is_intersect) break;
    }

    if (!is_intersect) {
        opt_times = times;
        opt_positions = positions;
        opt_vels = max_vels;
        return;
    }

    Line obs_line =
            std::make_pair(collision_point, std::make_pair(obstacle.t_.second, obstacle.s_.second));

    // 2. Update Optimal Velocity
    // If the vehicle is in the s_1 range
    int s1_position_id = -1;
    double s_to_s1;
    double t_to_t1;
    calcFirstStage(
            positions, max_vels, obs_line, margin_s1, obstacle.vel_, t_to_t1, s_to_s1, s1_position_id,
            opt_times, opt_positions, opt_vels);

    // Update Velocity from s_1 to s_2
    int s2_position_id = -1;
    double t1_to_t2;
    double s1_to_s2;
    double obs_offset = obs_line.first.second;
    double v_candidate = (obs_line.second.second - s_to_s1) / (obs_line.second.first - t_to_t1);
    calcSecondStage(
            positions, max_vels, obs_line, margin_s2, obstacle.vel_, obs_offset, v_candidate, t_to_t1,
            s_to_s1, s1_position_id, t1_to_t2, s1_to_s2, s2_position_id, opt_times, opt_positions,
            opt_vels);

    assert(t1_to_t2 <= obs_line.second.first);

    // Update Velocity from s_2 to s_end
    for (size_t i = s2_position_id; i < max_vels.size(); ++i) {
        opt_vels.push_back(0.0);
        opt_times.push_back(opt_times.back() + 0.1);
        opt_positions.push_back(opt_positions.back());
    }
}

void MaximumVelocityFilter::calcPosInterceptObsVelocity(
        const std::vector<double> & times, const std::vector<double> & positions,
        const std::vector<double> & max_vels, const Obstacle & obstacle, const double & margin_s1,
        const double & margin_s2, std::vector<double> & opt_times, std::vector<double> & opt_positions,
        std::vector<double> & opt_vels)
{
    opt_times.reserve(times.size());
    opt_positions.reserve(positions.size());
    opt_vels.reserve(max_vels.size());

    // 1. Detect the collision point(start from time 0)
    TSPoint obs_ini = std::make_pair(0.0, obstacle.y_intercept_);
    TSPoint obs_end = std::make_pair(obstacle.t_.second, obstacle.s_.second);
    assert(obstacle.vel_ > 1e-6);

    bool is_intersect = false;
    TSPoint collision_point;
    for (size_t id = 0; id < positions.size() - 1; ++id) {
        TSPoint pos_ini = std::make_pair(times[id], positions[id]);
        TSPoint pos_end = std::make_pair(times[id + 1], positions[id + 1]);
        is_intersect = calcIntersectionPoint(pos_ini, pos_end, obs_ini, obs_end, collision_point);
        if (is_intersect) break;
    }

    if (!is_intersect) {
        opt_times = times;
        opt_positions = positions;
        opt_vels = max_vels;
        return;
    }

    if (obstacle.t_.first < collision_point.first) {
        collision_point.first = obstacle.t_.first;
        collision_point.second = obstacle.s_.first;
    }
    Line obs_line =
            std::make_pair(collision_point, std::make_pair(obstacle.t_.second, obstacle.s_.second));

    // 2. Update Optimal Velocity
    // Update Velocity from 0 to s_1
    int s1_position_id = -1;
    double s_to_s1;
    double t_to_t1;
    calcFirstStage(
            positions, max_vels, obs_line, margin_s1, obstacle.vel_, t_to_t1, s_to_s1, s1_position_id,
            opt_times, opt_positions, opt_vels);
    assert(obs_line.second.first > t_to_t1);

    // 3. Update Velocity from s_1 to s_2
    int s2_position_id = -1;
    double t1_to_t2 = t_to_t1;
    double s1_to_s2 = s_to_s1;
    double obs_offset = obs_line.first.second;
    double v_candidate = (obs_line.second.second - s_to_s1) / (obs_line.second.first - t_to_t1);
    calcSecondStage(
            positions, max_vels, obs_line, margin_s2, obstacle.vel_, obs_offset, v_candidate, t_to_t1,
            s_to_s1, s1_position_id, t1_to_t2, s1_to_s2, s2_position_id, opt_times, opt_positions,
            opt_vels);

    // 4. Run with obstacle's velocity
    calcFinalStage(
            positions, max_vels, obs_line, obstacle.vel_, t1_to_t2, s1_to_s2, s2_position_id, opt_times,
            opt_positions, opt_vels);
}

void MaximumVelocityFilter::calcNegRightInterceptObsVelocity(
        const std::vector<double> & times, const std::vector<double> & positions,
        const std::vector<double> & max_vels, const Obstacle & obstacle, const double & margin_s1,
        const double & margin_s2, std::vector<double> & opt_times, std::vector<double> & opt_positions,
        std::vector<double> & opt_vels)
{
    opt_times.reserve(times.size());
    opt_positions.reserve(positions.size());
    opt_vels.reserve(max_vels.size());

    // Step 1. Detect the collision point(start from time 0)
    TSPoint obs_ini = std::make_pair(0.0, obstacle.s_.first);
    TSPoint obs_end = std::make_pair(obstacle.t_.first, obstacle.s_.first);
    assert(obstacle.vel_ > 1e-6);

    bool is_intersect = false;
    TSPoint collision_point;
    for (size_t id = 0; id < positions.size() - 1; ++id) {
        TSPoint pos_ini = std::make_pair(times[id], positions[id]);
        TSPoint pos_end = std::make_pair(times[id + 1], positions[id + 1]);
        is_intersect = calcIntersectionPoint(pos_ini, pos_end, obs_ini, obs_end, collision_point);
        if (is_intersect) break;
    }

    if (!is_intersect) {
        opt_times = times;
        opt_positions = positions;
        opt_vels = max_vels;
        return;
    }

    Line obs_line1 =
            std::make_pair(collision_point, std::make_pair(obstacle.t_.first, obstacle.s_.first));
    Line obs_line2 = std::make_pair(
            std::make_pair(obstacle.t_.first, obstacle.s_.first),
            std::make_pair(obstacle.t_.second, obstacle.s_.second));

    // 2. Update Optimal Velocity
    // Update Velocity from 0 to s_1
    int s1_position_id = -1;
    double t_to_t1;
    double s_to_s1;
    calcFirstStage(
            positions, max_vels, obs_line1, margin_s1, 0.0, t_to_t1, s_to_s1, s1_position_id, opt_times,
            opt_positions, opt_vels);
    assert(obs_line1.second.first > t_to_t1);

    // 3. Update Velocity from s_1 to s_2
    int s2_position_id = -1;
    double t1_to_t2;
    double s1_to_s2;
    double obs_offset = obs_line1.second.second;
    double v_candidate =
            (obs_line1.second.second - margin_s2 - s_to_s1) / (obs_line1.second.first - t_to_t1);
    calcSecondStage(
            positions, max_vels, obs_line1, margin_s2, 0.0, obs_offset, v_candidate, t_to_t1, s_to_s1,
            s1_position_id, t1_to_t2, s1_to_s2, s2_position_id, opt_times, opt_positions, opt_vels);

    // 4. Update Velocity at the final stage
    calcFinalStage(
            positions, max_vels, obs_line2, obstacle.vel_, t1_to_t2, s1_to_s2, s2_position_id, opt_times,
            opt_positions, opt_vels);
}

void MaximumVelocityFilter::calcNegLeftInterceptObsVelocity(
        const std::vector<double> & times, const std::vector<double> & positions,
        const std::vector<double> & max_vels, const Obstacle & obstacle, const double & margin_s1,
        const double & margin_s2, std::vector<double> & opt_times, std::vector<double> & opt_positions,
        std::vector<double> & opt_vels)
{
    opt_times.reserve(times.size());
    opt_positions.reserve(positions.size());
    opt_vels.reserve(max_vels.size());

    // 1. Detect the collision point(start from time 0)
    TSPoint obs_ini = std::make_pair(0.0, obstacle.y_intercept_);
    TSPoint obs_end = std::make_pair(obstacle.t_.second, obstacle.s_.second);
    assert(obstacle.vel_ > 1e-6);

    bool is_intersect = false;
    TSPoint collision_point;
    for (size_t id = 0; id < positions.size() - 1; ++id) {
        TSPoint pos_ini = std::make_pair(times[id], positions[id]);
        TSPoint pos_end = std::make_pair(times[id + 1], positions[id + 1]);
        is_intersect = calcIntersectionPoint(pos_ini, pos_end, obs_ini, obs_end, collision_point);
        if (is_intersect) break;
    }

    if (!is_intersect) {
        opt_times = times;
        opt_positions = positions;
        opt_vels = max_vels;
        return;
    }

    if (obstacle.t_.first < collision_point.first) {
        collision_point.first = obstacle.t_.first;
        collision_point.second = obstacle.s_.first;
    } else if (
            collision_point.first < obstacle.t_.first &&
            obstacle.s_.first - collision_point.second > margin_s1) {
        collision_point.first = obstacle.t_.first;
        collision_point.second = obstacle.s_.first;
    }
    Line obs_line =
            std::make_pair(collision_point, std::make_pair(obstacle.t_.second, obstacle.s_.second));

    // 2. Update Optimal Velocity
    // Update Velocity from 0 to s_1
    int s1_position_id = -1;
    double s_to_s1;
    double t_to_t1;
    calcFirstStage(
            positions, max_vels, obs_line, margin_s1, obstacle.vel_, t_to_t1, s_to_s1, s1_position_id,
            opt_times, opt_positions, opt_vels);
    assert(obs_line.second.first > t_to_t1);

    // 3. Update Velocity from s_1 to s_2
    int s2_position_id = -1;
    double t1_to_t2 = t_to_t1;
    double s1_to_s2 = s_to_s1;
    double obs_offset = obs_line.first.second;
    double v_candidate = (obs_line.second.second - s_to_s1) / (obs_line.second.first - t_to_t1);
    calcSecondStage(
            positions, max_vels, obs_line, margin_s2, obstacle.vel_, obs_offset, v_candidate, t_to_t1,
            s_to_s1, s1_position_id, t1_to_t2, s1_to_s2, s2_position_id, opt_times, opt_positions,
            opt_vels);

    // 4. Run with obstacle's velocity
    calcFinalStage(
            positions, max_vels, obs_line, obstacle.vel_, t1_to_t2, s1_to_s2, s2_position_id, opt_times,
            opt_positions, opt_vels);
}

void MaximumVelocityFilter::mergeFilteredVelocity(const std::vector<double> &forward_vels,
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
