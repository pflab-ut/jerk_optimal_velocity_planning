#include "obstacle.h"

Obstacle::Obstacle() : width_(0.0), length_(0.0), vel_(0.0), y_intercept_(0.0)
{
    s_.first = 0.0;
    s_.second = 0.0;
    t_.first = 0.0;
    t_.second = 0.0;
}

Obstacle::Obstacle(
        const double & t_ini, const double & t_end, const double & s_ini, const double & s_end,
        const double & width, const double & length, const double & margin_s)
        : width_(width), length_(length)
{
    assert(t_ini <= t_end);

    s_actual_.first = s_ini;
    s_actual_.second = s_end;
    t_actual_.first = t_ini;
    t_actual_.second = t_end;

    if (s_ini < margin_s) {
        double ds = margin_s - s_ini;
        s_.first = 0.0;
        s_.second = s_end - ds;
        t_.first = t_ini;
        t_.second = t_end;
    } else {
        s_.first = s_ini - margin_s;
        s_.second = s_end - margin_s;
        t_.first = t_ini;
        t_.second = t_end;
    }

    if (std::fabs(t_.first- t_.second) < 1e-6) {
        // High Speed Obstacle
        vel_ = 0.0;
        y_intercept_ = s_.first;
    } else if (s_.first> s_.second) {
        // Negative Velocity Obstacle
        vel_ = 0.0;
        y_intercept_ = s_.second;
        s_.first = s_.second;
    } else {
        vel_ = (s_.second- s_.first) / (t_.second- t_.first);
        y_intercept_ = s_.first- vel_ * t_.first;
    }
}
