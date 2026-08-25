#include "odometry.hpp"

#include <cmath>
#include <stdexcept>

namespace mobility {
namespace { constexpr double kPi = 3.14159265358979323846; }

DifferentialOdometry::DifferentialOdometry(OdometryConfig config) : config_(config) {
    if (config_.wheel_diameter_m <= 0 || config_.track_width_m <= 0 || config_.ticks_per_revolution <= 0) {
        throw std::invalid_argument("invalid odometry configuration");
    }
}

Pose2D DifferentialOdometry::update(EncoderFrame frame) {
    if (!initialized_) {
        previous_ = frame;
        initialized_ = true;
        return pose_;
    }
    const auto left_ticks = frame.left_ticks - previous_.left_ticks;
    const auto right_ticks = frame.right_ticks - previous_.right_ticks;
    previous_ = frame;
    const double meters_per_tick = kPi * config_.wheel_diameter_m / static_cast<double>(config_.ticks_per_revolution);
    const double left = left_ticks * meters_per_tick;
    const double right = right_ticks * meters_per_tick;
    const double distance = (left + right) / 2.0;
    const double heading_delta = (right - left) / config_.track_width_m;
    const double midpoint_heading = pose_.heading_rad + heading_delta / 2.0;
    pose_.x_m += distance * std::cos(midpoint_heading);
    pose_.y_m += distance * std::sin(midpoint_heading);
    pose_.heading_rad += heading_delta;
    pose_.distance_m += std::abs(distance);
    return pose_;
}

void DifferentialOdometry::reset(Pose2D pose) {
    pose_ = pose;
    previous_ = {};
    initialized_ = false;
}

}  // namespace mobility
