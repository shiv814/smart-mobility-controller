#pragma once

#include <cstdint>

namespace mobility {

struct OdometryConfig {
    double wheel_diameter_m{0.26};
    double track_width_m{0.48};
    std::int64_t ticks_per_revolution{1024};
};

struct EncoderFrame { std::int64_t left_ticks{0}; std::int64_t right_ticks{0}; };
struct Pose2D { double x_m{0.0}; double y_m{0.0}; double heading_rad{0.0}; double distance_m{0.0}; };

class DifferentialOdometry {
public:
    explicit DifferentialOdometry(OdometryConfig config = {});
    Pose2D update(EncoderFrame frame);
    void reset(Pose2D pose = {});
    const Pose2D& pose() const { return pose_; }

private:
    OdometryConfig config_;
    Pose2D pose_{};
    EncoderFrame previous_{};
    bool initialized_{false};
};

}  // namespace mobility
