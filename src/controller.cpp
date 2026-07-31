#include "controller.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mobility {
namespace {

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

}  // namespace

std::string to_string(Command value) {
    switch (value) {
        case Command::Stop: return "stop";
        case Command::Forward: return "forward";
        case Command::Reverse: return "reverse";
        case Command::Left: return "left";
        case Command::Right: return "right";
        case Command::Joystick: return "joystick";
    }
    return "unknown";
}

std::string to_string(DriveMode value) {
    switch (value) {
        case DriveMode::Eco: return "eco";
        case DriveMode::Normal: return "normal";
        case DriveMode::Sport: return "sport";
    }
    return "unknown";
}

std::string to_string(SafetyState value) {
    switch (value) {
        case SafetyState::Ready: return "ready";
        case SafetyState::Degraded: return "degraded";
        case SafetyState::Stopped: return "stopped";
        case SafetyState::Fault: return "fault";
    }
    return "unknown";
}

std::string to_string(FaultCode value) {
    switch (value) {
        case FaultCode::None: return "none";
        case FaultCode::CommandTimeout: return "command-timeout";
        case FaultCode::EmergencyStop: return "emergency-stop";
        case FaultCode::SeatUnoccupied: return "seat-unoccupied";
        case FaultCode::Obstacle: return "obstacle";
        case FaultCode::BatteryCritical: return "battery-critical";
        case FaultCode::SensorFault: return "sensor-fault";
    }
    return "unknown";
}

Command command_from_string(const std::string& value) {
    const auto cleaned = lower(value);
    if (cleaned == "stop" || cleaned == "s") return Command::Stop;
    if (cleaned == "forward" || cleaned == "f") return Command::Forward;
    if (cleaned == "reverse" || cleaned == "back" || cleaned == "b") return Command::Reverse;
    if (cleaned == "left" || cleaned == "l") return Command::Left;
    if (cleaned == "right" || cleaned == "r") return Command::Right;
    throw std::invalid_argument("unknown command: " + value);
}

Controller::Controller(ControllerConfig config) : config_(config) {
    if (config_.maximum_pwm <= 0 || config_.maximum_pwm > 255) throw std::invalid_argument("maximum PWM must be between 1 and 255");
    if (config_.acceleration_step <= 0 || config_.deceleration_step <= 0) throw std::invalid_argument("ramp steps must be positive");
    if (config_.command_timeout_ms == 0) throw std::invalid_argument("command timeout must be positive");
    if (!(0.0 < config_.obstacle_stop_cm && config_.obstacle_stop_cm < config_.obstacle_slowdown_cm)) throw std::invalid_argument("obstacle thresholds are invalid");
    if (!(config_.critical_battery_voltage < config_.low_battery_voltage && config_.low_battery_voltage < config_.full_battery_voltage)) throw std::invalid_argument("battery thresholds are invalid");
    if (config_.joystick_deadzone < 0.0 || config_.joystick_deadzone >= 1.0) throw std::invalid_argument("joystick deadzone must be in [0, 1)");
    if (config_.event_log_capacity == 0) throw std::invalid_argument("event log capacity must be positive");
    record_event(0, "controller initialized");
}

Controller::Controller(int maximum_pwm, int ramp_step, std::uint32_t timeout_ms)
    : Controller(ControllerConfig{maximum_pwm, ramp_step, ramp_step, timeout_ms}) {}

void Controller::record_event(std::uint32_t now_ms, const std::string& event) {
    std::ostringstream line;
    line << now_ms << "ms: " << event;
    event_log_.push_back(line.str());
    while (event_log_.size() > config_.event_log_capacity) event_log_.pop_front();
}

void Controller::command(Command value, std::uint32_t now_ms) {
    if (value == Command::Joystick) throw std::invalid_argument("use joystick() for joystick input");
    command_ = value;
    last_command_ms_ = now_ms;
    telemetry_.timed_out = false;
    record_event(now_ms, "command=" + to_string(value));
}

double Controller::clamp_unit(double value) {
    return std::clamp(value, -1.0, 1.0);
}

void Controller::joystick(double x, double y, std::uint32_t now_ms) {
    if (!std::isfinite(x) || !std::isfinite(y)) throw std::invalid_argument("joystick values must be finite");
    x = clamp_unit(x);
    y = clamp_unit(y);
    if (std::abs(x) < config_.joystick_deadzone) x = 0.0;
    if (std::abs(y) < config_.joystick_deadzone) y = 0.0;
    double left = y + x;
    double right = y - x;
    const double magnitude = std::max({1.0, std::abs(left), std::abs(right)});
    joystick_left_ = left / magnitude;
    joystick_right_ = right / magnitude;
    command_ = (joystick_left_ == 0.0 && joystick_right_ == 0.0) ? Command::Stop : Command::Joystick;
    last_command_ms_ = now_ms;
    telemetry_.timed_out = false;
    record_event(now_ms, "joystick input updated");
}

void Controller::set_drive_mode(DriveMode mode) {
    if (mode != drive_mode_) record_event(telemetry_.timestamp_ms, "drive mode=" + to_string(mode));
    drive_mode_ = mode;
}

int Controller::approach(int current, int target, int acceleration_step, int deceleration_step) {
    if (current == target) return current;
    const bool increasing_magnitude = (current == 0 || (current > 0) == (target > 0)) && std::abs(target) > std::abs(current);
    const int step = increasing_magnitude ? acceleration_step : deceleration_step;
    if (current < target) return std::min(current + step, target);
    return std::max(current - step, target);
}

bool Controller::reverses_direction(int current, int target) {
    return current != 0 && target != 0 && ((current > 0) != (target > 0));
}

bool Controller::valid_sensor_frame(const SensorFrame& sensors) {
    const double values[] = {sensors.front_distance_cm, sensors.rear_distance_cm, sensors.left_distance_cm, sensors.right_distance_cm, sensors.battery_voltage};
    for (const double value : values) {
        if (!std::isfinite(value) || value < 0.0) return false;
    }
    return true;
}

double Controller::mode_scale() const {
    switch (drive_mode_) {
        case DriveMode::Eco: return 0.55;
        case DriveMode::Normal: return 0.80;
        case DriveMode::Sport: return 1.00;
    }
    return 0.0;
}

MotorOutput Controller::target_for(Command value, double scale) const {
    const int maximum = static_cast<int>(std::lround(config_.maximum_pwm * std::clamp(scale, 0.0, 1.0)));
    switch (value) {
        case Command::Forward: return {maximum, maximum, maximum == 0};
        case Command::Reverse: return {-maximum, -maximum, maximum == 0};
        case Command::Left: return {-maximum / 2, maximum / 2, maximum == 0};
        case Command::Right: return {maximum / 2, -maximum / 2, maximum == 0};
        case Command::Joystick:
            return {
                static_cast<int>(std::lround(maximum * joystick_left_)),
                static_cast<int>(std::lround(maximum * joystick_right_)),
                maximum == 0,
            };
        case Command::Stop: default: return {0, 0, true};
    }
}

double Controller::obstacle_scale(const SensorFrame& sensors, double* nearest) const {
    double distance = 200.0;
    switch (command_) {
        case Command::Forward: distance = sensors.front_distance_cm; break;
        case Command::Reverse: distance = sensors.rear_distance_cm; break;
        case Command::Left: distance = std::min(sensors.front_distance_cm, sensors.left_distance_cm); break;
        case Command::Right: distance = std::min(sensors.front_distance_cm, sensors.right_distance_cm); break;
        case Command::Joystick:
            if (joystick_left_ + joystick_right_ >= 0) distance = sensors.front_distance_cm;
            else distance = sensors.rear_distance_cm;
            if (joystick_left_ > joystick_right_) distance = std::min(distance, sensors.left_distance_cm);
            if (joystick_right_ > joystick_left_) distance = std::min(distance, sensors.right_distance_cm);
            break;
        case Command::Stop: distance = std::min({sensors.front_distance_cm, sensors.rear_distance_cm, sensors.left_distance_cm, sensors.right_distance_cm}); break;
    }
    if (nearest) *nearest = distance;
    if (distance <= config_.obstacle_stop_cm) return 0.0;
    if (distance >= config_.obstacle_slowdown_cm) return 1.0;
    return (distance - config_.obstacle_stop_cm) / (config_.obstacle_slowdown_cm - config_.obstacle_stop_cm);
}

double Controller::battery_percent(double voltage) const {
    const auto fraction = (voltage - config_.critical_battery_voltage) /
                          (config_.full_battery_voltage - config_.critical_battery_voltage);
    return std::clamp(fraction * 100.0, 0.0, 100.0);
}

void Controller::latch_fault(FaultCode fault, std::uint32_t now_ms) {
    if (latched_fault_ == FaultCode::None) {
        latched_fault_ = fault;
        record_event(now_ms, "latched fault=" + to_string(fault));
    }
}

bool Controller::clear_fault(std::uint32_t now_ms, const SensorFrame& sensors) {
    if (sensors.emergency_stop || !sensors.seat_occupied || !valid_sensor_frame(sensors) ||
        sensors.battery_voltage <= config_.critical_battery_voltage) {
        record_event(now_ms, "fault clear rejected: unsafe inputs remain");
        return false;
    }
    if (latched_fault_ != FaultCode::None) {
        record_event(now_ms, "cleared fault=" + to_string(latched_fault_));
        latched_fault_ = FaultCode::None;
        command_ = Command::Stop;
        last_command_ms_ = now_ms;
    }
    return true;
}

MotorOutput Controller::update(std::uint32_t now_ms, const SensorFrame& sensors) {
    ++telemetry_.sequence;
    telemetry_.timestamp_ms = now_ms;
    telemetry_.command = command_;
    telemetry_.drive_mode = drive_mode_;
    telemetry_.timed_out = now_ms - last_command_ms_ > config_.command_timeout_ms;
    telemetry_.battery_percent = battery_percent(sensors.battery_voltage);

    FaultCode transient_fault = FaultCode::None;
    if (!valid_sensor_frame(sensors)) latch_fault(FaultCode::SensorFault, now_ms);
    if (sensors.emergency_stop) latch_fault(FaultCode::EmergencyStop, now_ms);
    if (sensors.battery_voltage <= config_.critical_battery_voltage) latch_fault(FaultCode::BatteryCritical, now_ms);
    if (!sensors.seat_occupied) transient_fault = FaultCode::SeatUnoccupied;
    else if (telemetry_.timed_out) transient_fault = FaultCode::CommandTimeout;

    double nearest = 0.0;
    double safety_scale = obstacle_scale(sensors, &nearest);
    telemetry_.nearest_obstacle_cm = nearest;
    if (safety_scale <= 0.0 && command_ != Command::Stop) transient_fault = FaultCode::Obstacle;
    if (sensors.battery_voltage < config_.low_battery_voltage) safety_scale = std::min(safety_scale, 0.55);
    telemetry_.requested_speed_scale = mode_scale();
    telemetry_.safety_speed_scale = safety_scale;

    FaultCode reported_fault = latched_fault_ != FaultCode::None ? latched_fault_ : transient_fault;
    SafetyState state = SafetyState::Ready;
    double final_scale = mode_scale() * safety_scale;
    if (latched_fault_ != FaultCode::None) {
        state = SafetyState::Fault;
        final_scale = 0.0;
    } else if (transient_fault != FaultCode::None) {
        state = SafetyState::Stopped;
        final_scale = 0.0;
    } else if (safety_scale < 1.0 || drive_mode_ == DriveMode::Eco || sensors.battery_voltage < config_.low_battery_voltage) {
        state = SafetyState::Degraded;
    }

    auto target = target_for(command_, final_scale);
    if (reverses_direction(output_.left_pwm, target.left_pwm)) target.left_pwm = 0;
    if (reverses_direction(output_.right_pwm, target.right_pwm)) target.right_pwm = 0;
    output_.left_pwm = approach(output_.left_pwm, target.left_pwm, config_.acceleration_step, config_.deceleration_step);
    output_.right_pwm = approach(output_.right_pwm, target.right_pwm, config_.acceleration_step, config_.deceleration_step);
    output_.brake = target.brake && output_.left_pwm == 0 && output_.right_pwm == 0;

    telemetry_.safety_state = state;
    telemetry_.fault = reported_fault;
    telemetry_.fault_latched = latched_fault_ != FaultCode::None;
    telemetry_.output = output_;

    if (reported_fault != previous_reported_fault_) {
        record_event(now_ms, "reported fault=" + to_string(reported_fault));
        previous_reported_fault_ = reported_fault;
    }
    if (state != previous_state_) {
        record_event(now_ms, "safety state=" + to_string(state));
        previous_state_ = state;
    }
    return output_;
}

}  // namespace mobility
