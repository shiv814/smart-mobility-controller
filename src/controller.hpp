#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace mobility {

enum class Command { Stop, Forward, Reverse, Left, Right, Joystick };
enum class DriveMode { Eco, Normal, Sport };
enum class SafetyState { Ready, Degraded, Stopped, Fault };
enum class FaultCode { None, CommandTimeout, EmergencyStop, SeatUnoccupied, Obstacle, BatteryCritical, SensorFault };

std::string to_string(Command value);
std::string to_string(DriveMode value);
std::string to_string(SafetyState value);
std::string to_string(FaultCode value);
Command command_from_string(const std::string& value);

struct MotorOutput {
    int left_pwm{0};
    int right_pwm{0};
    bool brake{true};
};

struct SensorFrame {
    double front_distance_cm{200.0};
    double rear_distance_cm{200.0};
    double left_distance_cm{200.0};
    double right_distance_cm{200.0};
    double battery_voltage{24.0};
    bool emergency_stop{false};
    bool seat_occupied{true};
};

struct ControllerConfig {
    int maximum_pwm{220};
    int acceleration_step{20};
    int deceleration_step{40};
    std::uint32_t command_timeout_ms{750};
    double obstacle_stop_cm{25.0};
    double obstacle_slowdown_cm{80.0};
    double low_battery_voltage{22.0};
    double critical_battery_voltage{20.0};
    double full_battery_voltage{25.2};
    double joystick_deadzone{0.08};
    std::size_t event_log_capacity{64};
};

struct Telemetry {
    std::uint64_t sequence{0};
    std::uint32_t timestamp_ms{0};
    Command command{Command::Stop};
    DriveMode drive_mode{DriveMode::Normal};
    SafetyState safety_state{SafetyState::Stopped};
    FaultCode fault{FaultCode::None};
    MotorOutput output{};
    double requested_speed_scale{0.0};
    double safety_speed_scale{0.0};
    double battery_percent{0.0};
    double nearest_obstacle_cm{0.0};
    bool timed_out{false};
    bool fault_latched{false};
};

class Controller {
public:
    explicit Controller(ControllerConfig config = {});
    Controller(int maximum_pwm, int ramp_step, std::uint32_t timeout_ms);

    void command(Command value, std::uint32_t now_ms);
    void joystick(double x, double y, std::uint32_t now_ms);
    void set_drive_mode(DriveMode mode);
    MotorOutput update(std::uint32_t now_ms, const SensorFrame& sensors = {});
    bool clear_fault(std::uint32_t now_ms, const SensorFrame& sensors = {});

    const Telemetry& telemetry() const { return telemetry_; }
    bool timed_out() const { return telemetry_.timed_out; }
    bool fault_latched() const { return latched_fault_ != FaultCode::None; }
    FaultCode latched_fault() const { return latched_fault_; }
    const std::deque<std::string>& event_log() const { return event_log_; }
    const ControllerConfig& config() const { return config_; }

private:
    static int approach(int current, int target, int acceleration_step, int deceleration_step);
    static double clamp_unit(double value);
    static bool reverses_direction(int current, int target);
    static bool valid_sensor_frame(const SensorFrame& sensors);

    MotorOutput target_for(Command value, double scale) const;
    double mode_scale() const;
    double obstacle_scale(const SensorFrame& sensors, double* nearest) const;
    double battery_percent(double voltage) const;
    void latch_fault(FaultCode fault, std::uint32_t now_ms);
    void record_event(std::uint32_t now_ms, const std::string& event);

    ControllerConfig config_;
    std::uint32_t last_command_ms_{0};
    Command command_{Command::Stop};
    DriveMode drive_mode_{DriveMode::Normal};
    MotorOutput output_{};
    double joystick_left_{0.0};
    double joystick_right_{0.0};
    FaultCode latched_fault_{FaultCode::None};
    FaultCode previous_reported_fault_{FaultCode::None};
    SafetyState previous_state_{SafetyState::Stopped};
    Telemetry telemetry_{};
    std::deque<std::string> event_log_;
};

}  // namespace mobility
