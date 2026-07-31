#include "controller.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace mobility;

#define CHECK(condition) do { if (!(condition)) { std::cerr << "Check failed at " << __FILE__ << ":" << __LINE__ << ": " #condition << "\n"; std::exit(1); } } while (false)

void test_ramping_and_reversal_guard() {
    ControllerConfig config;
    config.maximum_pwm = 200;
    config.acceleration_step = 50;
    config.deceleration_step = 100;
    config.command_timeout_ms = 500;
    Controller controller(config);
    controller.set_drive_mode(DriveMode::Sport);
    controller.command(Command::Forward, 0);
    auto output = controller.update(20);
    CHECK(output.left_pwm == 50 && output.right_pwm == 50);
    output = controller.update(80);
    CHECK(output.left_pwm == 100 && output.right_pwm == 100);
    controller.command(Command::Reverse, 100);
    output = controller.update(120);
    CHECK(output.left_pwm == 0 && output.right_pwm == 0); // brake before reversing
    output = controller.update(140);
    CHECK(output.left_pwm == -50 && output.right_pwm == -50);
}

void test_obstacle_slowdown_and_stop() {
    Controller controller;
    controller.set_drive_mode(DriveMode::Sport);
    controller.command(Command::Forward, 0);
    SensorFrame sensors;
    sensors.front_distance_cm = 50.0;
    auto output = controller.update(10, sensors);
    CHECK(controller.telemetry().safety_state == SafetyState::Degraded);
    CHECK(controller.telemetry().safety_speed_scale > 0.0 && controller.telemetry().safety_speed_scale < 1.0);
    CHECK(output.left_pwm > 0);
    sensors.front_distance_cm = 20.0;
    output = controller.update(20, sensors);
    CHECK(controller.telemetry().fault == FaultCode::Obstacle);
    CHECK(controller.telemetry().safety_state == SafetyState::Stopped);
}

void test_timeout_and_recovery_on_new_command() {
    Controller controller(200, 50, 500);
    controller.command(Command::Forward, 0);
    controller.update(100);
    controller.update(600);
    CHECK(controller.timed_out());
    CHECK(controller.telemetry().fault == FaultCode::CommandTimeout);
    controller.command(Command::Left, 610);
    controller.update(620);
    CHECK(!controller.timed_out());
    CHECK(controller.telemetry().fault == FaultCode::None);
}

void test_emergency_stop_latches_and_requires_clear() {
    Controller controller;
    controller.command(Command::Forward, 0);
    SensorFrame sensors;
    sensors.emergency_stop = true;
    controller.update(10, sensors);
    CHECK(controller.fault_latched());
    CHECK(controller.latched_fault() == FaultCode::EmergencyStop);
    sensors.emergency_stop = false;
    controller.update(20, sensors);
    CHECK(controller.telemetry().fault == FaultCode::EmergencyStop);
    CHECK(controller.clear_fault(30, sensors));
    controller.update(40, sensors);
    CHECK(!controller.fault_latched());
}

void test_battery_derating_and_critical_fault() {
    Controller controller;
    controller.set_drive_mode(DriveMode::Sport);
    controller.command(Command::Forward, 0);
    SensorFrame sensors;
    sensors.battery_voltage = 21.5;
    controller.update(10, sensors);
    CHECK(controller.telemetry().safety_state == SafetyState::Degraded);
    CHECK(controller.telemetry().safety_speed_scale <= 0.55);
    sensors.battery_voltage = 19.5;
    controller.update(20, sensors);
    CHECK(controller.latched_fault() == FaultCode::BatteryCritical);
}

void test_joystick_mixing_and_deadzone() {
    ControllerConfig config;
    config.acceleration_step = 255;
    config.deceleration_step = 255;
    Controller controller(config);
    controller.set_drive_mode(DriveMode::Sport);
    controller.joystick(0.5, 1.0, 0);
    auto output = controller.update(10);
    CHECK(output.left_pwm > output.right_pwm);
    CHECK(output.right_pwm > 0);
    controller.joystick(0.01, 0.01, 20);
    controller.update(30);
    CHECK(controller.telemetry().command == Command::Stop);
}

void test_seat_and_sensor_safety() {
    Controller controller;
    controller.command(Command::Forward, 0);
    SensorFrame sensors;
    sensors.seat_occupied = false;
    controller.update(10, sensors);
    CHECK(controller.telemetry().fault == FaultCode::SeatUnoccupied);
    sensors.seat_occupied = true;
    sensors.front_distance_cm = -1;
    controller.update(20, sensors);
    CHECK(controller.latched_fault() == FaultCode::SensorFault);
}

void test_event_log_and_validation() {
    ControllerConfig config;
    config.event_log_capacity = 3;
    Controller controller(config);
    controller.command(Command::Forward, 1);
    controller.command(Command::Left, 2);
    controller.command(Command::Stop, 3);
    CHECK(controller.event_log().size() == 3);
    bool failed = false;
    try { Controller invalid(300, 10, 500); }
    catch (const std::invalid_argument&) { failed = true; }
    CHECK(failed);
}

int main() {
    test_ramping_and_reversal_guard();
    test_obstacle_slowdown_and_stop();
    test_timeout_and_recovery_on_new_command();
    test_emergency_stop_latches_and_requires_clear();
    test_battery_derating_and_critical_fault();
    test_joystick_mixing_and_deadzone();
    test_seat_and_sensor_safety();
    test_event_log_and_validation();
    std::cout << "All Smart Mobility Controller 2.0 tests passed\n";
    return 0;
}
