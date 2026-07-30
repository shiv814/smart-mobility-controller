#include "controller.hpp"

#include <algorithm>
#include <stdexcept>

namespace mobility {

Controller::Controller(int maximum_pwm, int ramp_step, std::uint32_t timeout_ms)
    : maximum_pwm_(maximum_pwm), ramp_step_(ramp_step), timeout_ms_(timeout_ms) {
    if (maximum_pwm <= 0 || maximum_pwm > 255 || ramp_step <= 0 || timeout_ms == 0) {
        throw std::invalid_argument("invalid controller configuration");
    }
}

void Controller::command(Command value, std::uint32_t now_ms) {
    command_ = value;
    last_command_ms_ = now_ms;
    timed_out_ = false;
}

int Controller::approach(int current, int target, int step) {
    if (current < target) return std::min(current + step, target);
    if (current > target) return std::max(current - step, target);
    return current;
}

MotorOutput Controller::target_for(Command value) const {
    switch (value) {
        case Command::Forward: return {maximum_pwm_, maximum_pwm_};
        case Command::Reverse: return {-maximum_pwm_, -maximum_pwm_};
        case Command::Left: return {-maximum_pwm_ / 2, maximum_pwm_ / 2};
        case Command::Right: return {maximum_pwm_ / 2, -maximum_pwm_ / 2};
        case Command::Stop: default: return {0, 0};
    }
}

MotorOutput Controller::update(std::uint32_t now_ms) {
    if (now_ms - last_command_ms_ > timeout_ms_) {
        command_ = Command::Stop;
        timed_out_ = true;
    }
    const auto target = target_for(command_);
    output_.left_pwm = approach(output_.left_pwm, target.left_pwm, ramp_step_);
    output_.right_pwm = approach(output_.right_pwm, target.right_pwm, ramp_step_);
    return output_;
}

}  // namespace mobility
