#pragma once

#include <cstdint>

namespace mobility {

enum class Command { Stop, Forward, Reverse, Left, Right };

struct MotorOutput {
    int left_pwm{0};
    int right_pwm{0};
};

class Controller {
public:
    Controller(int maximum_pwm = 220, int ramp_step = 20, std::uint32_t timeout_ms = 750);
    void command(Command value, std::uint32_t now_ms);
    MotorOutput update(std::uint32_t now_ms);
    bool timed_out() const { return timed_out_; }

private:
    static int approach(int current, int target, int step);
    MotorOutput target_for(Command value) const;

    int maximum_pwm_;
    int ramp_step_;
    std::uint32_t timeout_ms_;
    std::uint32_t last_command_ms_{0};
    Command command_{Command::Stop};
    MotorOutput output_{};
    bool timed_out_{false};
};

}  // namespace mobility
