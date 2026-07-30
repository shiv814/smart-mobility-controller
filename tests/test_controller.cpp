#include "controller.hpp"

#include <cassert>
#include <iostream>

int main() {
    mobility::Controller controller(200, 50, 500);
    controller.command(mobility::Command::Forward, 0);
    auto output = controller.update(20);
    assert(output.left_pwm == 50 && output.right_pwm == 50);
    output = controller.update(80);
    assert(output.left_pwm == 100 && output.right_pwm == 100);

    controller.command(mobility::Command::Left, 100);
    output = controller.update(120);
    assert(output.left_pwm == 50 && output.right_pwm == 100);

    output = controller.update(700);
    assert(controller.timed_out());
    assert(output.left_pwm == 0 && output.right_pwm == 50);
    output = controller.update(720);
    assert(output.left_pwm == 0 && output.right_pwm == 0);
    std::cout << "All mobility controller tests passed\n";
    return 0;
}
