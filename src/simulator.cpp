#include "controller.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

struct ScenarioRow {
    std::uint32_t timestamp{0};
    mobility::Command command{mobility::Command::Stop};
    mobility::SensorFrame sensors{};
};

ScenarioRow parse_row(const std::string& line, std::size_t number) {
    std::stringstream input(line);
    std::string item;
    ScenarioRow row;
    if (!std::getline(input, item, ',')) throw std::invalid_argument("missing timestamp on row " + std::to_string(number));
    row.timestamp = static_cast<std::uint32_t>(std::stoul(item));
    if (!std::getline(input, item, ',')) throw std::invalid_argument("missing command on row " + std::to_string(number));
    row.command = mobility::command_from_string(item);
    auto read_double = [&](double& target) {
        if (std::getline(input, item, ',') && !item.empty()) target = std::stod(item);
    };
    read_double(row.sensors.front_distance_cm);
    read_double(row.sensors.rear_distance_cm);
    read_double(row.sensors.left_distance_cm);
    read_double(row.sensors.right_distance_cm);
    read_double(row.sensors.battery_voltage);
    if (std::getline(input, item, ',') && !item.empty()) row.sensors.emergency_stop = std::stoi(item) != 0;
    if (std::getline(input, item, ',') && !item.empty()) row.sensors.seat_occupied = std::stoi(item) != 0;
    return row;
}

void print_header() {
    std::cout << "timestamp_ms,command,state,fault,left_pwm,right_pwm,brake,battery_percent,nearest_obstacle_cm,safety_scale\n";
}

void print_telemetry(const mobility::Telemetry& t) {
    std::cout << t.timestamp_ms << ',' << mobility::to_string(t.command) << ',' << mobility::to_string(t.safety_state)
              << ',' << mobility::to_string(t.fault) << ',' << t.output.left_pwm << ',' << t.output.right_pwm << ','
              << t.output.brake << ',' << std::fixed << std::setprecision(1) << t.battery_percent << ','
              << t.nearest_obstacle_cm << ',' << t.safety_speed_scale << '\n';
}

void demo() {
    mobility::Controller controller;
    print_header();
    mobility::SensorFrame sensors;
    controller.command(mobility::Command::Forward, 0);
    for (std::uint32_t now : {0u, 100u, 200u, 300u}) {
        sensors.front_distance_cm = now < 200 ? 200.0 : 55.0;
        controller.update(now, sensors);
        print_telemetry(controller.telemetry());
    }
    sensors.front_distance_cm = 15.0;
    controller.update(400, sensors);
    print_telemetry(controller.telemetry());
    controller.command(mobility::Command::Reverse, 450);
    sensors.front_distance_cm = 200.0;
    controller.update(500, sensors);
    print_telemetry(controller.telemetry());
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 1 || (argc == 2 && std::string(argv[1]) == "--demo")) {
            demo();
            return 0;
        }
        if (argc != 2) {
            std::cerr << "Usage: mobility_simulator [--demo|scenario.csv]\n";
            return 1;
        }
        std::ifstream file(argv[1]);
        if (!file) throw std::runtime_error("unable to open scenario file");
        mobility::Controller controller;
        print_header();
        std::string line;
        std::size_t number = 0;
        while (std::getline(file, line)) {
            ++number;
            if (line.empty() || line[0] == '#') continue;
            const auto row = parse_row(line, number);
            controller.command(row.command, row.timestamp);
            controller.update(row.timestamp, row.sensors);
            print_telemetry(controller.telemetry());
        }
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
