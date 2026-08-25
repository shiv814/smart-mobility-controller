#include "arbitration.hpp"
#include "diagnostics.hpp"
#include "energy.hpp"
#include "odometry.hpp"

#include <cmath>
#include <iostream>

#define CHECK(expression) do { if (!(expression)) { std::cerr << "CHECK failed: " #expression << " at " << __FILE__ << ':' << __LINE__ << '\n'; return 1; } } while (0)

int main() {
    using namespace mobility;

    CommandArbiter arbiter;
    arbiter.submit({CommandSource::User, Command::Forward, 10, 0, 500});
    arbiter.submit({CommandSource::Navigation, Command::Left, 20, 10, 500});
    CHECK(arbiter.decide(20).command == Command::Left);
    CHECK(arbiter.decide(20, true).command == Command::Stop);
    CHECK(arbiter.decide(1000).fallback_stop);

    EnergyEstimator energy;
    const auto full = energy.estimate(25.2);
    const auto low = energy.estimate(20.5);
    CHECK(full.estimated_range_km > low.estimated_range_km);
    CHECK(low.reserve_active);

    DifferentialOdometry odometry;
    odometry.update({0, 0});
    const auto pose = odometry.update({1024, 1024});
    CHECK(pose.distance_m > 0.8 && pose.distance_m < 0.83);
    CHECK(std::abs(pose.y_m) < 1e-9);

    DiagnosticMonitor monitor;
    Telemetry telemetry;
    telemetry.safety_state = SafetyState::Ready;
    telemetry.battery_percent = 80;
    telemetry.nearest_obstacle_cm = 100;
    monitor.observe(telemetry);
    telemetry.safety_state = SafetyState::Fault;
    telemetry.fault = FaultCode::SensorFault;
    telemetry.battery_percent = 70;
    telemetry.nearest_obstacle_cm = 40;
    telemetry.requested_speed_scale = 0.8;
    telemetry.safety_speed_scale = 0.0;
    monitor.observe(telemetry);
    CHECK(monitor.kpis().frames == 2);
    CHECK(monitor.kpis().fault_frames == 1);
    CHECK(monitor.kpis().obstacle_interventions == 1);
    CHECK(monitor.kpis().minimum_obstacle_cm == 40);
    CHECK(monitor.to_json().find("safety_availability") != std::string::npos);

    std::cout << "Smart Mobility v3 safety tests passed\n";
    return 0;
}
