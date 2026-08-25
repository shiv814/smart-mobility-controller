#include "arbitration.hpp"
#include "diagnostics.hpp"
#include "energy.hpp"
#include "odometry.hpp"

#include <iostream>

int main() {
    using namespace mobility;
    CommandArbiter arbiter;
    arbiter.submit({CommandSource::User, Command::Forward, 10, 100, 500});
    arbiter.submit({CommandSource::Navigation, Command::Left, 20, 110, 500});
    const auto decision = arbiter.decide(120);
    std::cout << "command=" << to_string(decision.command) << " source=" << to_string(decision.source) << '\n';

    EnergyEstimator energy;
    const auto estimate = energy.estimate(23.4, 32.0, 6.0);
    std::cout << "soc=" << estimate.state_of_charge << " range_km=" << estimate.estimated_range_km << '\n';

    DifferentialOdometry odometry;
    odometry.update({0, 0});
    const auto pose = odometry.update({1024, 1024});
    std::cout << "distance_m=" << pose.distance_m << " x_m=" << pose.x_m << '\n';

    DiagnosticMonitor monitor;
    Telemetry telemetry;
    telemetry.safety_state = SafetyState::Ready;
    telemetry.battery_percent = 82;
    telemetry.nearest_obstacle_cm = 110;
    monitor.observe(telemetry);
    telemetry.safety_state = SafetyState::Degraded;
    telemetry.requested_speed_scale = 0.8;
    telemetry.safety_speed_scale = 0.5;
    telemetry.nearest_obstacle_cm = 55;
    monitor.observe(telemetry);
    std::cout << monitor.to_json() << '\n';
    return 0;
}
