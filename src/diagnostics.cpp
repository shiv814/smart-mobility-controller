#include "diagnostics.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace mobility {

void DiagnosticMonitor::observe(const Telemetry& telemetry) {
    ++kpis_.frames;
    switch (telemetry.safety_state) {
        case SafetyState::Ready: ++kpis_.ready_frames; break;
        case SafetyState::Degraded: ++kpis_.degraded_frames; break;
        case SafetyState::Stopped: ++kpis_.stopped_frames; break;
        case SafetyState::Fault: ++kpis_.fault_frames; break;
    }
    if (telemetry.timed_out) ++kpis_.timeout_frames;
    if (telemetry.fault != FaultCode::None) ++kpis_.fault_counts[telemetry.fault];
    if (telemetry.requested_speed_scale > 0.0
        && telemetry.requested_speed_scale > telemetry.safety_speed_scale
        && telemetry.nearest_obstacle_cm > 0.0) {
        ++kpis_.obstacle_interventions;
    }
    if (telemetry.nearest_obstacle_cm > 0.0) {
        if (!have_obstacle_ || telemetry.nearest_obstacle_cm < kpis_.minimum_obstacle_cm) {
            kpis_.minimum_obstacle_cm = telemetry.nearest_obstacle_cm;
        }
        have_obstacle_ = true;
    }
    kpis_.minimum_battery_percent = std::min(kpis_.minimum_battery_percent, telemetry.battery_percent);
    kpis_.safety_availability = kpis_.frames == 0 ? 1.0
        : static_cast<double>(kpis_.ready_frames + kpis_.degraded_frames) / static_cast<double>(kpis_.frames);
}

std::string DiagnosticMonitor::to_json() const {
    std::ostringstream out;
    out << std::fixed << std::setprecision(4)
        << "{\"frames\":" << kpis_.frames
        << ",\"ready\":" << kpis_.ready_frames
        << ",\"degraded\":" << kpis_.degraded_frames
        << ",\"stopped\":" << kpis_.stopped_frames
        << ",\"fault\":" << kpis_.fault_frames
        << ",\"timeouts\":" << kpis_.timeout_frames
        << ",\"obstacle_interventions\":" << kpis_.obstacle_interventions
        << ",\"minimum_obstacle_cm\":" << kpis_.minimum_obstacle_cm
        << ",\"minimum_battery_percent\":" << kpis_.minimum_battery_percent
        << ",\"safety_availability\":" << kpis_.safety_availability << "}";
    return out.str();
}

void DiagnosticMonitor::reset() {
    kpis_ = {};
    have_obstacle_ = false;
}

}  // namespace mobility
