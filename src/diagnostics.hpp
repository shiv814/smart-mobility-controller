#pragma once

#include "controller.hpp"

#include <cstddef>
#include <map>
#include <string>

namespace mobility {

struct SafetyKpis {
    std::size_t frames{0};
    std::size_t ready_frames{0};
    std::size_t degraded_frames{0};
    std::size_t stopped_frames{0};
    std::size_t fault_frames{0};
    std::size_t timeout_frames{0};
    std::size_t obstacle_interventions{0};
    double minimum_obstacle_cm{0.0};
    double minimum_battery_percent{100.0};
    double safety_availability{1.0};
    std::map<FaultCode, std::size_t> fault_counts;
};

class DiagnosticMonitor {
public:
    void observe(const Telemetry& telemetry);
    const SafetyKpis& kpis() const { return kpis_; }
    std::string to_json() const;
    void reset();

private:
    SafetyKpis kpis_{};
    bool have_obstacle_{false};
};

}  // namespace mobility
