#pragma once

namespace mobility {

struct BatteryConfig {
    double capacity_wh{480.0};
    double reserve_fraction{0.15};
    double empty_voltage{20.0};
    double full_voltage{25.2};
    double nominal_wh_per_km{35.0};
};

struct EnergyEstimate {
    double state_of_charge{0.0};
    double usable_wh{0.0};
    double estimated_range_km{0.0};
    double estimated_minutes{0.0};
    bool reserve_active{false};
};

class EnergyEstimator {
public:
    explicit EnergyEstimator(BatteryConfig config = {});
    EnergyEstimate estimate(double battery_voltage, double recent_wh_per_km = 0.0, double speed_kph = 0.0) const;
    const BatteryConfig& config() const { return config_; }

private:
    BatteryConfig config_;
};

}  // namespace mobility
