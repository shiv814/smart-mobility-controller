#include "energy.hpp"

#include <algorithm>
#include <stdexcept>

namespace mobility {

EnergyEstimator::EnergyEstimator(BatteryConfig config) : config_(config) {
    if (config_.capacity_wh <= 0 || config_.nominal_wh_per_km <= 0 || config_.full_voltage <= config_.empty_voltage
        || config_.reserve_fraction < 0 || config_.reserve_fraction >= 1) {
        throw std::invalid_argument("invalid battery configuration");
    }
}

EnergyEstimate EnergyEstimator::estimate(double voltage, double recent_wh_per_km, double speed_kph) const {
    const double soc = std::clamp(
        (voltage - config_.empty_voltage) / (config_.full_voltage - config_.empty_voltage), 0.0, 1.0
    );
    const bool reserve = soc <= config_.reserve_fraction;
    const double usable_fraction = std::max(0.0, soc - config_.reserve_fraction);
    const double usable = config_.capacity_wh * usable_fraction;
    const double consumption = recent_wh_per_km > 0 ? recent_wh_per_km : config_.nominal_wh_per_km;
    const double range = usable / consumption;
    const double minutes = speed_kph > 0 ? (range / speed_kph) * 60.0 : 0.0;
    return {soc, usable, range, minutes, reserve};
}

}  // namespace mobility
