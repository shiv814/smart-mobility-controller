#include "arbitration.hpp"

#include <algorithm>
#include <stdexcept>

namespace mobility {

std::string to_string(CommandSource source) {
    switch (source) {
        case CommandSource::User: return "user";
        case CommandSource::Remote: return "remote";
        case CommandSource::Navigation: return "navigation";
        case CommandSource::Safety: return "safety";
    }
    return "unknown";
}

bool CommandArbiter::expired(const CommandRequest& request, std::uint32_t now_ms) {
    return now_ms - request.issued_ms > request.ttl_ms;
}

void CommandArbiter::submit(CommandRequest request) {
    if (request.ttl_ms == 0) throw std::invalid_argument("ttl_ms must be positive");
    if (max_requests_ == 0) throw std::logic_error("arbiter capacity must be positive");
    if (requests_.size() >= max_requests_) requests_.erase(requests_.begin());
    requests_.push_back(request);
}

ArbitrationDecision CommandArbiter::decide(std::uint32_t now_ms, bool emergency_stop) {
    requests_.erase(
        std::remove_if(requests_.begin(), requests_.end(), [now_ms](const auto& request) {
            return expired(request, now_ms);
        }),
        requests_.end()
    );
    if (emergency_stop) return {Command::Stop, CommandSource::Safety, 1000, false, requests_.size()};
    if (requests_.empty()) return {Command::Stop, CommandSource::Safety, 0, true, 0};

    const auto best = std::max_element(requests_.begin(), requests_.end(), [](const auto& left, const auto& right) {
        if (left.priority != right.priority) return left.priority < right.priority;
        return left.issued_ms < right.issued_ms;
    });
    return {best->command, best->source, best->priority, false, requests_.size()};
}

void CommandArbiter::clear() { requests_.clear(); }

}  // namespace mobility
