#pragma once

#include "controller.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mobility {

enum class CommandSource { User, Remote, Navigation, Safety };

std::string to_string(CommandSource source);

struct CommandRequest {
    CommandSource source{CommandSource::User};
    Command command{Command::Stop};
    int priority{0};
    std::uint32_t issued_ms{0};
    std::uint32_t ttl_ms{500};
};

struct ArbitrationDecision {
    Command command{Command::Stop};
    CommandSource source{CommandSource::Safety};
    int priority{0};
    bool fallback_stop{true};
    std::size_t active_requests{0};
};

class CommandArbiter {
public:
    explicit CommandArbiter(std::size_t max_requests = 16) : max_requests_(max_requests) {}
    void submit(CommandRequest request);
    ArbitrationDecision decide(std::uint32_t now_ms, bool emergency_stop = false);
    void clear();
    std::size_t pending() const { return requests_.size(); }

private:
    static bool expired(const CommandRequest& request, std::uint32_t now_ms);
    std::size_t max_requests_;
    std::vector<CommandRequest> requests_;
};

}  // namespace mobility
