#include "SafetyInterlock.h"

SafetyInterlock::SafetyInterlock(int max_interval_ms)
    : max_interval_ms_(max_interval_ms) {}

bool SafetyInterlock::init() {
    last_ping_   = std::chrono::steady_clock::now();
    initialised_ = true;
    return true;
}

bool SafetyInterlock::selfTest() {
    return initialised_;
}

void SafetyInterlock::ping() {
    last_ping_ = std::chrono::steady_clock::now();
}

bool SafetyInterlock::isTripped() const {
    auto elapsed = std::chrono::steady_clock::now() - last_ping_;
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    return elapsed_ms > max_interval_ms_;
}

void SafetyInterlock::shutdown() {
    initialised_ = false;
}
