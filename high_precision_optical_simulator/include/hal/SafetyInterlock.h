#pragma once
#include "IDevice.h"
#include <chrono>

// Hardware watchdog — trips if ping() is not called within max_interval_ms.
class SafetyInterlock : public IDevice {
public:
    explicit SafetyInterlock(int max_interval_ms = 50);

    bool  init()     override;
    bool  selfTest() override;
    void  shutdown() override;
    const char* name() const override { return "SafetyInterlock"; }

    void ping();
    bool isTripped() const;

private:
    int max_interval_ms_;
    std::chrono::steady_clock::time_point last_ping_;
    bool initialised_{false};
};
