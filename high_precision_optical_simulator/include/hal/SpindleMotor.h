#pragma once
#include "IDevice.h"

// Simulated spindle with linear ramp-up/ramp-down curve.
class SpindleMotor : public IDevice {
public:
    explicit SpindleMotor(double max_rpm = 3000.0, double ramp_rate_rpm_per_s = 500.0);

    bool  init()     override;
    bool  selfTest() override;
    void  shutdown() override;
    const char* name() const override { return "SpindleMotor"; }

    void   setTargetRpm(double rpm);
    double currentRpm() const { return current_rpm_; }

    // Advance simulation by dt_s seconds.
    void step(double dt_s);

private:
    double max_rpm_;
    double ramp_rate_;
    double current_rpm_{0.0};
    double target_rpm_ {0.0};
    bool   initialised_{false};
};
