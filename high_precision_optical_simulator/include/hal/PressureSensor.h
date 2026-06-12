#pragma once
#include "IDevice.h"
#include <random>

// Contact-force sensor with Gaussian noise model.
class PressureSensor : public IDevice {
public:
    explicit PressureSensor(double noise_stddev_n = 0.05);

    bool  init()     override;
    bool  selfTest() override;
    void  shutdown() override;
    const char* name() const override { return "PressureSensor"; }

    void   setBaseForce(double f_n) { base_force_ = f_n; }
    double readForce() const;   // N, including Gaussian noise

private:
    double base_force_{0.0};
    mutable std::mt19937                     rng_{42};
    mutable std::normal_distribution<double> noise_;
    bool initialised_{false};
};
