#pragma once
#include "IDevice.h"
#include <string>

// Simulated linear axis with instant positioning.
// backlash_um is the mechanical play constant (documented; modelled as a positional offset).
class LinearAxis : public IDevice {
public:
    explicit LinearAxis(const char* axis_name, double backlash_um = 0.3);

    bool  init()     override;
    bool  selfTest() override;
    void  shutdown() override;
    const char* name() const override;

    void   setTarget(double pos_um);
    double position()    const { return position_; }
    void   step(double dt_s);

private:
    std::string name_;
    double backlash_um_;  // mechanical play (mm); available for real-driver compensation
    double position_{0.0};
    double target_  {0.0};
    bool   initialised_{false};
};
