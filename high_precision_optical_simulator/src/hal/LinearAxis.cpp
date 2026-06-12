#include "LinearAxis.h"

LinearAxis::LinearAxis(const char* axis_name, double backlash_um)
    : name_(axis_name), backlash_um_(backlash_um)
{
    (void)backlash_um_;  // stored for real-driver compensation; not modelled in simulation
}

bool LinearAxis::init() {
    position_    = 0.0;
    target_      = 0.0;
    initialised_ = true;
    return true;
}

bool LinearAxis::selfTest() {
    return initialised_;
}

const char* LinearAxis::name() const { return name_.c_str(); }

void LinearAxis::setTarget(double pos_um) { target_ = pos_um; }

// Instant positioning — in a real axis this would be a servo loop.
// backlash_um_ represents mechanical play; a real driver compensates at reversal.
void LinearAxis::step(double /*dt_s*/) { position_ = target_; }

void LinearAxis::shutdown() {
    target_      = 0.0;
    position_    = 0.0;
    initialised_ = false;
}
