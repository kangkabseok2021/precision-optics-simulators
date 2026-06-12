#include "SpindleMotor.h"
#include <algorithm>
#include <cmath>

SpindleMotor::SpindleMotor(double max_rpm, double ramp_rate_rpm_per_s)
    : max_rpm_(max_rpm), ramp_rate_(ramp_rate_rpm_per_s) {}

bool SpindleMotor::init() {
    current_rpm_ = 0.0;
    target_rpm_  = 0.0;
    initialised_ = true;
    return true;
}

bool SpindleMotor::selfTest() {
    return initialised_;
}

void SpindleMotor::setTargetRpm(double rpm) {
    target_rpm_ = std::max(0.0, std::min(rpm, max_rpm_));
}

void SpindleMotor::step(double dt_s) {
    double delta     = target_rpm_ - current_rpm_;
    double max_delta = ramp_rate_ * dt_s;
    if (std::abs(delta) <= max_delta)
        current_rpm_ = target_rpm_;
    else
        current_rpm_ += std::copysign(max_delta, delta);
}

void SpindleMotor::shutdown() {
    target_rpm_  = 0.0;
    current_rpm_ = 0.0;
    initialised_ = false;
}
