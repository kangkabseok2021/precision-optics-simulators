#include "PressureSensor.h"

PressureSensor::PressureSensor(double noise_stddev_n)
    : noise_(0.0, noise_stddev_n) {}

bool PressureSensor::init() {
    initialised_ = true;
    return true;
}

bool PressureSensor::selfTest() {
    return initialised_;
}

double PressureSensor::readForce() const {
    return base_force_ + noise_(rng_);
}

void PressureSensor::shutdown() {
    base_force_  = 0.0;
    initialised_ = false;
}
