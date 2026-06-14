#include "physics_model.h"
#include <algorithm>

PhysicsModel::PhysicsModel() {
    reset();
}

void PhysicsModel::set_throttle(double throttle) {
    state_.throttle = std::clamp(throttle, 0.0, 100.0);
}

void PhysicsModel::step(double dt) {
    // 1st order lag for RPM: target = 800 + throttle * 52
    double target_rpm = 800.0 + (state_.throttle * 52.0);
    // time constant = 1.0s
    state_.rpm += (target_rpm - state_.rpm) * (dt / 1.0);

    // 1st order lag for coolant temp: target = 25 + (state_.rpm / 80.0)
    double target_temp = 25.0 + (state_.rpm / 80.0);
    // time constant = 10.0s (heats slowly)
    state_.coolant_temp += (target_temp - state_.coolant_temp) * (dt / 10.0);

    // Oil pressure proportional to RPM: target = 150 + RPM * 0.1
    double target_oil = 150.0 + (state_.rpm * 0.08);
    // time constant = 0.5s
    state_.oil_pressure += (target_oil - state_.oil_pressure) * (dt / 0.5);

    // Battery voltage: fluctuates slightly around 13.8V based on RPM, 11.5V when idle
    double target_volt = (state_.rpm > 400.0) ? 13.8 : 11.5;
    state_.battery_voltage += (target_volt - state_.battery_voltage) * (dt / 2.0);

    // Vehicle speed: target = throttle * 1.8 km/h
    double target_speed = state_.throttle * 1.8;
    // time constant = 3.0s
    state_.vehicle_speed += (target_speed - state_.vehicle_speed) * (dt / 3.0);

    // Fuel level decreases slowly
    state_.fuel_level -= (0.0001 + 0.0005 * (state_.throttle / 100.0)) * dt;
    state_.fuel_level = std::max(0.0, state_.fuel_level);
}

const PhysicsState& PhysicsModel::get_state() const {
    return state_;
}

void PhysicsModel::reset() {
    state_.throttle = 0.0;
    state_.rpm = 800.0;
    state_.coolant_temp = 25.0;
    state_.oil_pressure = 214.0;
    state_.battery_voltage = 12.6;
    state_.fuel_level = 100.0;
    state_.vehicle_speed = 0.0;
    state_.ambient_temp = 22.5;
}
