#ifndef PHYSICS_MODEL_H
#define PHYSICS_MODEL_H

struct PhysicsState {
    double throttle;         // 0 - 100 %
    double rpm;              // RPM
    double coolant_temp;     // °C
    double oil_pressure;     // kPa
    double battery_voltage;  // V
    double fuel_level;       // %
    double vehicle_speed;    // km/h
    double ambient_temp;     // °C
};

class PhysicsModel {
public:
    PhysicsModel();
    void set_throttle(double throttle);
    void step(double dt);
    const PhysicsState& get_state() const;
    void reset();

private:
    PhysicsState state_;
};

#endif // PHYSICS_MODEL_H
