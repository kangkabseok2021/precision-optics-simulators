#ifndef ECU_STATE_MACHINE_H
#define ECU_STATE_MACHINE_H

#include "physics_model.h"
#include "can_fd_frames.h"

enum class EcuState {
    INIT,
    NORMAL,
    DIAGNOSTIC,
    FAULT
};

class EcuStateMachine {
public:
    EcuStateMachine();
    void update(double dt);
    uint8_t handle_command(const CommandFrame& cmd);
    
    EcuState get_state() const;
    uint32_t get_fault_bitmap() const;
    double get_telemetry_rate() const; // 50.0 Hz or 100.0 Hz
    
    void set_bus_off(bool bus_off);
    void trigger_bus_recovery();

    PhysicsModel& get_physics();
    const PhysicsModel& get_physics() const;

private:
    void evaluate_faults();
    
    EcuState state_;
    PhysicsModel physics_;
    uint32_t fault_bitmap_;
    double diag_timer_;
    bool bus_off_;
};

#endif // ECU_STATE_MACHINE_H
