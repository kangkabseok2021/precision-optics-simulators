#include "ecu_state_machine.h"

#define FAULT_OVER_TEMP          0x00000001
#define FAULT_LOW_OIL_PRESSURE   0x00000002
#define FAULT_LOW_BATTERY        0x00000004
#define FAULT_BUS_OFF            0x00000008

EcuStateMachine::EcuStateMachine() 
    : state_(EcuState::INIT), fault_bitmap_(0), diag_timer_(0.0), bus_off_(false) {
}

EcuState EcuStateMachine::get_state() const {
    return state_;
}

uint32_t EcuStateMachine::get_fault_bitmap() const {
    return fault_bitmap_;
}

double EcuStateMachine::get_telemetry_rate() const {
    return (state_ == EcuState::DIAGNOSTIC) ? 100.0 : 50.0;
}

PhysicsModel& EcuStateMachine::get_physics() {
    return physics_;
}

const PhysicsModel& EcuStateMachine::get_physics() const {
    return physics_;
}

void EcuStateMachine::set_bus_off(bool bus_off) {
    bus_off_ = bus_off;
    if (bus_off_) {
        fault_bitmap_ |= FAULT_BUS_OFF;
        state_ = EcuState::FAULT;
    } else {
        fault_bitmap_ &= ~FAULT_BUS_OFF;
    }
}

void EcuStateMachine::trigger_bus_recovery() {
    bus_off_ = false;
    fault_bitmap_ &= ~FAULT_BUS_OFF;
}

void EcuStateMachine::evaluate_faults() {
    const auto& s = physics_.get_state();
    
    // Evaluate Over-Temp
    if (s.coolant_temp > 110.0) {
        fault_bitmap_ |= FAULT_OVER_TEMP;
    } else if (s.coolant_temp < 105.0) {
        fault_bitmap_ &= ~FAULT_OVER_TEMP;
    }

    // Evaluate Low Oil Pressure (only when engine running / RPM > 800)
    if (s.rpm > 800.0 && s.oil_pressure < 150.0) {
        fault_bitmap_ |= FAULT_LOW_OIL_PRESSURE;
    } else if (s.rpm <= 800.0 || s.oil_pressure > 180.0) {
        fault_bitmap_ &= ~FAULT_LOW_OIL_PRESSURE;
    }

    // Evaluate Low Battery
    if (s.battery_voltage < 11.0) {
        fault_bitmap_ |= FAULT_LOW_BATTERY;
    } else if (s.battery_voltage > 11.5) {
        fault_bitmap_ &= ~FAULT_LOW_BATTERY;
    }

    if (bus_off_) {
        fault_bitmap_ |= FAULT_BUS_OFF;
    }
}

void EcuStateMachine::update(double dt) {
    if (state_ == EcuState::INIT) {
        state_ = EcuState::NORMAL;
        return;
    }

    physics_.step(dt);
    evaluate_faults();

    if (state_ != EcuState::FAULT) {
        if (fault_bitmap_ != 0) {
            state_ = EcuState::FAULT;
        }
    }

    if (state_ == EcuState::DIAGNOSTIC) {
        diag_timer_ += dt;
        if (diag_timer_ >= 5.0) {
            diag_timer_ = 0.0;
            state_ = EcuState::NORMAL;
        }
    }
}

uint8_t EcuStateMachine::handle_command(const CommandFrame& cmd) {
    switch (cmd.command_id) {
        case 0x01: { // SET_MODE
            uint8_t target_mode = cmd.param_u8;
            if (target_mode == 0) {
                state_ = EcuState::INIT;
                return 0;
            } else if (target_mode == 1) {
                if (state_ == EcuState::FAULT && fault_bitmap_ != 0) {
                    return 2;
                }
                state_ = EcuState::NORMAL;
                return 0;
            } else if (target_mode == 2) {
                if (state_ == EcuState::FAULT) {
                    return 2;
                }
                state_ = EcuState::DIAGNOSTIC;
                diag_timer_ = 0.0;
                return 0;
            } else if (target_mode == 3) {
                state_ = EcuState::FAULT;
                return 0;
            }
            return 1;
        }
        case 0x02: { // SET_THROTTLE
            if (cmd.param_u8 > 100) {
                return 1;
            }
            physics_.set_throttle(static_cast<double>(cmd.param_u8));
            return 0;
        }
        case 0x03: { // REQUEST_DIAG
            if (state_ == EcuState::FAULT) {
                return 2;
            }
            state_ = EcuState::DIAGNOSTIC;
            diag_timer_ = 0.0;
            return 0;
        }
        case 0x04: { // CLEAR_FAULTS
            if (state_ == EcuState::FAULT) {
                trigger_bus_recovery();
                evaluate_faults();
                if (fault_bitmap_ == 0) {
                    state_ = EcuState::NORMAL;
                    return 0;
                } else {
                    return 2;
                }
            }
            return 0;
        }
        default:
            return 1;
    }
}
