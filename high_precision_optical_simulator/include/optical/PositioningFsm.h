#pragma once
#include "../hal/IDevice.h"
#include <string>
#include <vector>

enum class PosState {
    IDLE,
    INITIALISING,
    CALIBRATING,
    READY,
    ACTIVE,
    FAULT
};

const char* posStateName(PosState s) noexcept;

// Six-state manufacturing FSM mirroring PLC lifecycle logic.
// Devices are driven through init() / selfTest() / shutdown() on transitions.
// All state mutations are single-threaded — caller serialises access.
class PositioningFsm {
public:
    explicit PositioningFsm(std::vector<IDevice*> devices);

    PosState    state()     const noexcept { return state_; }
    std::string lastError() const          { return last_error_; }

    // IDLE → INITIALISING → (calls init() on all devices)
    // Returns false and stays in INITIALISING if any device fails init().
    bool start();

    // INITIALISING → CALIBRATING → READY (calls selfTest() on all devices)
    // Returns false and records failing device name if selfTest() fails.
    bool calibrate();

    // READY → ACTIVE
    void activate();

    // ACTIVE → READY
    void pause();

    // Any state → FAULT; calls shutdown() on all devices.
    void fault(const std::string& reason);

    // FAULT → IDLE; calls shutdown() then re-evaluates all devices.
    // Returns false if any device reports shutdown issues.
    bool reset();

private:
    std::vector<IDevice*> devices_;
    PosState              state_{PosState::IDLE};
    std::string           last_error_;
};
