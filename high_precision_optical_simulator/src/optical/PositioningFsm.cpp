#include "PositioningFsm.h"

const char* posStateName(PosState s) noexcept {
    switch (s) {
        case PosState::IDLE:         return "IDLE";
        case PosState::INITIALISING: return "INITIALISING";
        case PosState::CALIBRATING:  return "CALIBRATING";
        case PosState::READY:        return "READY";
        case PosState::ACTIVE:       return "ACTIVE";
        case PosState::FAULT:        return "FAULT";
    }
    return "UNKNOWN";
}

PositioningFsm::PositioningFsm(std::vector<IDevice*> devices)
    : devices_(std::move(devices)) {}

bool PositioningFsm::start() {
    if (state_ != PosState::IDLE) return false;
    state_ = PosState::INITIALISING;

    for (auto* dev : devices_) {
        if (!dev->init()) {
            last_error_ = std::string("init() failed: ") + dev->name();
            fault(last_error_);
            return false;
        }
    }
    return true;
}

bool PositioningFsm::calibrate() {
    if (state_ != PosState::INITIALISING) return false;
    state_ = PosState::CALIBRATING;

    for (auto* dev : devices_) {
        if (!dev->selfTest()) {
            last_error_ = std::string("selfTest() failed: ") + dev->name();
            fault(last_error_);
            return false;
        }
    }
    state_ = PosState::READY;
    return true;
}

void PositioningFsm::activate() {
    if (state_ == PosState::READY)
        state_ = PosState::ACTIVE;
}

void PositioningFsm::pause() {
    if (state_ == PosState::ACTIVE)
        state_ = PosState::READY;
}

void PositioningFsm::fault(const std::string& reason) {
    last_error_ = reason;
    state_      = PosState::FAULT;
    for (auto* dev : devices_)
        dev->shutdown();
}

bool PositioningFsm::reset() {
    if (state_ != PosState::FAULT) return false;
    last_error_.clear();
    state_ = PosState::IDLE;
    return true;
}
