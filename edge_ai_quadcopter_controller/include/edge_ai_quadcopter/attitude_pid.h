#pragma once

#include "edge_ai_quadcopter/pid_controller.h"

namespace edge_ai_quadcopter {

// Holds the three axis PID controllers as direct value members — the
// whole controller fits in 3 * sizeof(PidController) = 72 bytes of
// static/BSS storage, with no virtual dispatch and no RTTI.
class AttitudePid {
 public:
  struct Output {
    float pitch;
    float roll;
    float yaw;
  };

  constexpr AttitudePid() noexcept
      : pitch_(1.5f, 0.1f, 0.05f, kIntegralLimit),
        roll_(1.5f, 0.1f, 0.05f, kIntegralLimit),
        yaw_(2.0f, 0.05f, 0.1f, kIntegralLimit) {}

  Output update(float pitch_error, float roll_error, float yaw_error,
                float dt) noexcept {
    return {pitch_.update(pitch_error, dt), roll_.update(roll_error, dt),
            yaw_.update(yaw_error, dt)};
  }

  void reset() noexcept {
    pitch_.reset();
    roll_.reset();
    yaw_.reset();
  }

  const PidController& pitch() const noexcept { return pitch_; }
  const PidController& roll() const noexcept { return roll_; }
  const PidController& yaw() const noexcept { return yaw_; }

 private:
  static constexpr float kIntegralLimit = 10.0f;

  PidController pitch_;
  PidController roll_;
  PidController yaw_;
};

}  // namespace edge_ai_quadcopter
