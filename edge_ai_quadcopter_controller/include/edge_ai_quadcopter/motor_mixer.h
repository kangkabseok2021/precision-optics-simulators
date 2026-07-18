#pragma once

#include <algorithm>

namespace edge_ai_quadcopter {

struct MotorOutputs {
  float fl;
  float fr;
  float br;
  float bl;
};

// X-frame motor mixer: maps throttle/pitch/roll/yaw commands to four
// motor thrust outputs, each clamped to [0, 1]. Pure function, no
// side effects, identical behavior on host and target.
class MotorMixer {
 public:
  // Positive pitch commands a nose-down (forward-accelerating) tilt, so
  // it increases rear-motor thrust and decreases front-motor thrust.
  static MotorOutputs mix(float throttle, float pitch, float roll,
                           float yaw) noexcept {
    return {
        std::clamp(throttle - pitch - roll + yaw, 0.0f, 1.0f),
        std::clamp(throttle - pitch + roll - yaw, 0.0f, 1.0f),
        std::clamp(throttle + pitch + roll + yaw, 0.0f, 1.0f),
        std::clamp(throttle + pitch - roll - yaw, 0.0f, 1.0f),
    };
  }
};

}  // namespace edge_ai_quadcopter
