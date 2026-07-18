// Cortex-M7 cross-compile smoke target: forces the compiler to
// instantiate PidController/AttitudePid/MotorMixer/InferenceEngine
// under the ARM toolchain's hard-float ABI so arm-none-eabi-size
// reports the real .text/.data/.bss footprint of the algorithmic core.
// A static library, not a linked executable — no vector table, linker
// script, or startup code is needed just to prove the code
// cross-compiles for the target CPU.
#include <array>

#include "edge_ai_quadcopter/attitude_pid.h"
#include "edge_ai_quadcopter/inference_engine.h"
#include "edge_ai_quadcopter/motor_mixer.h"

namespace edge_ai_quadcopter::arm_smoke {

static AttitudePid g_attitude_pid;
static InferenceEngine g_inference_engine;

void RunOneControlTick(const std::array<float, 16>& lidar_beams,
                       float pitch_error, float roll_error, float yaw_error,
                       float dt, float throttle,
                       MotorOutputs* out) noexcept {
  g_inference_engine.init();
  const float obstacle_prob =
      g_inference_engine.run(lidar_beams.data(), lidar_beams.size());
  const float safe_throttle =
      (obstacle_prob > 0.8f) ? throttle * 0.5f : throttle;

  const auto pid_out =
      g_attitude_pid.update(pitch_error, roll_error, yaw_error, dt);
  *out = MotorMixer::mix(safe_throttle, pid_out.pitch, pid_out.roll,
                         pid_out.yaw);
}

}  // namespace edge_ai_quadcopter::arm_smoke
