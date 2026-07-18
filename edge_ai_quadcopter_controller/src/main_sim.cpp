// Host-side simulation entry point: runs the attitude controller and
// obstacle-detection inference over a scripted flight scenario and
// prints per-tick status. Not part of the test suite — a runnable demo
// of the same code path exercised by the ARM cross-compile target.
#include <array>
#include <cstdio>

#include "edge_ai_quadcopter/attitude_pid.h"
#include "edge_ai_quadcopter/inference_engine.h"
#include "edge_ai_quadcopter/motor_mixer.h"

int main() {
  using edge_ai_quadcopter::AttitudePid;
  using edge_ai_quadcopter::InferenceEngine;
  using edge_ai_quadcopter::MotorMixer;

  static AttitudePid attitude_pid;
  static InferenceEngine inference_engine;
  inference_engine.init();

  constexpr float kDt = 1.0f / 250.0f;  // 250 Hz control loop
  constexpr float kThrottle = 0.55f;

  std::array<float, 16> lidar_far;
  lidar_far.fill(6.0f);
  std::array<float, 16> lidar_obstacle = lidar_far;
  for (int i = 4; i < 8; ++i) lidar_obstacle[i] = 0.5f;

  for (int tick = 0; tick < 5; ++tick) {
    const bool obstacle_tick = (tick == 3);
    const auto& lidar = obstacle_tick ? lidar_obstacle : lidar_far;

    const float obstacle_prob = inference_engine.run(lidar.data(), lidar.size());
    const float throttle = (obstacle_prob > 0.8f) ? kThrottle * 0.5f : kThrottle;

    const auto pid_out = attitude_pid.update(/*pitch_error=*/0.02f,
                                              /*roll_error=*/-0.01f,
                                              /*yaw_error=*/0.0f, kDt);
    const auto motors = MotorMixer::mix(throttle, pid_out.pitch, pid_out.roll,
                                         pid_out.yaw);

    std::printf(
        "tick=%d obstacle_prob=%.3f throttle=%.3f motors=[%.3f %.3f %.3f "
        "%.3f]\n",
        tick, obstacle_prob, throttle, motors.fl, motors.fr, motors.br,
        motors.bl);
  }

  return 0;
}
