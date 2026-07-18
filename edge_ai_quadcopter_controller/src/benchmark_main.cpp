// Google Benchmark harness for the two hot-path stages of the 250 Hz
// control loop. scripts/benchmark.sh sums the "PID+Mixer" and
// "TFLM Inference" median times and fails the build if the combined
// per-tick cost exceeds the 500 us real-time deadline.
#include <array>

#include <benchmark/benchmark.h>

#include "edge_ai_quadcopter/attitude_pid.h"
#include "edge_ai_quadcopter/inference_engine.h"
#include "edge_ai_quadcopter/motor_mixer.h"

using edge_ai_quadcopter::AttitudePid;
using edge_ai_quadcopter::InferenceEngine;
using edge_ai_quadcopter::MotorMixer;

static void BM_PidMixer(benchmark::State& state) {
  AttitudePid attitude_pid;
  constexpr float kDt = 1.0f / 250.0f;
  for (auto _ : state) {
    const auto pid_out = attitude_pid.update(0.02f, -0.01f, 0.0f, kDt);
    const auto motors =
        MotorMixer::mix(0.55f, pid_out.pitch, pid_out.roll, pid_out.yaw);
    benchmark::DoNotOptimize(motors);
  }
}
BENCHMARK(BM_PidMixer)->Name("PID+Mixer");

static void BM_TflmInference(benchmark::State& state) {
  InferenceEngine engine;
  engine.init();
  std::array<float, 16> lidar;
  lidar.fill(3.0f);
  for (auto _ : state) {
    const float prob = engine.run(lidar.data(), lidar.size());
    benchmark::DoNotOptimize(prob);
  }
}
BENCHMARK(BM_TflmInference)->Name("TFLM Inference");

BENCHMARK_MAIN();
