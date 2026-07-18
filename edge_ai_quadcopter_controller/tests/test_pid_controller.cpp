#include "edge_ai_quadcopter/pid_controller.h"

#include <cmath>

#include <gtest/gtest.h>

using edge_ai_quadcopter::PidController;

namespace {

constexpr float kDt = 0.01f;

TEST(PidController, ZeroErrorGivesZeroOutput) {
  PidController pid(1.0f, 1.0f, 1.0f, 10.0f);
  EXPECT_FLOAT_EQ(pid.update(0.0f, kDt), 0.0f);
}

TEST(PidController, ProportionalTermOnly) {
  PidController pid(2.0f, 0.0f, 0.0f, 10.0f);
  // With Ki = Kd = 0, output on the first tick is exactly Kp * error.
  EXPECT_FLOAT_EQ(pid.update(3.0f, kDt), 6.0f);
}

TEST(PidController, IntegralAccumulation) {
  constexpr float kKi = 0.5f;
  PidController pid(0.0f, kKi, 0.0f, 100.0f);
  constexpr float kError = 2.0f;
  constexpr int kSteps = 10;
  float output = 0.0f;
  for (int i = 0; i < kSteps; ++i) {
    output = pid.update(kError, kDt);
  }
  // integral ~= N * error * dt (Kd term is zero after the first step
  // since error doesn't change), so output ~= Ki * N * error * dt.
  const float expected = kKi * kSteps * kError * kDt;
  EXPECT_NEAR(output, expected, 1e-4f);
}

TEST(PidController, IntegralWindupClamp) {
  constexpr float kIlim = 1.0f;
  PidController pid(0.0f, 1.0f, 0.0f, kIlim);
  for (int i = 0; i < 1000; ++i) {
    pid.update(100.0f, kDt);
  }
  EXPECT_LE(std::abs(pid.integral()), kIlim);
}

TEST(PidController, DerivativeOnErrorChange) {
  constexpr float kKd = 0.5f;
  PidController pid(0.0f, 0.0f, kKd, 10.0f);
  pid.update(1.0f, kDt);
  // Second call: error jumps from 1.0 to 3.0 -> de/dt = (3-1)/dt.
  const float output = pid.update(3.0f, kDt);
  const float expected = kKd * (3.0f - 1.0f) / kDt;
  EXPECT_NEAR(output, expected, 1e-3f);
}

TEST(PidController, ResetClearsState) {
  PidController pid(1.0f, 1.0f, 1.0f, 10.0f);
  pid.update(5.0f, kDt);
  pid.update(5.0f, kDt);
  pid.reset();
  EXPECT_FLOAT_EQ(pid.integral(), 0.0f);
  // With state cleared, a lone update(0, dt) must return exactly zero
  // (proportional and derivative terms both vanish on zero error).
  EXPECT_FLOAT_EQ(pid.update(0.0f, kDt), 0.0f);
}

}  // namespace
