#include "edge_ai_quadcopter/motor_mixer.h"

#include <gtest/gtest.h>

using edge_ai_quadcopter::MotorMixer;

namespace {

TEST(MotorMixer, HoverEquality) {
  const auto out = MotorMixer::mix(0.5f, 0.0f, 0.0f, 0.0f);
  EXPECT_FLOAT_EQ(out.fl, 0.5f);
  EXPECT_FLOAT_EQ(out.fr, 0.5f);
  EXPECT_FLOAT_EQ(out.br, 0.5f);
  EXPECT_FLOAT_EQ(out.bl, 0.5f);
}

TEST(MotorMixer, PitchGeometry) {
  // Positive pitch command -> rear motors get more thrust than front.
  const auto out = MotorMixer::mix(0.5f, 0.2f, 0.0f, 0.0f);
  EXPECT_GT(out.br, out.fl);
  EXPECT_GT(out.bl, out.fr);
}

TEST(MotorMixer, SaturationBounds) {
  const auto out = MotorMixer::mix(0.9f, 0.5f, 0.5f, 0.5f);
  for (float v : {out.fl, out.fr, out.br, out.bl}) {
    EXPECT_GE(v, 0.0f);
    EXPECT_LE(v, 1.0f);
  }
  const auto low = MotorMixer::mix(0.1f, -0.5f, -0.5f, -0.5f);
  for (float v : {low.fl, low.fr, low.br, low.bl}) {
    EXPECT_GE(v, 0.0f);
    EXPECT_LE(v, 1.0f);
  }
}

TEST(MotorMixer, YawDiagonalPairing) {
  // FL and BR share +yaw gain; FR and BL share -yaw gain (diagonal pairs
  // spin opposite directions on an X-frame, so a positive yaw command
  // increases thrust on one diagonal and decreases it on the other).
  constexpr float kThrottle = 0.5f;
  constexpr float kYaw = 0.2f;
  const auto zero = MotorMixer::mix(kThrottle, 0.0f, 0.0f, 0.0f);
  const auto yawed = MotorMixer::mix(kThrottle, 0.0f, 0.0f, kYaw);

  const float fl_delta = yawed.fl - zero.fl;
  const float br_delta = yawed.br - zero.br;
  const float fr_delta = yawed.fr - zero.fr;
  const float bl_delta = yawed.bl - zero.bl;

  EXPECT_FLOAT_EQ(fl_delta, br_delta);
  EXPECT_FLOAT_EQ(fr_delta, bl_delta);
  EXPECT_FLOAT_EQ(fl_delta, -fr_delta);
}

}  // namespace
