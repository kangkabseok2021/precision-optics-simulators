#include "edge_ai_quadcopter/inference_engine.h"

#include <array>
#include <cmath>

#include <gtest/gtest.h>

using edge_ai_quadcopter::InferenceEngine;

namespace {

TEST(InferenceEngine, UninitializedReturnsNan) {
  InferenceEngine engine;
  std::array<float, 16> beams;
  beams.fill(5.0f);
  EXPECT_TRUE(std::isnan(engine.run(beams.data(), beams.size())));
}

TEST(InferenceEngine, WrongInputSizeReturnsNan) {
  InferenceEngine engine;
  engine.init();
  std::array<float, 8> too_short;
  too_short.fill(5.0f);
  EXPECT_TRUE(std::isnan(engine.run(too_short.data(), too_short.size())));
}

TEST(InferenceEngine, ClearPathGivesLowProbability) {
  InferenceEngine engine;
  engine.init();
  std::array<float, 16> beams;
  beams.fill(6.0f);  // all beams far clear
  const float prob = engine.run(beams.data(), beams.size());
  ASSERT_FALSE(std::isnan(prob));
  EXPECT_LT(prob, 0.5f);
}

TEST(InferenceEngine, ObstacleGivesHighProbability) {
  InferenceEngine engine;
  engine.init();
  std::array<float, 16> beams;
  beams.fill(6.0f);
  // A contiguous 4-beam arc reads close range, matching the training
  // distribution's obstacle-arc construction.
  for (int i = 2; i < 6; ++i) beams[i] = 0.4f;
  const float prob = engine.run(beams.data(), beams.size());
  ASSERT_FALSE(std::isnan(prob));
  EXPECT_GT(prob, 0.5f);
}

TEST(InferenceEngine, ProbabilityStaysInUnitRange) {
  InferenceEngine engine;
  engine.init();
  std::array<float, 16> near_beams;
  near_beams.fill(0.1f);
  const float prob = engine.run(near_beams.data(), near_beams.size());
  ASSERT_FALSE(std::isnan(prob));
  EXPECT_GE(prob, 0.0f);
  EXPECT_LE(prob, 1.0f);
}

TEST(InferenceEngine, ArenaSizeIsThirtyTwoKiB) {
  EXPECT_EQ(InferenceEngine::kArenaSize, static_cast<std::size_t>(32 * 1024));
}

}  // namespace
