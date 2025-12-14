#include <gtest/gtest.h>

#include "lanelock/math.hpp"

using namespace lanelock;

TEST(Math, WrapAngleKeepsValuesInRange) {
  EXPECT_NEAR(wrapAngle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(wrapAngle(kPi + 0.1), -kPi + 0.1, 1e-12);
  EXPECT_NEAR(wrapAngle(-kPi - 0.1), kPi - 0.1, 1e-12);
  EXPECT_NEAR(wrapAngle(4.0 * kPi + 0.25), 0.25, 1e-12);
  for (double a = -20.0; a < 20.0; a += 0.37) {
    const double w = wrapAngle(a);
    EXPECT_GT(w, -kPi - 1e-12);
    EXPECT_LE(w, kPi + 1e-12);
  }
}

TEST(Math, ClampAndSign) {
  EXPECT_DOUBLE_EQ(clamp(5.0, -1.0, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(clamp(-5.0, -1.0, 1.0), -1.0);
  EXPECT_DOUBLE_EQ(clamp(0.3, -1.0, 1.0), 0.3);
  EXPECT_DOUBLE_EQ(sign(2.0), 1.0);
  EXPECT_DOUBLE_EQ(sign(-0.1), -1.0);
  EXPECT_DOUBLE_EQ(sign(0.0), 0.0);
}

TEST(Math, DegreeConversionsRoundTrip) {
  EXPECT_NEAR(deg2rad(180.0), kPi, 1e-12);
  EXPECT_NEAR(rad2deg(deg2rad(37.5)), 37.5, 1e-12);
}

TEST(Math, RateLimitBoundsTheStep) {
  EXPECT_DOUBLE_EQ(rateLimit(1.0, 0.0, 0.5, 0.1), 0.05);
  EXPECT_DOUBLE_EQ(rateLimit(-1.0, 0.0, 0.5, 0.1), -0.05);
  EXPECT_DOUBLE_EQ(rateLimit(0.01, 0.0, 0.5, 0.1), 0.01);
}

TEST(Math, LowPassFilterConvergesToStepInput) {
  LowPassFilter f(0.2);
  EXPECT_DOUBLE_EQ(f.update(0.0, 0.01), 0.0);  // first sample initialises
  double y = 0.0;
  for (int i = 0; i < 20; ++i) y = f.update(1.0, 0.01);  // one time constant
  EXPECT_GT(y, 0.55);
  EXPECT_LT(y, 0.7);
  for (int i = 0; i < 500; ++i) y = f.update(1.0, 0.01);
  EXPECT_NEAR(y, 1.0, 1e-6);
}

TEST(Math, LowPassFilterWithZeroTauIsPassThrough) {
  LowPassFilter f(0.0);
  EXPECT_DOUBLE_EQ(f.update(3.0, 0.01), 3.0);
  EXPECT_DOUBLE_EQ(f.update(-2.0, 0.01), -2.0);
}
