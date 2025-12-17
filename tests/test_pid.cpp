#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "lanelock/pid.hpp"

using namespace lanelock;

namespace {
constexpr double kDt = 0.01;
}

TEST(Pid, ProportionalOnly) {
  Pid pid({2.0, 0.0, 0.0}, {});
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.25, kDt), 1.5);
  EXPECT_DOUBLE_EQ(pid.lastTerms().p, 1.5);
  EXPECT_DOUBLE_EQ(pid.lastTerms().i, 0.0);
}

TEST(Pid, IntegralAccumulatesError) {
  Pid pid({0.0, 1.0, 0.0}, {});
  for (int i = 0; i < 100; ++i) pid.update(1.0, 0.0, kDt);
  EXPECT_NEAR(pid.integral(), 1.0, 1e-9);
  EXPECT_NEAR(pid.update(1.0, 0.0, kDt), 1.0, 1e-9);
}

TEST(Pid, IntegralIsClamped) {
  PidLimits lim;
  lim.integral_limit = 0.5;
  Pid pid({0.0, 1.0, 0.0}, lim);
  for (int i = 0; i < 1000; ++i) pid.update(1.0, 0.0, kDt);
  EXPECT_DOUBLE_EQ(pid.integral(), 0.5);
}

TEST(Pid, ConditionalIntegrationStopsWindupAtSaturation) {
  PidLimits lim;
  lim.output_max = 1.0;
  lim.output_min = -1.0;
  Pid pid({0.0, 1.0, 0.0}, lim);
  for (int i = 0; i < 500; ++i) pid.update(1.0, 0.0, kDt);
  EXPECT_LE(pid.integral(), 1.0 + 2.0 * kDt);
  EXPECT_DOUBLE_EQ(pid.lastTerms().output, 1.0);
}

TEST(Pid, BackCalculationSettlesAtAnalyticEquilibrium) {
  PidLimits lim;
  lim.output_max = 1.0;
  lim.output_min = -1.0;
  lim.back_calculation_gain = 5.0;
  Pid pid({0.0, 1.0, 0.0}, lim);
  for (int i = 0; i < 2000; ++i) pid.update(1.0, 0.0, kDt);
  // ki * e + kb * (u_sat - I) = 0  ->  I = u_sat + ki * e / kb = 1.2
  EXPECT_NEAR(pid.integral(), 1.2, 1e-3);
}

TEST(Pid, WindupRecoveryIsFastWithAntiWindup) {
  PidLimits lim;
  lim.output_max = 1.0;
  lim.output_min = -1.0;
  lim.back_calculation_gain = 5.0;
  Pid pid({1.0, 1.0, 0.0}, lim);
  for (int i = 0; i < 1000; ++i) pid.update(5.0, 0.0, kDt);  // long saturation
  // Error reverses; output must leave saturation almost immediately.
  int steps = 0;
  while (pid.update(-0.5, 0.0, kDt) >= 1.0 && steps < 1000) ++steps;
  EXPECT_LT(steps, 10);
}

TEST(Pid, DerivativeActsOnMeasurementNotSetpoint) {
  Pid pid({0.0, 0.0, 1.0}, {});
  pid.update(0.0, 0.0, kDt);
  EXPECT_DOUBLE_EQ(pid.update(10.0, 0.0, kDt), 0.0);  // set-point jump -> no kick
  pid.reset();
  double out = 0.0;
  for (int i = 0; i < 10; ++i) out = pid.update(0.0, 2.0 * i * kDt, kDt);  // ramp at 2/s
  EXPECT_NEAR(out, -2.0, 1e-9);
}

TEST(Pid, DerivativeFilterSmoothsNoise) {
  Pid raw({0.0, 0.0, 1.0}, {});
  Pid filtered({0.0, 0.0, 1.0}, {}, 0.1);
  double max_raw = 0.0;
  double max_filtered = 0.0;
  for (int i = 0; i < 200; ++i) {
    const double noisy = (i % 2 == 0) ? 0.01 : -0.01;
    max_raw = std::max(max_raw, std::abs(raw.update(0.0, noisy, kDt)));
    max_filtered = std::max(max_filtered, std::abs(filtered.update(0.0, noisy, kDt)));
  }
  EXPECT_GT(max_raw, 1.9);
  EXPECT_LT(max_filtered, 0.5 * max_raw);
}

TEST(Pid, GainChangeIsBumpless) {
  Pid pid({0.0, 2.0, 0.0}, {});
  for (int i = 0; i < 50; ++i) pid.update(1.0, 0.0, kDt);
  const double before = pid.integral();
  pid.setGains({0.0, 10.0, 0.0});
  EXPECT_DOUBLE_EQ(pid.integral(), before);
  const double out = pid.update(0.0, 0.0, kDt);  // zero error: output == stored integral
  EXPECT_DOUBLE_EQ(out, before);
}

TEST(Pid, FrozenIntegratorHoldsValue) {
  Pid pid({0.0, 1.0, 0.0}, {});
  for (int i = 0; i < 10; ++i) pid.update(1.0, 0.0, kDt);
  const double held = pid.integral();
  pid.setIntegratorFrozen(true);
  for (int i = 0; i < 10; ++i) pid.update(1.0, 0.0, kDt);
  EXPECT_DOUBLE_EQ(pid.integral(), held);
}
