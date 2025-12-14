#include <gtest/gtest.h>

#include "lanelock/actuator.hpp"

using namespace lanelock;

namespace {
SteeringActuatorConfig instant(double max_angle = 1.0) {
  SteeringActuatorConfig c = SteeringActuatorConfig::ideal();
  c.max_angle = max_angle;
  return c;
}
}  // namespace

TEST(Actuator, IdealActuatorFollowsCommandImmediately) {
  SteeringActuator a(SteeringActuatorConfig::ideal());
  EXPECT_DOUBLE_EQ(a.update(0.3, 0.01), 0.3);
  EXPECT_DOUBLE_EQ(a.update(-0.2, 0.01), -0.2);
  EXPECT_FALSE(a.saturated());
}

TEST(Actuator, RateLimitBoundsSlewRate) {
  SteeringActuatorConfig c = instant();
  c.max_rate = 0.5;
  SteeringActuator a(c);
  EXPECT_NEAR(a.update(1.0, 0.01), 0.005, 1e-12);
  EXPECT_NEAR(a.lastRate(), 0.5, 1e-9);
  for (int i = 1; i < 100; ++i) a.update(1.0, 0.01);
  EXPECT_NEAR(a.angle(), 0.5, 1e-9);
}

TEST(Actuator, SaturatesAtAuthorityLimit) {
  SteeringActuator a(instant(0.4));
  EXPECT_DOUBLE_EQ(a.update(1.0, 0.01), 0.4);
  EXPECT_TRUE(a.saturated());
  EXPECT_DOUBLE_EQ(a.update(-1.0, 0.01), -0.4);
}

TEST(Actuator, DeadTimeDelaysCommands) {
  SteeringActuatorConfig c = instant();
  c.dead_time = 0.05;
  SteeringActuator a(c);
  for (int i = 0; i < 5; ++i) EXPECT_DOUBLE_EQ(a.update(0.2, 0.01), 0.0) << "step " << i;
  EXPECT_DOUBLE_EQ(a.update(0.2, 0.01), 0.2);
}

TEST(Actuator, FirstOrderLagReachesAboutSixtyPercentInOneTimeConstant) {
  SteeringActuatorConfig c = instant();
  c.time_constant = 0.1;
  SteeringActuator a(c);
  for (int i = 0; i < 10; ++i) a.update(1.0, 0.01);
  EXPECT_GT(a.angle(), 0.55);
  EXPECT_LT(a.angle(), 0.7);
  for (int i = 0; i < 300; ++i) a.update(1.0, 0.01);
  EXPECT_NEAR(a.angle(), 1.0, 1e-6);
}

TEST(Actuator, ResetClearsState) {
  SteeringActuatorConfig c = instant();
  c.dead_time = 0.03;
  SteeringActuator a(c);
  for (int i = 0; i < 10; ++i) a.update(0.3, 0.01);
  a.reset(0.1);
  EXPECT_DOUBLE_EQ(a.angle(), 0.1);
  EXPECT_DOUBLE_EQ(a.update(0.5, 0.01), 0.1);  // delay line refilled from scratch
}
