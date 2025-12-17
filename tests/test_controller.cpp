#include <gtest/gtest.h>

#include <cmath>

#include "lanelock/controller.hpp"

using namespace lanelock;

namespace {

SensorFrame frame(double t, double offset, double heading, double curvature = 0.0,
                  double speed = 20.0, bool valid = true) {
  SensorFrame f;
  f.time = t;
  f.speed = speed;
  f.yaw_rate = 0.0;
  f.lane.timestamp = t;
  f.lane.lateral_offset = offset;
  f.lane.heading_error = heading;
  f.lane.curvature = curvature;
  f.lane.valid = valid;
  return f;
}

PidLaneKeepingConfig kinematicConfig() {
  PidLaneKeepingConfig c;
  c.compensate_understeer = false;
  return c;
}

}  // namespace

TEST(Controller, GainsAreScheduledInverselyWithSpeedSquared) {
  PidLaneKeepingController ctrl(PidLaneKeepingConfig{}, VehicleParams{});
  const PidGains g10 = ctrl.scheduledGains(10.0);
  const PidGains g20 = ctrl.scheduledGains(20.0);
  EXPECT_NEAR(g10.kp / g20.kp, 4.0, 1e-9);
  const double wn = ctrl.config().natural_frequency;
  EXPECT_NEAR(g20.kp, VehicleParams{}.wheelbase() * wn * wn / 400.0, 1e-12);
  EXPECT_NEAR(g20.ki, g20.kp * ctrl.config().ki_ratio, 1e-12);
  // Below the scheduling floor gains stop growing.
  EXPECT_DOUBLE_EQ(ctrl.scheduledGains(1.0).kp, ctrl.scheduledGains(ctrl.config().min_schedule_speed).kp);
}

TEST(Controller, LookaheadScalesWithSpeedAndIsClamped) {
  PidLaneKeepingController ctrl(PidLaneKeepingConfig{}, VehicleParams{});
  const auto& c = ctrl.config();
  const double t_la = 2.0 * c.damping_ratio / c.natural_frequency;
  EXPECT_NEAR(ctrl.lookaheadDistance(20.0), t_la * 20.0, 1e-12);
  EXPECT_DOUBLE_EQ(ctrl.lookaheadDistance(0.5), c.min_lookahead);
  EXPECT_DOUBLE_EQ(ctrl.lookaheadDistance(500.0), c.max_lookahead);
}

TEST(Controller, UnavailableUntilFirstValidFrame) {
  PidLaneKeepingController ctrl(PidLaneKeepingConfig{}, VehicleParams{});
  EXPECT_DOUBLE_EQ(ctrl.computeSteering(frame(0.0, 0.5, 0.0, 0.0, 20.0, false), 0.01), 0.0);
  EXPECT_EQ(ctrl.mode(), ControllerMode::Unavailable);
}

TEST(Controller, SteersBackTowardsLaneCentre) {
  PidLaneKeepingController left(PidLaneKeepingConfig{}, VehicleParams{});
  EXPECT_LT(left.computeSteering(frame(0.0, 0.5, 0.0), 0.01), 0.0);
  EXPECT_EQ(left.mode(), ControllerMode::Active);

  PidLaneKeepingController right(PidLaneKeepingConfig{}, VehicleParams{});
  EXPECT_GT(right.computeSteering(frame(0.0, -0.5, 0.0), 0.01), 0.0);

  PidLaneKeepingController yawed(PidLaneKeepingConfig{}, VehicleParams{});
  EXPECT_LT(yawed.computeSteering(frame(0.0, 0.0, 0.01), 0.01), 0.0);
}

TEST(Controller, FeedforwardMatchesKinematicSteeringOnArc) {
  VehicleParams vp;
  PidLaneKeepingController ctrl(kinematicConfig(), vp);
  const double k = 0.01;
  const double steer = ctrl.computeSteering(frame(0.0, 0.0, 0.0, k), 0.01);
  EXPECT_NEAR(steer, vp.wheelbase() * k, 1e-9);
  EXPECT_NEAR(ctrl.debug().feedback, 0.0, 1e-12);
}

TEST(Controller, FeedforwardIncludesUndersteerWhenEnabled) {
  VehicleParams vp;
  PidLaneKeepingController ctrl(PidLaneKeepingConfig{}, vp);
  const double k = 0.002;
  const double v = 30.0;
  ctrl.computeSteering(frame(0.0, 0.0, 0.0, k, v), 0.01);
  EXPECT_NEAR(ctrl.debug().feedforward, (vp.wheelbase() + vp.understeerGradient() * v * v) * k,
              1e-9);
}

TEST(Controller, SideslipCompensationRemovesSteadyStateHeadingError) {
  VehicleParams vp;
  PidLaneKeepingController ctrl(PidLaneKeepingConfig{}, vp);
  const double k = 0.004;
  const double v = 30.0;
  // A perfectly tracking car on this arc has e_psi = -beta_ss.
  const double heading = -ctrl.steadyStateSideslip(k, v);
  ctrl.computeSteering(frame(0.0, 0.0, heading, k, v), 0.01);
  EXPECT_NEAR(ctrl.debug().lookahead_error, 0.0, 1e-9);
}

TEST(Controller, ModeDegradesWithMissingFrames) {
  PidLaneKeepingController ctrl(PidLaneKeepingConfig{}, VehicleParams{});
  const auto& c = ctrl.config();
  ctrl.computeSteering(frame(0.0, 0.1, 0.0), 0.01);
  EXPECT_EQ(ctrl.mode(), ControllerMode::Active);

  double t = 0.0;
  auto advanceTo = [&](double until) {
    while (t < until - 1e-9) {
      t += 0.01;
      ctrl.computeSteering(frame(t, 0.0, 0.0, 0.0, 20.0, false), 0.01);
    }
  };
  advanceTo(0.5 * c.stale_after);
  EXPECT_EQ(ctrl.mode(), ControllerMode::Active);
  advanceTo(c.stale_after + 0.05);
  EXPECT_EQ(ctrl.mode(), ControllerMode::Holding);
  advanceTo(c.hold_timeout + 0.05);
  EXPECT_EQ(ctrl.mode(), ControllerMode::Unavailable);
  EXPECT_DOUBLE_EQ(ctrl.debug().feedback, 0.0);

  // Lane comes back -> active again.
  t += 0.01;
  ctrl.computeSteering(frame(t, 0.0, 0.0), 0.01);
  EXPECT_EQ(ctrl.mode(), ControllerMode::Active);
}

TEST(Controller, DeadReckoningUsesYawRate) {
  PidLaneKeepingController ctrl(kinematicConfig(), VehicleParams{});
  ctrl.computeSteering(frame(0.0, 0.0, 0.0), 0.01);
  // No new frames, vehicle yawing left at 0.02 rad/s on a straight road for 0.2 s.
  for (int i = 1; i <= 20; ++i) {
    SensorFrame f = frame(i * 0.01, 0.0, 0.0, 0.0, 20.0, false);
    f.yaw_rate = 0.02;
    ctrl.computeSteering(f, 0.01);
  }
  EXPECT_NEAR(ctrl.debug().est_heading, 0.02 * 0.2, 1e-9);
  EXPECT_GT(ctrl.debug().est_offset, 0.0);
}

TEST(Controller, OutputIsLimitedToMaxSteer) {
  PidLaneKeepingConfig c;
  c.max_steer = 0.05;
  PidLaneKeepingController ctrl(c, VehicleParams{});
  const double steer = ctrl.computeSteering(frame(0.0, 0.0, 0.0, 0.1, 10.0), 0.01);
  EXPECT_DOUBLE_EQ(steer, 0.05);
}

TEST(Controller, ResetReturnsToInitialState) {
  PidLaneKeepingController ctrl(PidLaneKeepingConfig{}, VehicleParams{});
  ctrl.computeSteering(frame(0.0, 0.3, 0.0), 0.01);
  ctrl.reset();
  EXPECT_EQ(ctrl.mode(), ControllerMode::Unavailable);
  EXPECT_DOUBLE_EQ(ctrl.debug().est_offset, 0.0);
}

TEST(Controller, ModeNames) {
  EXPECT_STREQ(toString(ControllerMode::Active), "active");
  EXPECT_STREQ(toString(ControllerMode::Holding), "holding");
  EXPECT_STREQ(toString(ControllerMode::Unavailable), "unavailable");
}
