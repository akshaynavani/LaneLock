// Closed-loop behaviour of the PID lane-keeping controller in targeted situations.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "lanelock/scenario.hpp"
#include "lanelock/simulator.hpp"

using namespace lanelock;

namespace {

Scenario idealScenario(double speed, Road road) {
  Scenario sc;
  sc.name = "test";
  sc.speed = speed;
  sc.road = std::move(road);
  sc.sensors = SensorSuiteConfig::ideal();
  sc.duration = 20.0;
  return sc;
}

/// A controller that never steers: used to show the envelope catches unsafe behaviour.
class NoSteeringController : public LateralController {
 public:
  double computeSteering(const SensorFrame&, double) override { return 0.0; }
  void reset() override {}
  std::string name() const override { return "none"; }
  ControllerMode mode() const override { return ControllerMode::Active; }
  const ControllerDebug& debug() const override { return debug_; }

 private:
  ControllerDebug debug_;
};

}  // namespace

TEST(ClosedLoop, ConvergesToLaneCentreFromOffset) {
  Scenario sc = idealScenario(25.0, Road::Builder().straight(1000.0).build());
  sc.initial_offset = 0.6;
  const SimulationResult r = runScenario(sc);
  EXPECT_TRUE(r.verdict.passed) << r.verdict.summary();
  EXPECT_LT(r.metrics.final_lateral_error, 0.02);
  EXPECT_LT(r.metrics.settling_time, 6.0);
  EXPECT_FALSE(r.metrics.lane_departure);
}

TEST(ClosedLoop, ConvergenceIsWellDampedAcrossSpeeds) {
  for (double v : {12.0, 20.0, 30.0, 38.0}) {
    Scenario sc = idealScenario(v, Road::Builder().straight(2000.0).build());
    sc.initial_offset = 0.5;
    double min_e_y = 0.0;
    Simulator sim(sc);
    sim.run([&](const TraceSample& s) { min_e_y = std::min(min_e_y, s.e_y); });
    // Overshoot below the centre line stays small (zeta ~ 0.9 design).
    EXPECT_GT(min_e_y, -0.15) << "v=" << v;
    EXPECT_LT(std::abs(sim.state().e_y), 0.02) << "v=" << v;
  }
}

TEST(ClosedLoop, TracksConstantCurveWithSmallError) {
  for (VehicleModelType model : {VehicleModelType::Kinematic, VehicleModelType::Dynamic}) {
    const double v = 25.0;
    Scenario sc = idealScenario(
        v, Road::Builder().straight(75.0).curve(400.0, 2.0 / (v * v), 50.0).straight(500.0).build());
    sc.model = model;
    sc.controller.compensate_understeer = model == VehicleModelType::Dynamic;
    const SimulationResult r = runScenario(sc);
    EXPECT_TRUE(r.verdict.passed) << toString(model) << ": " << r.verdict.summary();
    EXPECT_LT(r.metrics.max_abs_lateral_error, 0.35) << toString(model);
    EXPECT_NEAR(r.metrics.max_abs_lateral_accel, 2.0, 0.6) << toString(model);
  }
}

TEST(ClosedLoop, RejectsCrosswindGust) {
  Scenario sc = idealScenario(30.0, Road::Builder().straight(2000.0).build());
  sc.disturbances.push_back({5.0, 0.3, 2.0, 700.0});
  const SimulationResult r = runScenario(sc);
  EXPECT_TRUE(r.verdict.passed) << r.verdict.summary();
  EXPECT_GT(r.metrics.max_abs_lateral_error, 0.01);  // the gust is felt...
  EXPECT_LT(r.metrics.max_abs_lateral_error, 0.5);   // ...but rejected
  EXPECT_LT(r.metrics.final_lateral_error, 0.05);
}

TEST(ClosedLoop, RidesThroughShortCameraBlackout) {
  Scenario sc = idealScenario(25.0, Road::Builder().straight(100.0).curve(300.0, 0.003, 50.0)
                                        .straight(500.0).build());
  sc.sensors.camera.blackout_start = 6.0;
  sc.sensors.camera.blackout_duration = 0.8;
  std::vector<ControllerMode> modes;
  Simulator sim(sc);
  const SimulationResult r = sim.run([&](const TraceSample& s) { modes.push_back(s.mode); });
  EXPECT_TRUE(r.verdict.passed) << r.verdict.summary();
  EXPECT_GT(r.metrics.time_holding, 0.3);
  EXPECT_DOUBLE_EQ(r.metrics.time_unavailable, 0.0);  // shorter than the hold timeout
  EXPECT_NE(std::find(modes.begin(), modes.end(), ControllerMode::Holding), modes.end());
}

TEST(ClosedLoop, LongBlackoutTriggersUnavailableMode) {
  Scenario sc = idealScenario(25.0, Road::Builder().straight(2000.0).build());
  sc.sensors.camera.blackout_start = 5.0;
  sc.sensors.camera.blackout_duration = 3.0;
  const SimulationResult r = runScenario(sc);
  EXPECT_GT(r.metrics.time_unavailable, 1.5);
  EXPECT_FALSE(r.metrics.lane_departure);
}

TEST(ClosedLoop, NoSteeringControllerLeavesLaneOnCurve) {
  Scenario sc = idealScenario(25.0, Road::Builder().straight(50.0).arc(600.0, 1.0 / 300.0).build());
  Simulator sim(sc, std::make_unique<NoSteeringController>());
  const SimulationResult r = sim.run();
  EXPECT_FALSE(r.verdict.passed);
  EXPECT_TRUE(r.metrics.lane_departure);
}

TEST(ClosedLoop, IdenticalSeedsGiveIdenticalRuns) {
  auto sc = ScenarioLibrary::find("s_curve_ay2_v30_degraded_offset_left_gust");
  ASSERT_TRUE(sc.has_value());
  const SimulationResult a = runScenario(*sc);
  const SimulationResult b = runScenario(*sc);
  EXPECT_EQ(a.metrics.max_abs_lateral_error, b.metrics.max_abs_lateral_error);
  EXPECT_EQ(a.metrics.rms_lateral_error, b.metrics.rms_lateral_error);
  EXPECT_EQ(a.camera_frames_dropped, b.camera_frames_dropped);
}

TEST(ClosedLoop, DifferentSeedsChangeNoiseRealisation) {
  auto sc = ScenarioLibrary::find("straight_v25_degraded_centered_calm");
  ASSERT_TRUE(sc.has_value());
  Scenario other = *sc;
  other.seed += 1;
  EXPECT_NE(runScenario(*sc).metrics.rms_lateral_error, runScenario(other).metrics.rms_lateral_error);
}

TEST(ClosedLoop, StepApiMatchesRun) {
  auto sc = ScenarioLibrary::find("curve_left_ay1_v20_nominal_centered_calm");
  ASSERT_TRUE(sc.has_value());
  Simulator stepped(*sc);
  while (!stepped.done()) stepped.step();
  Simulator batch(*sc);
  batch.run();
  EXPECT_DOUBLE_EQ(stepped.state().e_y, batch.state().e_y);
  EXPECT_NEAR(stepped.time(), sc->duration, 1e-9);
}

TEST(ClosedLoop, WorldPoseTraceFollowsRoad) {
  Scenario sc = idealScenario(20.0, Road::Builder().arc(2000.0, 0.002).build());
  sc.duration = 5.0;
  Simulator sim(sc);
  TraceSample last;
  sim.run([&](const TraceSample& s) { last = s; }, /*world_pose=*/true);
  const Pose2D centre = sc.road.poseAt(last.s);
  EXPECT_NEAR(std::hypot(last.x - centre.x, last.y - centre.y), std::abs(last.e_y), 1e-6);
}
