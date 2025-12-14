#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "lanelock/noise.hpp"
#include "lanelock/sensors.hpp"

using namespace lanelock;

namespace {

double mean(const std::vector<double>& v) {
  double s = 0.0;
  for (double x : v) s += x;
  return s / static_cast<double>(v.size());
}

double stddev(const std::vector<double>& v) {
  const double m = mean(v);
  double s = 0.0;
  for (double x : v) s += (x - m) * (x - m);
  return std::sqrt(s / static_cast<double>(v.size() - 1));
}

LaneCameraConfig noiselessCamera(double rate_hz, double latency) {
  LaneCameraConfig c = LaneCameraConfig::ideal();
  c.rate_hz = rate_hz;
  c.latency = latency;
  return c;
}

}  // namespace

// ---------------------------------------------------------------------------- noise

TEST(Noise, SameSeedGivesIdenticalSequence) {
  NoiseParams p{0.1, 0.0, 0.01, 0.0};
  NoiseModel a(p, 42);
  NoiseModel b(p, 42);
  for (int i = 0; i < 100; ++i) EXPECT_DOUBLE_EQ(a.apply(1.0, 0.01), b.apply(1.0, 0.01));
}

TEST(Noise, DifferentSeedsDiffer) {
  NoiseParams p{0.1, 0.0, 0.0, 0.0};
  NoiseModel a(p, 1);
  NoiseModel b(p, 2);
  int equal = 0;
  for (int i = 0; i < 100; ++i) equal += a.apply(0.0, 0.01) == b.apply(0.0, 0.01);
  EXPECT_LT(equal, 5);
}

TEST(Noise, GaussianStatisticsMatchParameters) {
  NoiseModel n(NoiseParams{0.1, 0.5, 0.0, 0.0}, 7);
  std::vector<double> samples;
  for (int i = 0; i < 20000; ++i) samples.push_back(n.apply(2.0, 0.01) - 2.0);
  EXPECT_NEAR(mean(samples), 0.5, 0.005);
  EXPECT_NEAR(stddev(samples), 0.1, 0.005);
}

TEST(Noise, NoneIsExact) {
  NoiseModel n(NoiseParams::none(), 3);
  EXPECT_DOUBLE_EQ(n.apply(1.2345, 0.01), 1.2345);
}

TEST(Noise, QuantisationSnapsToResolution) {
  NoiseModel n(NoiseParams{0.0, 0.0, 0.0, 0.05}, 1);
  EXPECT_NEAR(n.apply(0.123, 0.01), 0.10, 1e-12);
  EXPECT_NEAR(n.apply(0.126, 0.01), 0.15, 1e-12);
}

TEST(Noise, RandomWalkDriftAccumulates) {
  NoiseModel n(NoiseParams{0.0, 0.0, 0.05, 0.0}, 11);
  double out = 0.0;
  for (int i = 0; i < 1000; ++i) out = n.apply(0.0, 0.01);
  EXPECT_NE(n.drift(), 0.0);
  EXPECT_DOUBLE_EQ(out, n.drift());
}

TEST(Noise, DeriveSeedProducesDistinctStreams) {
  EXPECT_NE(deriveSeed(1, 0), deriveSeed(1, 1));
  EXPECT_NE(deriveSeed(1, 0), deriveSeed(2, 0));
  EXPECT_EQ(deriveSeed(5, 3), deriveSeed(5, 3));
}

TEST(Dropout, RateMatchesProbability) {
  DropoutModel d(0.1, 1, 99);
  int dropped = 0;
  const int n = 20000;
  for (int i = 0; i < n; ++i) dropped += d.dropped();
  EXPECT_NEAR(static_cast<double>(dropped) / n, 0.1, 0.01);
}

TEST(Dropout, ZeroProbabilityNeverDrops) {
  DropoutModel d(0.0, 5, 1);
  for (int i = 0; i < 1000; ++i) EXPECT_FALSE(d.dropped());
}

TEST(Dropout, BurstsLastAtLeastBurstLength) {
  DropoutModel d(0.05, 4, 5);
  int run = 0;
  int bursts = 0;
  for (int i = 0; i < 20000; ++i) {
    if (d.dropped()) {
      ++run;
    } else if (run > 0) {
      EXPECT_GE(run, 4);
      ++bursts;
      run = 0;
    }
  }
  EXPECT_GT(bursts, 10);
}

// ---------------------------------------------------------------------------- camera

TEST(LaneCamera, IdealCameraReportsTruth) {
  const Road road = Road::Builder().straight(100.0).arc(500.0, 0.004).build();
  LaneCamera cam(LaneCameraConfig::ideal(), 1);
  VehicleState truth;
  truth.s = 200.0;
  truth.e_y = 0.37;
  truth.e_psi = -0.02;
  const LaneMeasurement m = cam.update(0.0, truth, road);
  ASSERT_TRUE(m.valid);
  EXPECT_DOUBLE_EQ(m.lateral_offset, 0.37);
  EXPECT_DOUBLE_EQ(m.heading_error, -0.02);
  EXPECT_DOUBLE_EQ(m.curvature, 0.004);
  EXPECT_NEAR(m.curvature_rate, 0.0, 1e-12);
}

TEST(LaneCamera, ReportsCurvatureRateOnClothoid) {
  const Road road = Road::Builder().clothoid(1000.0, 0.0, 0.01).build();
  LaneCamera cam(LaneCameraConfig::ideal(), 1);
  VehicleState truth;
  truth.s = 300.0;
  const LaneMeasurement m = cam.update(0.0, truth, road);
  EXPECT_NEAR(m.curvature, 0.003, 1e-12);
  EXPECT_NEAR(m.curvature_rate, 1e-5, 1e-10);
}

TEST(LaneCamera, OutputIsDelayedByLatency) {
  const Road road = Road::Builder().straight(1000.0).build();
  LaneCamera cam(noiselessCamera(20.0, 0.05), 1);
  VehicleState truth;
  const double dt = 0.01;
  bool seen_valid = false;
  for (int i = 0; i <= 100; ++i) {
    const double t = i * dt;
    truth.e_y = t;  // encode capture time in the offset
    const LaneMeasurement m = cam.update(t, truth, road);
    if (t < 0.05 - 1e-9) {
      EXPECT_FALSE(m.valid) << "t=" << t;
      continue;
    }
    ASSERT_TRUE(m.valid) << "t=" << t;
    seen_valid = true;
    EXPECT_LE(m.timestamp, t - 0.05 + 1e-9);
    EXPECT_GT(m.timestamp, t - 0.05 - 0.05 - 1e-9);  // never older than latency + one frame
    EXPECT_NEAR(m.lateral_offset, m.timestamp, 1e-9);
  }
  EXPECT_TRUE(seen_valid);
  EXPECT_EQ(cam.framesCaptured(), 21);  // 20 Hz over 1 s, including t=0
}

TEST(LaneCamera, BlackoutWindowInvalidatesFrames) {
  const Road road = Road::Builder().straight(1000.0).build();
  LaneCameraConfig c = noiselessCamera(20.0, 0.0);
  c.blackout_start = 0.5;
  c.blackout_duration = 0.3;
  LaneCamera cam(c, 1);
  VehicleState truth;
  for (int i = 0; i <= 100; ++i) {
    const double t = i * 0.01;
    const LaneMeasurement m = cam.update(t, truth, road);
    const bool in_blackout = m.timestamp >= 0.5 - 1e-9 && m.timestamp < 0.8 - 1e-9;
    EXPECT_EQ(m.valid, !in_blackout) << "t=" << t;
  }
  EXPECT_EQ(cam.framesDropped(), 6);
}

TEST(LaneCamera, LosesLaneFarOutsideBoundary) {
  const Road road = Road::Builder().straight(1000.0).build();
  LaneCamera cam(LaneCameraConfig::ideal(), 1);
  VehicleState truth;
  truth.e_y = 6.0;
  EXPECT_FALSE(cam.update(0.0, truth, road).valid);
}

TEST(SensorSuite, IdealSuitePassesThroughSpeedAndYawRate) {
  const Road road = Road::Builder().straight(1000.0).build();
  SensorSuite suite(SensorSuiteConfig::ideal(), 3);
  VehicleState truth;
  truth.vx = 27.0;
  truth.yaw_rate = 0.05;
  const SensorFrame f = suite.update(0.0, 0.01, truth, road);
  EXPECT_DOUBLE_EQ(f.speed, 27.0);
  EXPECT_DOUBLE_EQ(f.yaw_rate, 0.05);
  EXPECT_TRUE(f.lane.valid);
}
