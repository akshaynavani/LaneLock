#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <sstream>

#include "lanelock/io.hpp"
#include "lanelock/metrics.hpp"
#include "lanelock/scenario.hpp"

using namespace lanelock;

namespace {
VehicleState at(double e_y, double ay = 0.0) {
  VehicleState s;
  s.e_y = e_y;
  s.lateral_accel = ay;
  return s;
}
}  // namespace

TEST(Metrics, TracksPeaksAndRms) {
  MetricsAccumulator acc(3.7, 1.85);
  acc.add(0.01, 0.01, at(0.3, 1.0), 0.01, 0.1, ControllerMode::Active);
  acc.add(0.02, 0.01, at(-0.4, -2.0), -0.02, -0.3, ControllerMode::Active);
  const Metrics m = acc.finish();
  EXPECT_EQ(m.samples, 2);
  EXPECT_DOUBLE_EQ(m.max_abs_lateral_error, 0.4);
  EXPECT_NEAR(m.rms_lateral_error, std::sqrt((0.09 + 0.16) / 2.0), 1e-12);
  EXPECT_DOUBLE_EQ(m.max_abs_lateral_accel, 2.0);
  EXPECT_DOUBLE_EQ(m.max_abs_steer, 0.02);
  EXPECT_DOUBLE_EQ(m.max_abs_steer_rate, 0.3);
  EXPECT_DOUBLE_EQ(m.final_lateral_error, 0.4);
  EXPECT_FALSE(m.lane_departure);
  EXPECT_NEAR(m.min_boundary_margin, 1.85 - 0.4 - 0.925, 1e-12);
}

TEST(Metrics, DetectsLaneDepartureWhenTyreCrossesLine) {
  MetricsAccumulator acc(3.7, 1.85);
  acc.add(0.01, 0.01, at(0.9), 0.0, 0.0, ControllerMode::Active);
  EXPECT_FALSE(acc.finish().lane_departure);
  acc.add(0.02, 0.01, at(0.95), 0.0, 0.0, ControllerMode::Active);
  const Metrics m = acc.finish();
  EXPECT_TRUE(m.lane_departure);
  EXPECT_DOUBLE_EQ(m.first_departure_time, 0.02);
  EXPECT_LT(m.min_boundary_margin, 0.0);
}

TEST(Metrics, AccumulatesModeTimesAndSettling) {
  MetricsAccumulator acc(3.7, 1.85, 0.2);
  for (int i = 1; i <= 100; ++i) {
    const double t = i * 0.01;
    const ControllerMode mode = i <= 20 ? ControllerMode::Unavailable
                                : i <= 50 ? ControllerMode::Holding
                                          : ControllerMode::Active;
    acc.add(t, 0.01, at(i <= 40 ? 0.5 : 0.05), 0.0, 0.0, mode);
  }
  const Metrics m = acc.finish();
  EXPECT_NEAR(m.time_unavailable, 0.2, 1e-9);
  EXPECT_NEAR(m.time_holding, 0.3, 1e-9);
  EXPECT_NEAR(m.settling_time, 0.4, 1e-9);
}

TEST(Envelope, PassesCleanRunAndListsEveryViolation) {
  Metrics ok;
  ok.max_abs_lateral_error = 0.2;
  ok.rms_lateral_error = 0.05;
  ok.max_abs_lateral_accel = 2.0;
  ok.final_lateral_error = 0.01;
  EXPECT_TRUE(evaluate(ok, SafetyEnvelope{}).passed);
  EXPECT_EQ(evaluate(ok, SafetyEnvelope{}).summary(), "PASS");

  Metrics bad = ok;
  bad.lane_departure = true;
  bad.first_departure_time = 3.0;
  bad.max_abs_lateral_error = 1.2;
  bad.max_abs_lateral_accel = 7.0;
  const Verdict v = evaluate(bad, SafetyEnvelope{});
  EXPECT_FALSE(v.passed);
  EXPECT_EQ(v.violations.size(), 3u);
  EXPECT_NE(v.summary().find("lane_departure"), std::string::npos);
}

TEST(Envelope, UnavailableTimeLimitIsOptional) {
  Metrics m;
  m.time_unavailable = 5.0;
  EXPECT_TRUE(evaluate(m, SafetyEnvelope{}).passed);
  SafetyEnvelope strict;
  strict.max_time_unavailable = 1.0;
  EXPECT_FALSE(evaluate(m, strict).passed);
}

TEST(Disturbance, TrapezoidProfile) {
  Disturbance d{1.0, 0.5, 1.0, 400.0};
  EXPECT_DOUBLE_EQ(d.forceAt(0.5), 0.0);
  EXPECT_DOUBLE_EQ(d.forceAt(1.25), 200.0);
  EXPECT_DOUBLE_EQ(d.forceAt(2.0), 400.0);
  EXPECT_DOUBLE_EQ(d.forceAt(2.75), 200.0);
  EXPECT_DOUBLE_EQ(d.forceAt(3.5), 0.0);
}

TEST(Io, BaselineRoundTrip) {
  ScenarioRecord a{"alpha", {}, true, "PASS"};
  a.metrics.max_abs_lateral_error = 0.123456789;
  a.metrics.rms_lateral_error = 0.05;
  a.metrics.max_abs_lateral_accel = 2.5;
  ScenarioRecord b{"beta", {}, false, "FAIL"};
  b.metrics.max_abs_lateral_error = 0.9;

  std::stringstream ss;
  writeBaseline(ss, {a, b});
  const Baseline base = readBaseline(ss);
  ASSERT_EQ(base.size(), 2u);
  EXPECT_NEAR(base.at("alpha").max_abs_lateral_error, 0.123456789, 1e-9);
  EXPECT_NEAR(base.at("alpha").max_abs_lateral_accel, 2.5, 1e-9);
  EXPECT_NEAR(base.at("beta").max_abs_lateral_error, 0.9, 1e-9);
}

TEST(Io, BaselineComparisonFlagsOnlyRegressions) {
  Metrics before;
  before.max_abs_lateral_error = 0.30;
  before.rms_lateral_error = 0.10;
  before.max_abs_lateral_accel = 2.0;

  Metrics same = before;
  EXPECT_TRUE(compareToBaseline(same, before).empty());

  Metrics better = before;
  better.max_abs_lateral_error = 0.1;
  EXPECT_TRUE(compareToBaseline(better, before).empty());

  Metrics worse = before;
  worse.max_abs_lateral_error = 0.40;  // +33% > 10% tolerance
  worse.max_abs_lateral_accel = 2.5;
  EXPECT_EQ(compareToBaseline(worse, before).size(), 2u);
}

TEST(Io, TraceWriterEmitsHeaderAndRows) {
  std::stringstream ss;
  CsvTraceWriter w(ss);
  TraceSample s;
  s.time = 0.5;
  s.mode = ControllerMode::Holding;
  w.write(s);
  std::string header;
  std::string row;
  std::getline(ss, header);
  std::getline(ss, row);
  EXPECT_EQ(header.rfind("time,s,x,y,e_y", 0), 0u);
  EXPECT_EQ(row.rfind("0.5,", 0), 0u);
  EXPECT_NE(row.find("holding"), std::string::npos);
  // Same number of columns in header and row.
  EXPECT_EQ(std::count(header.begin(), header.end(), ','), std::count(row.begin(), row.end(), ','));
}
