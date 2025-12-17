// Scenario regression suite: every scenario in the library is a separate Google Test case that
// must satisfy its safety envelope. Optionally, results are compared against a recorded
// metrics baseline so performance regressions are caught even when the envelope still passes.
//
// Baseline lookup order: $LANELOCK_BASELINE, then baselines/metrics_baseline.csv in the source
// tree. Missing baseline -> the comparison tests are skipped.

#include <gtest/gtest.h>

#include <cctype>
#include <cstdlib>
#include <set>
#include <string>

#include "lanelock/io.hpp"
#include "lanelock/scenario.hpp"
#include "lanelock/simulator.hpp"

using namespace lanelock;

namespace {

#ifndef LANELOCK_DEFAULT_BASELINE
#define LANELOCK_DEFAULT_BASELINE "baselines/metrics_baseline.csv"
#endif

const std::vector<Scenario>& library() {
  static const std::vector<Scenario> scenarios = ScenarioLibrary::all();
  return scenarios;
}

const Baseline* baseline() {
  static Baseline data;
  static const bool loaded = [] {
    const char* env = std::getenv("LANELOCK_BASELINE");
    const std::string path = env != nullptr ? env : LANELOCK_DEFAULT_BASELINE;
    return loadBaselineFile(path, data) && !data.empty();
  }();
  return loaded ? &data : nullptr;
}

std::string testName(const ::testing::TestParamInfo<std::size_t>& info) {
  std::string name = library()[info.param].name;
  for (char& c : name) {
    if (!std::isalnum(static_cast<unsigned char>(c))) c = '_';
  }
  return name;
}

std::vector<std::size_t> indices() {
  std::vector<std::size_t> out(library().size());
  for (std::size_t i = 0; i < out.size(); ++i) out[i] = i;
  return out;
}

}  // namespace

// ---------------------------------------------------------------------------- library sanity

TEST(ScenarioLibrary, ContainsAtLeastFiveHundredScenarios) {
  EXPECT_EQ(ScenarioLibrary::parametric().size(), 540u);
  EXPECT_GE(library().size(), 500u);
}

TEST(ScenarioLibrary, NamesAreUniqueAndSeedsDistinct) {
  std::set<std::string> names;
  std::set<std::uint64_t> seeds;
  for (const auto& sc : library()) {
    EXPECT_TRUE(names.insert(sc.name).second) << "duplicate name " << sc.name;
    EXPECT_TRUE(seeds.insert(sc.seed).second) << "duplicate seed in " << sc.name;
  }
}

TEST(ScenarioLibrary, CoversEveryDimension) {
  for (const auto& g : ScenarioLibrary::geometries()) {
    EXPECT_FALSE(ScenarioLibrary::filtered(g + "_").empty()) << g;
  }
  for (const auto& q : ScenarioLibrary::sensorQualities()) {
    EXPECT_FALSE(ScenarioLibrary::filtered("_" + q + "_").empty()) << q;
  }
  EXPECT_FALSE(ScenarioLibrary::filtered("edge_").empty());
}

TEST(ScenarioLibrary, CurvesHitTargetLateralAcceleration) {
  for (double v : ScenarioLibrary::speeds()) {
    const Road road = ScenarioLibrary::makeRoad("curve_left_ay3", v);
    EXPECT_NEAR(road.maxAbsCurvature() * v * v, 3.0, 1e-9) << "v=" << v;
  }
}

TEST(ScenarioLibrary, FindReturnsScenarioByName) {
  const auto& first = library().front();
  const auto found = ScenarioLibrary::find(first.name);
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(found->seed, first.seed);
  EXPECT_FALSE(ScenarioLibrary::find("does_not_exist").has_value());
}

// ---------------------------------------------------------------------------- per-scenario

class ScenarioRegression : public ::testing::TestWithParam<std::size_t> {};

TEST_P(ScenarioRegression, StaysWithinSafetyEnvelope) {
  const Scenario& sc = library()[GetParam()];
  const SimulationResult r = runScenario(sc);
  EXPECT_TRUE(r.verdict.passed) << sc.name << ": " << r.verdict.summary();
  EXPECT_FALSE(r.metrics.lane_departure) << sc.name;
}

TEST_P(ScenarioRegression, NoWorseThanBaseline) {
  const Baseline* base = baseline();
  if (base == nullptr) GTEST_SKIP() << "no metrics baseline available";
  const Scenario& sc = library()[GetParam()];
  const auto it = base->find(sc.name);
  if (it == base->end()) GTEST_SKIP() << "scenario not in baseline";
  const SimulationResult r = runScenario(sc);
  const auto regressions = compareToBaseline(r.metrics, it->second);
  for (const auto& msg : regressions) ADD_FAILURE() << sc.name << ": " << msg;
}

INSTANTIATE_TEST_SUITE_P(Library, ScenarioRegression, ::testing::ValuesIn(indices()), testName);
