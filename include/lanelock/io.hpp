#pragma once

#include <map>
#include <ostream>
#include <string>
#include <vector>

#include "lanelock/metrics.hpp"
#include "lanelock/simulator.hpp"

namespace lanelock {

/// Streams TraceSamples as CSV rows (header written on construction).
class CsvTraceWriter {
 public:
  explicit CsvTraceWriter(std::ostream& out);
  void write(const TraceSample& sample);

 private:
  std::ostream& out_;
};

/// One row of a suite summary / baseline.
struct ScenarioRecord {
  std::string name;
  Metrics metrics;
  bool passed = true;
  std::string verdict;
};

void writeSummaryCsv(std::ostream& out, const std::vector<ScenarioRecord>& records);

/// Baseline file: CSV with the key regression metrics per scenario name.
using Baseline = std::map<std::string, Metrics>;

void writeBaseline(std::ostream& out, const std::vector<ScenarioRecord>& records);
Baseline readBaseline(std::istream& in);
bool loadBaselineFile(const std::string& path, Baseline& baseline);

/// Tolerances used when comparing a run against a recorded baseline.
struct BaselineTolerance {
  double lateral_error_abs = 0.02;   // m
  double lateral_error_rel = 0.10;   // fraction
  double lateral_accel_abs = 0.10;   // m/s^2
};

/// Returns human-readable regressions (empty when within tolerance). Only *worse* results are
/// regressions; improvements are reported by the caller if desired.
std::vector<std::string> compareToBaseline(const Metrics& current, const Metrics& baseline,
                                           const BaselineTolerance& tol = {});

}  // namespace lanelock
