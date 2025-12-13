// lanelock_sim — command-line front end for the LaneLock closed-loop simulator.
//
//   lanelock_sim list [filter]
//   lanelock_sim run <scenario> [--trace trace.csv]
//   lanelock_sim suite [--filter text] [--summary summary.csv] [--baseline baseline.csv]
//                      [--write-baseline baseline.csv] [--jobs N] [--verbose]

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "lanelock/io.hpp"
#include "lanelock/scenario.hpp"
#include "lanelock/simulator.hpp"

using namespace lanelock;

namespace {

void usage() {
  std::cerr << "usage:\n"
               "  lanelock_sim list [filter]\n"
               "  lanelock_sim run <scenario> [--trace trace.csv]\n"
               "  lanelock_sim suite [--filter text] [--summary summary.csv]\n"
               "                     [--baseline baseline.csv] [--write-baseline baseline.csv]\n"
               "                     [--jobs N] [--verbose]\n";
}

/// Returns the value following `flag`, or `fallback` when the flag is absent.
std::string option(const std::vector<std::string>& args, const std::string& flag,
                   const std::string& fallback = "") {
  for (std::size_t i = 0; i + 1 < args.size(); ++i) {
    if (args[i] == flag) return args[i + 1];
  }
  return fallback;
}

bool hasFlag(const std::vector<std::string>& args, const std::string& flag) {
  return std::find(args.begin(), args.end(), flag) != args.end();
}

void printMetrics(const Metrics& m) {
  std::cout << std::fixed << std::setprecision(3)
            << "  max |e_y|          " << m.max_abs_lateral_error << " m\n"
            << "  rms e_y            " << m.rms_lateral_error << " m\n"
            << "  max |e_psi|        " << m.max_abs_heading_error << " rad\n"
            << "  max |steer|        " << m.max_abs_steer << " rad\n"
            << "  max |steer rate|   " << m.max_abs_steer_rate << " rad/s\n"
            << "  max |a_y|          " << m.max_abs_lateral_accel << " m/s^2\n"
            << "  min line margin    " << m.min_boundary_margin << " m\n"
            << "  final |e_y|        " << m.final_lateral_error << " m\n"
            << "  settling time      " << m.settling_time << " s\n"
            << "  time holding       " << m.time_holding << " s\n"
            << "  time unavailable   " << m.time_unavailable << " s\n"
            << "  lane departure     " << (m.lane_departure ? "YES" : "no") << '\n';
}

int cmdList(const std::vector<std::string>& args) {
  const std::string filter = args.empty() ? "" : args[0];
  const auto scenarios = ScenarioLibrary::filtered(filter);
  for (const auto& sc : scenarios) {
    std::cout << sc.name << "  (" << toString(sc.model) << ", " << sc.speed << " m/s, "
              << sc.duration << " s)\n";
  }
  std::cout << scenarios.size() << " scenario(s)\n";
  return 0;
}

int cmdRun(const std::vector<std::string>& args) {
  if (args.empty()) {
    usage();
    return 2;
  }
  const auto scenario = ScenarioLibrary::find(args[0]);
  if (!scenario) {
    std::cerr << "unknown scenario: " << args[0] << " (try `lanelock_sim list`)\n";
    return 2;
  }

  Simulator sim(*scenario);
  SimulationResult result;
  const std::string trace_path = option(args, "--trace");
  if (!trace_path.empty()) {
    std::ofstream out(trace_path);
    if (!out) {
      std::cerr << "cannot open " << trace_path << '\n';
      return 2;
    }
    CsvTraceWriter writer(out);
    result = sim.run([&](const TraceSample& s) { writer.write(s); }, /*world_pose=*/true);
    std::cout << "trace written to " << trace_path << '\n';
  } else {
    result = sim.run();
  }

  std::cout << scenario->name << ": " << result.verdict.summary() << '\n';
  printMetrics(result.metrics);
  std::cout << "  camera frames      " << result.camera_frames << " ("
            << result.camera_frames_dropped << " dropped)\n";
  return result.verdict.passed ? 0 : 1;
}

int cmdSuite(const std::vector<std::string>& args) {
  const auto scenarios = ScenarioLibrary::filtered(option(args, "--filter"));
  const bool verbose = hasFlag(args, "--verbose");
  unsigned jobs = std::max(1u, std::thread::hardware_concurrency());
  const std::string jobs_opt = option(args, "--jobs");
  if (!jobs_opt.empty()) jobs = static_cast<unsigned>(std::max(1, std::atoi(jobs_opt.c_str())));

  std::vector<ScenarioRecord> records(scenarios.size());
  std::atomic<std::size_t> next{0};
  auto worker = [&]() {
    for (std::size_t i = next++; i < scenarios.size(); i = next++) {
      const SimulationResult r = runScenario(scenarios[i]);
      records[i] = {scenarios[i].name, r.metrics, r.verdict.passed, r.verdict.summary()};
    }
  };
  std::vector<std::thread> pool;
  for (unsigned t = 0; t < jobs; ++t) pool.emplace_back(worker);
  for (auto& th : pool) th.join();

  int failed = 0;
  for (const auto& r : records) {
    if (!r.passed) ++failed;
    if (verbose || !r.passed) std::cout << r.name << ": " << r.verdict << '\n';
  }

  int regressed = 0;
  const std::string baseline_path = option(args, "--baseline");
  if (!baseline_path.empty()) {
    Baseline baseline;
    if (!loadBaselineFile(baseline_path, baseline)) {
      std::cerr << "cannot read baseline " << baseline_path << '\n';
      return 2;
    }
    for (const auto& r : records) {
      const auto it = baseline.find(r.name);
      if (it == baseline.end()) continue;
      const auto diffs = compareToBaseline(r.metrics, it->second);
      if (!diffs.empty()) {
        ++regressed;
        for (const auto& d : diffs) std::cout << r.name << ": " << d << '\n';
      }
    }
  }

  const std::string summary_path = option(args, "--summary");
  if (!summary_path.empty()) {
    std::ofstream out(summary_path);
    writeSummaryCsv(out, records);
    std::cout << "summary written to " << summary_path << '\n';
  }
  const std::string write_baseline_path = option(args, "--write-baseline");
  if (!write_baseline_path.empty()) {
    std::ofstream out(write_baseline_path);
    writeBaseline(out, records);
    std::cout << "baseline written to " << write_baseline_path << '\n';
  }

  std::cout << "\n" << records.size() << " scenarios, " << (records.size() - failed)
            << " passed, " << failed << " failed";
  if (!baseline_path.empty()) std::cout << ", " << regressed << " regressed vs baseline";
  std::cout << '\n';
  return (failed == 0 && regressed == 0) ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    usage();
    return 2;
  }
  const std::string command = argv[1];
  const std::vector<std::string> args(argv + 2, argv + argc);
  try {
    if (command == "list") return cmdList(args);
    if (command == "run") return cmdRun(args);
    if (command == "suite") return cmdSuite(args);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 3;
  }
  usage();
  return 2;
}
