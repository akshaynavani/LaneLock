#include "lanelock/io.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace lanelock {

// ---------------------------------------------------------------------------- trace

CsvTraceWriter::CsvTraceWriter(std::ostream& out) : out_(out) {
  out_ << "time,s,x,y,e_y,e_psi,road_curvature,lateral_accel,steer_command,steer_actual,"
          "meas_offset,meas_valid,disturbance,mode,est_offset,est_heading,est_curvature,"
          "lookahead_error,feedforward,feedback,pid_p,pid_i,pid_d\n";
  out_ << std::setprecision(6);
}

void CsvTraceWriter::write(const TraceSample& t) {
  const auto& c = t.controller;
  out_ << t.time << ',' << t.s << ',' << t.x << ',' << t.y << ',' << t.e_y << ',' << t.e_psi
       << ',' << t.road_curvature << ',' << t.lateral_accel << ',' << t.steer_command << ','
       << t.steer_actual << ',' << t.meas_offset << ',' << (t.meas_valid ? 1 : 0) << ','
       << t.disturbance << ',' << toString(t.mode) << ',' << c.est_offset << ','
       << c.est_heading << ',' << c.est_curvature << ',' << c.lookahead_error << ','
       << c.feedforward << ',' << c.feedback << ',' << c.pid.p << ',' << c.pid.i << ','
       << c.pid.d << '\n';
}

// ---------------------------------------------------------------------------- summary

void writeSummaryCsv(std::ostream& out, const std::vector<ScenarioRecord>& records) {
  out << "scenario,passed,max_abs_lateral_error,rms_lateral_error,max_abs_heading_error,"
         "max_abs_steer,max_abs_steer_rate,max_abs_lateral_accel,min_boundary_margin,"
         "final_lateral_error,settling_time,time_holding,time_unavailable,lane_departure,"
         "verdict\n";
  out << std::setprecision(6);
  for (const auto& r : records) {
    const auto& m = r.metrics;
    out << r.name << ',' << (r.passed ? 1 : 0) << ',' << m.max_abs_lateral_error << ','
        << m.rms_lateral_error << ',' << m.max_abs_heading_error << ',' << m.max_abs_steer << ','
        << m.max_abs_steer_rate << ',' << m.max_abs_lateral_accel << ','
        << m.min_boundary_margin << ',' << m.final_lateral_error << ',' << m.settling_time << ','
        << m.time_holding << ',' << m.time_unavailable << ',' << (m.lane_departure ? 1 : 0)
        << ",\"" << r.verdict << "\"\n";
  }
}

// ---------------------------------------------------------------------------- baseline

void writeBaseline(std::ostream& out, const std::vector<ScenarioRecord>& records) {
  out << "scenario,max_abs_lateral_error,rms_lateral_error,max_abs_lateral_accel\n";
  out << std::setprecision(9);
  for (const auto& r : records) {
    out << r.name << ',' << r.metrics.max_abs_lateral_error << ','
        << r.metrics.rms_lateral_error << ',' << r.metrics.max_abs_lateral_accel << '\n';
  }
}

Baseline readBaseline(std::istream& in) {
  Baseline baseline;
  std::string line;
  bool header = true;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    if (header) {
      header = false;
      continue;
    }
    std::istringstream row(line);
    std::string name;
    std::string field;
    Metrics m;
    if (!std::getline(row, name, ',')) continue;
    try {
      std::getline(row, field, ',');
      m.max_abs_lateral_error = std::stod(field);
      std::getline(row, field, ',');
      m.rms_lateral_error = std::stod(field);
      std::getline(row, field, ',');
      m.max_abs_lateral_accel = std::stod(field);
    } catch (const std::exception&) {
      continue;  // skip malformed rows
    }
    baseline[name] = m;
  }
  return baseline;
}

bool loadBaselineFile(const std::string& path, Baseline& baseline) {
  std::ifstream in(path);
  if (!in) return false;
  baseline = readBaseline(in);
  return true;
}

std::vector<std::string> compareToBaseline(const Metrics& current, const Metrics& baseline,
                                           const BaselineTolerance& tol) {
  std::vector<std::string> regressions;
  auto check = [&](const char* what, double now, double before, double abs_tol, double rel_tol) {
    const double allowed = before + std::max(abs_tol, rel_tol * before);
    if (now > allowed) {
      std::ostringstream os;
      os << what << " regressed: " << now << " > baseline " << before << " (+tol -> " << allowed
         << ')';
      regressions.push_back(os.str());
    }
  };
  check("max_abs_lateral_error", current.max_abs_lateral_error, baseline.max_abs_lateral_error,
        tol.lateral_error_abs, tol.lateral_error_rel);
  check("rms_lateral_error", current.rms_lateral_error, baseline.rms_lateral_error,
        tol.lateral_error_abs, tol.lateral_error_rel);
  check("max_abs_lateral_accel", current.max_abs_lateral_accel, baseline.max_abs_lateral_accel,
        tol.lateral_accel_abs, 0.0);
  return regressions;
}

}  // namespace lanelock
