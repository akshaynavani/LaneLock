#include "lanelock/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace lanelock {

MetricsAccumulator::MetricsAccumulator(double lane_width, double vehicle_width, double settle_band)
    : half_lane_(0.5 * lane_width), half_vehicle_(0.5 * vehicle_width), settle_band_(settle_band) {}

void MetricsAccumulator::add(double time, double dt, const VehicleState& state, double steer,
                             double steer_rate, ControllerMode mode) {
  const double abs_ey = std::abs(state.e_y);
  ++m_.samples;
  m_.duration = time;
  m_.max_abs_lateral_error = std::max(m_.max_abs_lateral_error, abs_ey);
  m_.max_abs_heading_error = std::max(m_.max_abs_heading_error, std::abs(state.e_psi));
  m_.max_abs_steer = std::max(m_.max_abs_steer, std::abs(steer));
  m_.max_abs_steer_rate = std::max(m_.max_abs_steer_rate, std::abs(steer_rate));
  m_.max_abs_lateral_accel = std::max(m_.max_abs_lateral_accel, std::abs(state.lateral_accel));
  sum_sq_error_ += state.e_y * state.e_y;

  // Distance from the outer tyre edge to the lane marking (negative = crossed the line).
  const double margin = half_lane_ - (abs_ey + half_vehicle_);
  m_.min_boundary_margin = std::min(m_.min_boundary_margin, margin);
  if (margin < 0.0 && !m_.lane_departure) {
    m_.lane_departure = true;
    m_.first_departure_time = time;
  }

  if (abs_ey > settle_band_) m_.settling_time = time;
  m_.final_lateral_error = abs_ey;

  if (mode == ControllerMode::Holding) m_.time_holding += dt;
  if (mode == ControllerMode::Unavailable) m_.time_unavailable += dt;
}

Metrics MetricsAccumulator::finish() const {
  Metrics out = m_;
  out.rms_lateral_error = m_.samples > 0 ? std::sqrt(sum_sq_error_ / m_.samples) : 0.0;
  return out;
}

std::string Verdict::summary() const {
  if (passed) return "PASS";
  std::ostringstream os;
  os << "FAIL:";
  for (const auto& v : violations) os << ' ' << v << ';';
  return os.str();
}

Verdict evaluate(const Metrics& m, const SafetyEnvelope& env) {
  Verdict v;
  auto fail = [&](const std::string& what, double value, double limit) {
    std::ostringstream os;
    os << what << '=' << value << " (limit " << limit << ')';
    v.violations.push_back(os.str());
    v.passed = false;
  };
  if (env.forbid_lane_departure && m.lane_departure) {
    std::ostringstream os;
    os << "lane_departure at t=" << m.first_departure_time << 's';
    v.violations.push_back(os.str());
    v.passed = false;
  }
  if (m.max_abs_lateral_error > env.max_abs_lateral_error)
    fail("max_abs_lateral_error", m.max_abs_lateral_error, env.max_abs_lateral_error);
  if (m.max_abs_lateral_accel > env.max_abs_lateral_accel)
    fail("max_abs_lateral_accel", m.max_abs_lateral_accel, env.max_abs_lateral_accel);
  if (m.final_lateral_error > env.max_final_lateral_error)
    fail("final_lateral_error", m.final_lateral_error, env.max_final_lateral_error);
  if (m.rms_lateral_error > env.max_rms_lateral_error)
    fail("rms_lateral_error", m.rms_lateral_error, env.max_rms_lateral_error);
  if (m.time_unavailable > env.max_time_unavailable)
    fail("time_unavailable", m.time_unavailable, env.max_time_unavailable);
  return v;
}

}  // namespace lanelock
