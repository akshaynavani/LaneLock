#pragma once

#include <limits>
#include <string>
#include <vector>

#include "lanelock/controller.hpp"
#include "lanelock/vehicle.hpp"

namespace lanelock {

/// Aggregate results of one closed-loop run.
struct Metrics {
  double duration = 0.0;
  double max_abs_lateral_error = 0.0;   // m
  double rms_lateral_error = 0.0;       // m
  double max_abs_heading_error = 0.0;   // rad
  double max_abs_steer = 0.0;           // rad, road wheel
  double max_abs_steer_rate = 0.0;      // rad/s
  double max_abs_lateral_accel = 0.0;   // m/s^2
  double min_boundary_margin = std::numeric_limits<double>::infinity();  // m, tyre to line
  double final_lateral_error = 0.0;     // m, |e_y| at the end of the run
  double settling_time = 0.0;           // s, last time |e_y| left the settle band
  double time_holding = 0.0;            // s spent dead-reckoning
  double time_unavailable = 0.0;        // s with the controller unavailable
  bool lane_departure = false;
  double first_departure_time = -1.0;   // s, -1 if none
  int samples = 0;
};

/// Streaming metric accumulator; feed one sample per simulation step.
class MetricsAccumulator {
 public:
  MetricsAccumulator(double lane_width, double vehicle_width, double settle_band = 0.2);

  void add(double time, double dt, const VehicleState& state, double steer, double steer_rate,
           ControllerMode mode);
  Metrics finish() const;

 private:
  double half_lane_;
  double half_vehicle_;
  double settle_band_;
  double sum_sq_error_ = 0.0;
  Metrics m_;
};

/// Pass/fail criteria for a scenario.
struct SafetyEnvelope {
  double max_abs_lateral_error = 0.8;  // m
  bool forbid_lane_departure = true;
  double max_abs_lateral_accel = 5.0;  // m/s^2
  double max_final_lateral_error = 0.4;  // m
  double max_rms_lateral_error = 0.5;    // m
  double max_time_unavailable = std::numeric_limits<double>::infinity();  // s
};

struct Verdict {
  bool passed = true;
  std::vector<std::string> violations;

  std::string summary() const;
};

Verdict evaluate(const Metrics& metrics, const SafetyEnvelope& envelope);

}  // namespace lanelock
