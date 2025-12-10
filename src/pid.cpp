#include "lanelock/pid.hpp"

#include <cmath>

namespace lanelock {

Pid::Pid(PidGains gains, PidLimits limits, double derivative_filter_tau)
    : gains_(gains), limits_(limits), derivative_filter_(derivative_filter_tau) {}

void Pid::reset() {
  integral_ = 0.0;
  prev_measurement_ = 0.0;
  has_prev_ = false;
  derivative_filter_.reset();
  terms_ = {};
}

double Pid::update(double setpoint, double measurement, double dt) {
  if (dt <= 0.0) return terms_.output;
  const double error = setpoint - measurement;

  // Derivative on measurement, low-pass filtered.
  double d_meas = 0.0;
  if (has_prev_) d_meas = (measurement - prev_measurement_) / dt;
  prev_measurement_ = measurement;
  has_prev_ = true;
  const double d_filtered = derivative_filter_.update(d_meas, dt);

  terms_.p = gains_.kp * error;
  terms_.d = -gains_.kd * d_filtered;

  // Compute the output with the current integral, then update the integral with anti-windup.
  const double unsat = terms_.p + integral_ + terms_.d;
  const double out = clamp(unsat, limits_.output_min, limits_.output_max);

  if (!integrator_frozen_) {
    double di = gains_.ki * error * dt;
    if (limits_.back_calculation_gain > 0.0) {
      di += limits_.back_calculation_gain * (out - unsat) * dt;
    } else if (out != unsat && sign(error) == sign(unsat)) {
      di = 0.0;  // conditional integration: don't wind further into saturation
    }
    integral_ = clamp(integral_ + di, -limits_.integral_limit, limits_.integral_limit);
  }

  terms_.i = integral_;
  terms_.unsaturated = unsat;
  terms_.output = out;
  return out;
}

}  // namespace lanelock
