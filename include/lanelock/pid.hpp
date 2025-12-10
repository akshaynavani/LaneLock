#pragma once

#include <limits>

#include "lanelock/math.hpp"

namespace lanelock {

struct PidGains {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
};

struct PidLimits {
  double output_min = -std::numeric_limits<double>::infinity();
  double output_max = std::numeric_limits<double>::infinity();
  double integral_limit = std::numeric_limits<double>::infinity();  // on the I contribution
  double back_calculation_gain = 0.0;  // 1/s, 0 disables back-calculation anti-windup
};

/// Contribution of each term on the last update (useful for logging / tests).
struct PidTerms {
  double p = 0.0;
  double i = 0.0;
  double d = 0.0;
  double unsaturated = 0.0;
  double output = 0.0;
};

/// Discrete PID controller with
///  - derivative on measurement (no derivative kick on set-point changes),
///  - first-order low-pass filtered derivative,
///  - integrator clamping plus optional back-calculation anti-windup,
///  - output saturation.
/// Gains may be changed between updates (gain scheduling) without bumping the integral state,
/// because the integrator stores the accumulated I contribution rather than the raw error sum.
class Pid {
 public:
  Pid() = default;
  Pid(PidGains gains, PidLimits limits, double derivative_filter_tau = 0.0);

  double update(double setpoint, double measurement, double dt);

  void setGains(const PidGains& gains) { gains_ = gains; }
  const PidGains& gains() const { return gains_; }
  const PidLimits& limits() const { return limits_; }
  const PidTerms& lastTerms() const { return terms_; }
  double integral() const { return integral_; }

  /// Freezes the integrator (e.g. while the controller is not in control).
  void setIntegratorFrozen(bool frozen) { integrator_frozen_ = frozen; }
  void reset();

 private:
  PidGains gains_;
  PidLimits limits_;
  LowPassFilter derivative_filter_;
  double integral_ = 0.0;
  double prev_measurement_ = 0.0;
  bool has_prev_ = false;
  bool integrator_frozen_ = false;
  PidTerms terms_;
};

}  // namespace lanelock
