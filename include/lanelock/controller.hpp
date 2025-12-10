#pragma once

#include <string>

#include "lanelock/pid.hpp"
#include "lanelock/sensors.hpp"
#include "lanelock/vehicle.hpp"

namespace lanelock {

enum class ControllerMode {
  Active,       // fresh lane measurements, closed-loop control
  Holding,      // lane lost briefly: steering on a dead-reckoned estimate
  Unavailable,  // lane lost longer than the hold timeout: feed-forward only, ready for handover
};

const char* toString(ControllerMode mode);

/// Debug signals exposed by a controller for tracing and tests.
struct ControllerDebug {
  double est_offset = 0.0;
  double est_heading = 0.0;
  double est_curvature = 0.0;
  double lookahead_error = 0.0;
  double feedforward = 0.0;
  double feedback = 0.0;
  PidTerms pid;
};

/// Interface every lateral controller under test implements.
class LateralController {
 public:
  virtual ~LateralController() = default;
  /// Returns the requested road-wheel angle (rad, + = left) for this step.
  virtual double computeSteering(const SensorFrame& frame, double dt) = 0;
  virtual void reset() = 0;
  virtual std::string name() const = 0;
  virtual ControllerMode mode() const = 0;
  virtual const ControllerDebug& debug() const = 0;
};

struct PidLaneKeepingConfig {
  // Desired closed-loop behaviour of the lateral error dynamics; PID gains are derived from
  // these and re-scheduled with speed every step.
  double natural_frequency = 1.2;  // rad/s
  double damping_ratio = 0.9;
  double ki_ratio = 0.15;          // ki = kp * ki_ratio  [1/s]
  double kd_time = 0.10;           // kd = kp * kd_time   [s]
  double derivative_filter_tau = 0.1;
  double min_lookahead = 4.0;      // m
  double max_lookahead = 60.0;     // m
  double min_schedule_speed = 5.0;  // m/s, floor for gain scheduling
  double integral_limit = 0.03;    // rad of steering from the integrator
  double back_calculation_gain = 2.0;
  double max_steer = 0.4;          // rad

  bool use_feedforward = true;
  bool use_latency_compensation = true;
  /// Model-based steady-state cornering compensation: adds the understeer term K_us v^2 kappa
  /// to the feed-forward and removes the expected sideslip from the heading error. Enable when
  /// the plant has tyre slip (dynamic model / real car); disable for a kinematic plant.
  bool compensate_understeer = true;
  double hold_timeout = 1.0;       // s of dead reckoning before going Unavailable
  double stale_after = 0.25;       // s without a new valid frame before switching to Holding
};

/// PID lane-keeping controller.
///
/// Error signal: look-ahead lateral error  e_L = e_y + L_a * e_psi,  L_a = T_la * v.
/// For the kinematic bicycle with delta = -kp * e_L the error dynamics are
///   s^2 + (v^2 kp T_la / L) s + v^2 kp / L = 0
/// so choosing kp = L wn^2 / v^2 and T_la = 2 zeta / wn gives speed-independent wn and zeta.
///
/// Steering = feedback(PID on e_L) + feed-forward (L + K_us v^2) * kappa.
/// A small observer integrates yaw-rate to propagate the last lane measurement through
/// camera latency, between frames and across short dropouts.
class PidLaneKeepingController : public LateralController {
 public:
  PidLaneKeepingController(PidLaneKeepingConfig config, VehicleParams vehicle);

  double computeSteering(const SensorFrame& frame, double dt) override;
  void reset() override;
  std::string name() const override { return "pid_lka"; }
  ControllerMode mode() const override { return mode_; }
  const ControllerDebug& debug() const override { return debug_; }

  const PidLaneKeepingConfig& config() const { return config_; }
  /// Gains that would be scheduled at speed `v`.
  PidGains scheduledGains(double v) const;
  double lookaheadDistance(double v) const;
  /// Steady-state sideslip angle at the CG for curvature `kappa` at speed `v` (linear tyres).
  double steadyStateSideslip(double kappa, double v) const;
  double feedforwardSteer(double kappa, double v) const;

 private:
  void updateEstimate(const SensorFrame& frame, double dt);

  PidLaneKeepingConfig config_;
  VehicleParams vehicle_;
  Pid pid_;
  ControllerMode mode_ = ControllerMode::Unavailable;
  ControllerDebug debug_;

  bool have_estimate_ = false;
  double last_frame_timestamp_ = -1.0;
  double last_valid_time_ = -1e9;
  double est_offset_ = 0.0;
  double est_heading_ = 0.0;
  double est_curvature_ = 0.0;
};

}  // namespace lanelock
