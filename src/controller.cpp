#include "lanelock/controller.hpp"

#include <algorithm>
#include <cmath>

namespace lanelock {

const char* toString(ControllerMode mode) {
  switch (mode) {
    case ControllerMode::Active:
      return "active";
    case ControllerMode::Holding:
      return "holding";
    case ControllerMode::Unavailable:
      return "unavailable";
  }
  return "unknown";
}

PidLaneKeepingController::PidLaneKeepingController(PidLaneKeepingConfig config,
                                                   VehicleParams vehicle)
    : config_(config), vehicle_(vehicle), curvature_filter_(config.curvature_filter_tau),
      curvature_rate_filter_(config.curvature_filter_tau) {
  PidLimits limits;
  limits.output_min = -config_.max_steer;
  limits.output_max = config_.max_steer;
  limits.integral_limit = config_.integral_limit;
  limits.back_calculation_gain = config_.back_calculation_gain;
  pid_ = Pid(scheduledGains(20.0), limits, config_.derivative_filter_tau);
}

void PidLaneKeepingController::reset() {
  pid_.reset();
  mode_ = ControllerMode::Unavailable;
  debug_ = {};
  have_estimate_ = false;
  last_frame_timestamp_ = -1.0;
  last_valid_time_ = -1e9;
  est_offset_ = est_heading_ = est_curvature_ = 0.0;
  curvature_filter_.reset();
  curvature_rate_filter_.reset();
}

double PidLaneKeepingController::curvatureAhead(double v, double time_ahead) const {
  // The low-pass filters delay the clothoid by ~tau; look that much further ahead to undo it.
  // Extrapolation is limited to the look-ahead range so a stale clothoid can't run away.
  const double distance =
      std::min(v * (std::max(time_ahead, 0.0) + config_.curvature_filter_tau), config_.max_lookahead);
  return curvature_filter_.value() + curvature_rate_filter_.value() * distance;
}

PidGains PidLaneKeepingController::scheduledGains(double v) const {
  const double vs = std::max(v, config_.min_schedule_speed);
  const double wn = config_.natural_frequency;
  PidGains g;
  g.kp = vehicle_.wheelbase() * wn * wn / (vs * vs);
  g.ki = g.kp * config_.ki_ratio;
  g.kd = g.kp * config_.kd_time;
  return g;
}

double PidLaneKeepingController::lookaheadDistance(double v) const {
  const double t_la = 2.0 * config_.damping_ratio / config_.natural_frequency;
  return clamp(t_la * std::max(v, 0.0), config_.min_lookahead, config_.max_lookahead);
}

double PidLaneKeepingController::steadyStateSideslip(double kappa, double v) const {
  const double L = vehicle_.wheelbase();
  return (vehicle_.lr - vehicle_.mass * vehicle_.lf * v * v /
                            (vehicle_.cornering_stiffness_rear * L)) *
         kappa;
}

double PidLaneKeepingController::feedforwardSteer(double kappa, double v) const {
  if (!config_.use_feedforward) return 0.0;
  double gain = vehicle_.wheelbase();
  if (config_.compensate_understeer) gain += vehicle_.understeerGradient() * v * v;
  return gain * kappa;
}

void PidLaneKeepingController::updateEstimate(const SensorFrame& frame, double dt) {
  const double v = std::max(frame.speed, 0.0);

  // Dead-reckoning of the lane-relative pose using gyro yaw rate and the last curvature:
  //   e_psi_dot = r - v kappa,   e_y_dot = v sin(e_psi) + vy,   vy ~= v * beta_ss(kappa)
  auto propagate = [&](double horizon) {
    if (horizon <= 0.0) return;
    const double vy = config_.compensate_understeer ? v * steadyStateSideslip(est_curvature_, v) : 0.0;
    est_heading_ = wrapAngle(est_heading_ + (frame.yaw_rate - v * est_curvature_) * horizon);
    est_offset_ += (v * std::sin(est_heading_) + vy) * horizon;
  };

  const LaneMeasurement& lane = frame.lane;
  const bool new_frame = lane.valid && lane.timestamp > last_frame_timestamp_;
  if (have_estimate_) {
    est_curvature_ = curvatureAhead(v, frame.time - last_frame_timestamp_);
    propagate(dt);  // predict to the current time
  }
  if (!new_frame) return;

  // The frame describes the vehicle at capture time; bring it forward to now using the same
  // kinematics so it can be compared with the prediction.
  const double frame_dt = std::max(lane.timestamp - last_frame_timestamp_, dt);
  // After a long outage the dead-reckoned prediction is worthless: re-initialise from the camera.
  const bool trust_prediction = have_estimate_ && (lane.timestamp - last_valid_time_) <= config_.hold_timeout;
  last_frame_timestamp_ = lane.timestamp;
  last_valid_time_ = lane.timestamp;
  curvature_filter_.update(lane.curvature, frame_dt);
  curvature_rate_filter_.update(lane.curvature_rate, frame_dt);

  const double pred_offset = est_offset_;
  const double pred_heading = est_heading_;
  est_offset_ = lane.lateral_offset;
  est_heading_ = lane.heading_error;
  est_curvature_ = curvatureAhead(v, frame.time - lane.timestamp);
  if (config_.use_latency_compensation) propagate(frame.time - lane.timestamp);

  if (trust_prediction) {
    // Correction step of a fixed-gain observer: blend measurement with the prediction.
    est_offset_ = pred_offset + config_.offset_correction_gain * (est_offset_ - pred_offset);
    est_heading_ = pred_heading + config_.heading_correction_gain * (est_heading_ - pred_heading);
  }
  have_estimate_ = true;
}

double PidLaneKeepingController::computeSteering(const SensorFrame& frame, double dt) {
  updateEstimate(frame, dt);
  const double v = std::max(frame.speed, 0.0);

  // Mode logic.
  const double age = frame.time - last_valid_time_;
  const ControllerMode previous = mode_;
  if (!have_estimate_ || age > config_.hold_timeout) {
    mode_ = ControllerMode::Unavailable;
  } else if (age > config_.stale_after) {
    mode_ = ControllerMode::Holding;
  } else {
    mode_ = ControllerMode::Active;
  }
  if (previous == ControllerMode::Unavailable && mode_ != ControllerMode::Unavailable) {
    pid_.reset();  // bumpless re-engagement: no stale integral or derivative history
  }

  const double ff_curvature =
      curvatureAhead(v, frame.time - last_frame_timestamp_ + config_.feedforward_preview_time);
  const double ff = have_estimate_ ? feedforwardSteer(ff_curvature, v) : 0.0;
  debug_.est_offset = est_offset_;
  debug_.est_heading = est_heading_;
  debug_.est_curvature = est_curvature_;
  debug_.ff_curvature = ff_curvature;
  debug_.feedforward = ff;

  if (mode_ == ControllerMode::Unavailable) {
    pid_.setIntegratorFrozen(true);
    debug_.feedback = 0.0;
    debug_.lookahead_error = 0.0;
    debug_.pid = {};
    return clamp(ff, -config_.max_steer, config_.max_steer);
  }

  // Heading error relative to the attitude a perfectly tracking car would have (removes the
  // expected steady-state sideslip so cornering doesn't look like a heading error).
  double heading = est_heading_;
  if (config_.compensate_understeer) heading += steadyStateSideslip(est_curvature_, v);

  const double lookahead_error = est_offset_ + lookaheadDistance(v) * heading;
  pid_.setGains(scheduledGains(v));
  // Hold the integrator while dead-reckoning: the estimate drifts and shouldn't be integrated.
  pid_.setIntegratorFrozen(mode_ != ControllerMode::Active);
  const double fb = pid_.update(0.0, lookahead_error, dt);

  debug_.lookahead_error = lookahead_error;
  debug_.feedback = fb;
  debug_.pid = pid_.lastTerms();
  return clamp(ff + fb, -config_.max_steer, config_.max_steer);
}

}  // namespace lanelock
