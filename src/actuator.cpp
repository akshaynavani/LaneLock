#include "lanelock/actuator.hpp"

#include <cmath>

#include "lanelock/math.hpp"

namespace lanelock {

SteeringActuatorConfig SteeringActuatorConfig::ideal() {
  SteeringActuatorConfig c;
  c.time_constant = 0.0;
  c.max_rate = 1e6;
  c.max_angle = 1.0;
  c.dead_time = 0.0;
  return c;
}

SteeringActuator::SteeringActuator(SteeringActuatorConfig config) : config_(config) {}

void SteeringActuator::reset(double angle) {
  delay_line_.clear();
  angle_ = angle;
  last_rate_ = 0.0;
  saturated_ = false;
}

double SteeringActuator::update(double command, double dt) {
  if (dt <= 0.0) return angle_;

  // Transport delay: hold commands for round(dead_time / dt) steps.
  const auto delay_steps = static_cast<std::size_t>(std::lround(config_.dead_time / dt));
  delay_line_.push_back(command);
  double delayed = 0.0;
  if (delay_line_.size() > delay_steps) {
    delayed = delay_line_.front();
    delay_line_.pop_front();
  } else {
    delayed = angle_;  // pipeline still filling: hold the current angle
  }

  const double limited_cmd = clamp(delayed, -config_.max_angle, config_.max_angle);
  saturated_ = std::abs(delayed) > config_.max_angle;

  // First-order lag towards the command, then rate limiting.
  double target = limited_cmd;
  if (config_.time_constant > 0.0) {
    target = angle_ + dt / (config_.time_constant + dt) * (limited_cmd - angle_);
  }
  const double next = rateLimit(target, angle_, config_.max_rate, dt);
  last_rate_ = (next - angle_) / dt;
  angle_ = next;
  return angle_;
}

}  // namespace lanelock
