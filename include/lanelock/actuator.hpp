#pragma once

#include <deque>

namespace lanelock {

/// Electric power steering (EPS) model, all angles at the road wheel.
struct SteeringActuatorConfig {
  double time_constant = 0.08;  // s, first-order motor/rack response
  double max_rate = 0.5;        // rad/s, rack rate limit
  double max_angle = 0.4;       // rad, authority limit granted to the LKA function
  double dead_time = 0.02;      // s, bus + ECU transport delay

  static SteeringActuatorConfig ideal();
};

/// command -> dead time -> rate limit -> first-order lag -> saturation
class SteeringActuator {
 public:
  explicit SteeringActuator(SteeringActuatorConfig config = {});

  /// Feeds a new command and returns the road-wheel angle after `dt`.
  double update(double command, double dt);

  double angle() const { return angle_; }
  double lastRate() const { return last_rate_; }
  bool saturated() const { return saturated_; }
  const SteeringActuatorConfig& config() const { return config_; }
  void reset(double angle = 0.0);

 private:
  SteeringActuatorConfig config_;
  std::deque<double> delay_line_;
  double angle_ = 0.0;
  double last_rate_ = 0.0;
  bool saturated_ = false;
};

}  // namespace lanelock
