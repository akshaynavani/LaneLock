#pragma once

#include <algorithm>
#include <cmath>

namespace lanelock {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.81;

inline double clamp(double value, double lo, double hi) { return std::min(std::max(value, lo), hi); }

inline double sign(double value) { return (value > 0.0) - (value < 0.0); }

inline double deg2rad(double deg) { return deg * kPi / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / kPi; }

/// Wraps an angle to (-pi, pi].
inline double wrapAngle(double angle) {
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) angle += 2.0 * kPi;
  return angle - kPi;
}

/// Limits how far `target` may move away from `previous` in one step of length `dt`.
inline double rateLimit(double target, double previous, double max_rate, double dt) {
  const double max_delta = max_rate * dt;
  return previous + clamp(target - previous, -max_delta, max_delta);
}

/// Discrete first-order low-pass filter: y += alpha * (u - y), alpha = dt / (tau + dt).
class LowPassFilter {
 public:
  explicit LowPassFilter(double time_constant = 0.0) : tau_(time_constant) {}

  double update(double input, double dt) {
    if (!initialized_ || tau_ <= 0.0) {
      state_ = input;
      initialized_ = true;
      return state_;
    }
    const double alpha = dt / (tau_ + dt);
    state_ += alpha * (input - state_);
    return state_;
  }

  void reset() { initialized_ = false; state_ = 0.0; }
  void reset(double value) { initialized_ = true; state_ = value; }
  double value() const { return state_; }
  double timeConstant() const { return tau_; }

 private:
  double tau_;
  double state_ = 0.0;
  bool initialized_ = false;
};

}  // namespace lanelock
