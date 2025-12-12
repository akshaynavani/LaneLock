#include "lanelock/sensors.hpp"

#include <cmath>

namespace lanelock {

namespace {
// Stream ids for deriving independent RNG seeds per noise source.
enum Stream : std::uint64_t {
  kOffset = 1,
  kHeading = 2,
  kCurvature = 3,
  kDropout = 4,
  kSpeed = 5,
  kYawRate = 6,
  kCamera = 7,
};
constexpr double kTimeEps = 1e-9;
}  // namespace

LaneCameraConfig LaneCameraConfig::ideal() {
  LaneCameraConfig c;
  c.latency = 0.0;
  c.rate_hz = 100.0;
  c.offset_noise = NoiseParams::none();
  c.heading_noise = NoiseParams::none();
  c.curvature_noise = NoiseParams::none();
  return c;
}

SensorSuiteConfig SensorSuiteConfig::ideal() {
  SensorSuiteConfig c;
  c.camera = LaneCameraConfig::ideal();
  c.speed_noise = NoiseParams::none();
  c.yaw_rate_noise = NoiseParams::none();
  return c;
}

// ---------------------------------------------------------------------------- LaneCamera

LaneCamera::LaneCamera(LaneCameraConfig config, std::uint64_t seed)
    : config_(config),
      offset_noise_(config.offset_noise, deriveSeed(seed, kOffset)),
      heading_noise_(config.heading_noise, deriveSeed(seed, kHeading)),
      curvature_noise_(config.curvature_noise, deriveSeed(seed, kCurvature)),
      dropout_(config.dropout_probability, config.dropout_burst_frames, deriveSeed(seed, kDropout)) {}

LaneMeasurement LaneCamera::capture(double time, double dt, const VehicleState& truth,
                                    const Road& road) {
  LaneMeasurement m;
  m.timestamp = time;
  ++frames_captured_;

  // Noise processes evolve whether or not the frame survives.
  m.lateral_offset = offset_noise_.apply(truth.e_y, dt);
  m.heading_error = heading_noise_.apply(truth.e_psi, dt);
  m.curvature = curvature_noise_.apply(road.previewCurvature(truth.s, config_.preview_distance), dt);

  const double boundary = 0.5 * road.laneWidth();
  const bool lane_visible = std::abs(truth.e_y) < boundary + config_.max_detect_offset;
  const bool dropped = dropout_.dropped();
  const bool blackout = config_.blackout_duration > 0.0 && time >= config_.blackout_start &&
                        time < config_.blackout_start + config_.blackout_duration;
  m.valid = lane_visible && !dropped && !blackout;
  if (!m.valid) ++frames_dropped_;
  return m;
}

LaneMeasurement LaneCamera::update(double time, const VehicleState& truth, const Road& road) {
  const double period = config_.rate_hz > 0.0 ? 1.0 / config_.rate_hz : 0.0;
  if (time + kTimeEps >= next_capture_time_) {
    const double dt = frames_captured_ == 0 ? period : time - last_capture_time_;
    pipeline_.push_back(capture(time, dt, truth, road));
    last_capture_time_ = time;
    // Advance on a fixed grid so frame timing doesn't drift; if the camera is faster than the
    // simulation step, it simply captures once per step.
    next_capture_time_ += period;
    if (next_capture_time_ <= time) next_capture_time_ = time + period;
  }
  // Release every frame whose latency has elapsed; the newest released frame is the output.
  while (!pipeline_.empty() && pipeline_.front().timestamp + config_.latency <= time + kTimeEps) {
    output_ = pipeline_.front();
    pipeline_.pop_front();
  }
  return output_;
}

// ---------------------------------------------------------------------------- SensorSuite

SensorSuite::SensorSuite(const SensorSuiteConfig& config, std::uint64_t seed)
    : camera_(config.camera, deriveSeed(seed, kCamera)),
      speed_(config.speed_noise, deriveSeed(seed, kSpeed)),
      yaw_rate_(config.yaw_rate_noise, deriveSeed(seed, kYawRate)) {}

SensorFrame SensorSuite::update(double time, double dt, const VehicleState& truth,
                                const Road& road) {
  SensorFrame frame;
  frame.time = time;
  frame.lane = camera_.update(time, truth, road);
  frame.speed = speed_.read(truth.vx, dt);
  frame.yaw_rate = yaw_rate_.read(truth.yaw_rate, dt);
  return frame;
}

}  // namespace lanelock
