#pragma once

#include <cstdint>
#include <deque>

#include "lanelock/noise.hpp"
#include "lanelock/road.hpp"
#include "lanelock/vehicle.hpp"

namespace lanelock {

/// What a lane-detection camera reports about the ego vehicle relative to its lane.
struct LaneMeasurement {
  double timestamp = 0.0;       // capture time, s
  double lateral_offset = 0.0;  // m, + = vehicle left of lane centre
  double heading_error = 0.0;   // rad, + = vehicle yawed left of lane tangent
  double curvature = 0.0;       // 1/m, previewed lane curvature
  bool valid = false;
};

struct LaneCameraConfig {
  double rate_hz = 20.0;            // frame rate
  double latency = 0.05;            // s, capture-to-output delay
  double preview_distance = 20.0;   // m, horizon over which curvature is estimated
  double dropout_probability = 0.0;  // per frame
  int dropout_burst_frames = 1;     // frames lost once a dropout starts
  /// The detector loses the lane when the vehicle is this far outside the lane boundary.
  double max_detect_offset = 3.0;   // m
  /// Deterministic blackout window (e.g. tunnel exit glare, faded markings); duration 0 = off.
  double blackout_start = 0.0;      // s
  double blackout_duration = 0.0;   // s
  NoiseParams offset_noise{0.03, 0.0, 0.0, 0.0};
  NoiseParams heading_noise{0.002, 0.0, 0.0, 0.0};
  NoiseParams curvature_noise{1e-4, 0.0, 0.0, 0.0};

  static LaneCameraConfig ideal();
};

/// Camera lane detector with frame rate, latency, noise and dropouts.
class LaneCamera {
 public:
  LaneCamera(LaneCameraConfig config, std::uint64_t seed);

  /// Called every simulation step. Captures a new frame when one is due and returns the most
  /// recent frame whose latency has elapsed (invalid until the first frame is available).
  LaneMeasurement update(double time, const VehicleState& truth, const Road& road);

  const LaneCameraConfig& config() const { return config_; }
  int framesCaptured() const { return frames_captured_; }
  int framesDropped() const { return frames_dropped_; }

 private:
  LaneMeasurement capture(double time, double dt, const VehicleState& truth, const Road& road);

  LaneCameraConfig config_;
  NoiseModel offset_noise_;
  NoiseModel heading_noise_;
  NoiseModel curvature_noise_;
  DropoutModel dropout_;
  std::deque<LaneMeasurement> pipeline_;  // frames in flight (latency buffer)
  LaneMeasurement output_;
  double next_capture_time_ = 0.0;
  double last_capture_time_ = 0.0;
  int frames_captured_ = 0;
  int frames_dropped_ = 0;
};

/// Generic noisy scalar sensor (wheel speed, gyro yaw rate).
class ScalarSensor {
 public:
  ScalarSensor(NoiseParams params, std::uint64_t seed) : noise_(params, seed) {}
  double read(double true_value, double dt) { return noise_.apply(true_value, dt); }

 private:
  NoiseModel noise_;
};

struct SensorSuiteConfig {
  LaneCameraConfig camera;
  NoiseParams speed_noise{0.05, 0.0, 0.0, 0.0};
  NoiseParams yaw_rate_noise{0.002, 0.0, 0.0005, 0.0};

  static SensorSuiteConfig ideal();
};

/// Everything the controller is allowed to know about the world in one step.
struct SensorFrame {
  double time = 0.0;
  LaneMeasurement lane;
  double speed = 0.0;
  double yaw_rate = 0.0;
};

class SensorSuite {
 public:
  SensorSuite(const SensorSuiteConfig& config, std::uint64_t seed);

  SensorFrame update(double time, double dt, const VehicleState& truth, const Road& road);
  const LaneCamera& camera() const { return camera_; }

 private:
  LaneCamera camera_;
  ScalarSensor speed_;
  ScalarSensor yaw_rate_;
};

}  // namespace lanelock
