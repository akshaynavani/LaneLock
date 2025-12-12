#pragma once

#include <functional>
#include <memory>

#include "lanelock/actuator.hpp"
#include "lanelock/controller.hpp"
#include "lanelock/metrics.hpp"
#include "lanelock/scenario.hpp"
#include "lanelock/sensors.hpp"
#include "lanelock/vehicle.hpp"

namespace lanelock {

/// One row of the per-step trace.
struct TraceSample {
  double time = 0.0;
  double s = 0.0;
  double x = 0.0;  // world position (only filled when world pose tracing is enabled)
  double y = 0.0;
  double e_y = 0.0;
  double e_psi = 0.0;
  double road_curvature = 0.0;
  double lateral_accel = 0.0;
  double steer_command = 0.0;
  double steer_actual = 0.0;
  double meas_offset = 0.0;
  bool meas_valid = false;
  double disturbance = 0.0;
  ControllerMode mode = ControllerMode::Unavailable;
  ControllerDebug controller;
};

using TraceCallback = std::function<void(const TraceSample&)>;

struct SimulationResult {
  Metrics metrics;
  Verdict verdict;
  int camera_frames = 0;
  int camera_frames_dropped = 0;
};

/// Closed-loop simulator: road + vehicle + sensors + controller + actuator.
///
/// Per step (fixed dt):
///   1. sensors sample the true state            -> SensorFrame
///   2. controller computes a steering request   -> command
///   3. actuator applies dead time, lag, limits  -> road-wheel angle
///   4. vehicle model integrates with disturbance (RK4)
///   5. metrics / trace are updated
class Simulator {
 public:
  /// Uses the scenario's PID controller configuration.
  explicit Simulator(const Scenario& scenario);
  /// Injects a custom controller (e.g. a new algorithm under test).
  Simulator(const Scenario& scenario, std::unique_ptr<LateralController> controller);

  /// Runs to completion. `trace` (optional) receives every step.
  SimulationResult run(const TraceCallback& trace = nullptr, bool world_pose = false);

  /// Single-step API for interactive use and fine-grained tests.
  void step();
  bool done() const;
  double time() const { return time_; }
  const VehicleState& state() const { return state_; }
  const LateralController& controller() const { return *controller_; }
  const TraceSample& lastSample() const { return last_; }

 private:
  void fillWorldPose(TraceSample& sample) const;

  Scenario scenario_;
  std::unique_ptr<VehicleModel> model_;
  SensorSuite sensors_;
  std::unique_ptr<LateralController> controller_;
  SteeringActuator actuator_;
  MetricsAccumulator metrics_;
  VehicleState state_;
  double time_ = 0.0;
  long step_count_ = 0;
  long total_steps_ = 0;
  TraceSample last_;
};

/// Convenience: run a scenario with its default controller and return the result.
SimulationResult runScenario(const Scenario& scenario);

}  // namespace lanelock
