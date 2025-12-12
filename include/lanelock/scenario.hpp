#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "lanelock/actuator.hpp"
#include "lanelock/controller.hpp"
#include "lanelock/metrics.hpp"
#include "lanelock/road.hpp"
#include "lanelock/sensors.hpp"
#include "lanelock/vehicle.hpp"

namespace lanelock {

/// Trapezoidal lateral force pulse (e.g. a crosswind gust from a bridge gap or passing truck).
struct Disturbance {
  double start = 0.0;     // s
  double ramp = 0.3;      // s, rise and fall time
  double hold = 1.0;      // s at full force
  double force = 0.0;     // N, + pushes the vehicle left

  double forceAt(double t) const;
};

/// Everything needed to reproduce one closed-loop run.
struct Scenario {
  std::string name;
  std::string category;
  std::vector<std::string> tags;

  Road road;
  VehicleModelType model = VehicleModelType::Dynamic;
  VehicleParams vehicle;
  double speed = 20.0;            // m/s, held constant
  double initial_offset = 0.0;    // m
  double initial_heading = 0.0;   // rad

  SensorSuiteConfig sensors;
  SteeringActuatorConfig actuator;
  PidLaneKeepingConfig controller;
  std::vector<Disturbance> disturbances;

  double duration = 25.0;  // s
  double dt = 0.01;        // s
  std::uint64_t seed = 1;
  SafetyEnvelope envelope;

  double disturbanceAt(double t) const;
  bool hasTag(const std::string& tag) const;
};

/// Deterministic generator of the regression scenario set.
///
/// The parametric grid crosses
///   5 speeds x 6 road geometries x 3 sensor-quality levels x 3 initial conditions x
///   2 disturbance settings = 540 scenarios,
/// and is followed by a set of named edge cases.
class ScenarioLibrary {
 public:
  static std::vector<Scenario> parametric();
  static std::vector<Scenario> edgeCases();
  static std::vector<Scenario> all();

  /// Scenarios whose name contains `filter` (empty filter = all).
  static std::vector<Scenario> filtered(const std::string& filter);
  static std::optional<Scenario> find(const std::string& name);

  /// Building blocks, exposed for custom scenarios and tests.
  static Road makeRoad(const std::string& geometry, double speed, double lane_width = 3.7);
  static SensorSuiteConfig makeSensors(const std::string& quality);
  static std::vector<std::string> geometries();
  static std::vector<std::string> sensorQualities();
  static std::vector<double> speeds();
};

}  // namespace lanelock
