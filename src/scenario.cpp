#include "lanelock/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include "lanelock/math.hpp"
#include "lanelock/noise.hpp"

namespace lanelock {

// ---------------------------------------------------------------------------- Disturbance

double Disturbance::forceAt(double t) const {
  const double rel = t - start;
  if (rel < 0.0) return 0.0;
  const double r = std::max(ramp, 1e-9);
  if (rel < ramp) return force * rel / r;
  if (rel < ramp + hold) return force;
  if (rel < 2.0 * ramp + hold) return force * (1.0 - (rel - ramp - hold) / r);
  return 0.0;
}

double Scenario::disturbanceAt(double t) const {
  double f = 0.0;
  for (const auto& d : disturbances) f += d.forceAt(t);
  return f;
}

bool Scenario::hasTag(const std::string& tag) const {
  return std::find(tags.begin(), tags.end(), tag) != tags.end();
}

// ---------------------------------------------------------------------------- building blocks

namespace {

constexpr std::uint64_t kLibrarySeed = 0x1A9E10C4ULL;
constexpr double kGustForce = 500.0;   // N
constexpr double kGustStart = 12.0;    // s
constexpr double kDefaultDuration = 30.0;

/// Curvature that produces lateral acceleration `ay` at speed `v`.
double curvatureFor(double ay, double v) { return ay / (v * v); }

std::string speedLabel(double v) {
  std::ostringstream os;
  os << 'v' << static_cast<int>(std::lround(v));
  return os.str();
}

Scenario baseScenario(double speed) {
  Scenario sc;
  sc.speed = speed;
  sc.model = VehicleModelType::Dynamic;
  sc.controller.compensate_understeer = true;
  sc.duration = kDefaultDuration;
  return sc;
}

struct InitialCondition {
  std::string name;
  double offset;
  double heading;
};

const std::vector<InitialCondition>& initialConditions() {
  static const std::vector<InitialCondition> ics = {
      {"centered", 0.0, 0.0},
      {"offset_left", 0.5, 0.0},
      {"drifting_right", -0.2, -0.015},
  };
  return ics;
}

}  // namespace

std::vector<double> ScenarioLibrary::speeds() { return {15.0, 20.0, 25.0, 30.0, 35.0}; }

std::vector<std::string> ScenarioLibrary::geometries() {
  return {"straight",       "curve_left_ay1", "curve_right_ay2",
          "curve_left_ay3", "s_curve_ay2",    "decreasing_radius"};
}

std::vector<std::string> ScenarioLibrary::sensorQualities() {
  return {"ideal", "nominal", "degraded"};
}

Road ScenarioLibrary::makeRoad(const std::string& geometry, double v, double lane_width) {
  // Distances scale with speed so every geometry is traversed in a similar amount of time:
  // a 3 s lead-in, the feature, then straight road to the end of the run.
  Road::Builder b(lane_width);
  b.name(geometry);
  const double lead = 3.0 * v;
  const double transition = std::max(40.0, 2.0 * v);
  b.straight(lead);

  if (geometry == "straight") {
    b.straight(30.0 * v);
  } else if (geometry == "curve_left_ay1") {
    b.curve(6.0 * v, curvatureFor(1.0, v), transition);
  } else if (geometry == "curve_right_ay2") {
    b.curve(6.0 * v, -curvatureFor(2.0, v), transition);
  } else if (geometry == "curve_left_ay3") {
    b.curve(6.0 * v, curvatureFor(3.0, v), transition);
  } else if (geometry == "s_curve_ay2") {
    const double k = curvatureFor(2.0, v);
    b.curve(3.0 * v, k, transition);
    b.straight(1.0 * v);
    b.curve(3.0 * v, -k, transition);
  } else if (geometry == "decreasing_radius") {
    const double k = curvatureFor(2.5, v);
    b.clothoid(8.0 * v, 0.0, k);
    b.arc(3.0 * v, k);
    b.clothoid(3.0 * v, k, 0.0);
  } else {
    throw std::invalid_argument("unknown road geometry: " + geometry);
  }
  b.straight(10.0 * v);
  return b.build();
}

SensorSuiteConfig ScenarioLibrary::makeSensors(const std::string& quality) {
  if (quality == "ideal") return SensorSuiteConfig::ideal();
  SensorSuiteConfig c;  // defaults are the "nominal" production-grade camera
  c.camera.dropout_probability = 0.01;
  c.camera.dropout_burst_frames = 2;
  if (quality == "nominal") return c;
  if (quality == "degraded") {
    c.camera.latency = 0.10;
    c.camera.rate_hz = 15.0;
    c.camera.offset_noise = NoiseParams{0.08, 0.05, 0.01, 0.01};
    c.camera.heading_noise = NoiseParams{0.006, 0.0, 0.0005, 0.0};
    c.camera.curvature_noise = NoiseParams{3e-4, 0.0, 0.0, 0.0};
    c.camera.dropout_probability = 0.05;
    c.camera.dropout_burst_frames = 4;
    c.speed_noise = NoiseParams{0.2, 0.1, 0.0, 0.0};
    c.yaw_rate_noise = NoiseParams{0.005, 0.001, 0.001, 0.0};
    return c;
  }
  throw std::invalid_argument("unknown sensor quality: " + quality);
}

// ---------------------------------------------------------------------------- library

std::vector<Scenario> ScenarioLibrary::parametric() {
  std::vector<Scenario> out;
  std::uint64_t index = 0;
  for (const auto& geometry : geometries()) {
    for (double v : speeds()) {
      const Road road = makeRoad(geometry, v);
      // Gusts push towards the outside of the first curve (worst case); left on straights.
      const double k_first = road.maxAbsCurvature() > 0.0 ? road.previewCurvature(3.0 * v, 400.0, 40) : 0.0;
      const double gust_sign = k_first > 0.0 ? -1.0 : 1.0;

      for (const auto& quality : sensorQualities()) {
        for (const auto& ic : initialConditions()) {
          for (bool gust : {false, true}) {
            Scenario sc = baseScenario(v);
            sc.road = road;
            sc.category = geometry;
            sc.sensors = makeSensors(quality);
            sc.initial_offset = ic.offset;
            sc.initial_heading = ic.heading;
            if (gust) sc.disturbances.push_back({kGustStart, 0.3, 1.5, gust_sign * kGustForce});
            sc.seed = deriveSeed(kLibrarySeed, index++);
            sc.tags = {geometry, speedLabel(v), quality, ic.name, gust ? "gust" : "calm"};
            sc.name = geometry + "_" + speedLabel(v) + "_" + quality + "_" + ic.name + "_" +
                      (gust ? "gust" : "calm");
            out.push_back(std::move(sc));
          }
        }
      }
    }
  }
  return out;
}

std::vector<Scenario> ScenarioLibrary::edgeCases() {
  std::vector<Scenario> out;
  std::uint64_t index = 10000;
  auto add = [&](Scenario sc, const std::string& name) {
    sc.name = "edge_" + name;
    sc.category = "edge";
    sc.tags.insert(sc.tags.begin(), "edge");
    sc.seed = deriveSeed(kLibrarySeed, index++);
    out.push_back(std::move(sc));
  };
  auto curveRoad = [](double v, double ay) {
    return Road::Builder().name("edge_curve").straight(3.0 * v)
        .curve(8.0 * v, curvatureFor(ay, v), std::max(30.0, 2.0 * v)).straight(20.0 * v).build();
  };

  {  // Tight hairpin at urban speed.
    Scenario sc = baseScenario(10.0);
    sc.road = Road::Builder().name("hairpin").straight(40.0).curve(120.0, 1.0 / 40.0, 25.0)
                  .straight(300.0).build();
    sc.sensors = makeSensors("nominal");
    sc.tags = {"hairpin", "low_speed"};
    add(sc, "hairpin_low_speed");
  }
  {  // Kinematic plant, controller calibrated accordingly.
    Scenario sc = baseScenario(12.0);
    sc.model = VehicleModelType::Kinematic;
    sc.controller.compensate_understeer = false;
    sc.road = curveRoad(12.0, 2.5);
    sc.sensors = makeSensors("nominal");
    sc.tags = {"kinematic"};
    add(sc, "kinematic_tight_curve");
  }
  {
    Scenario sc = baseScenario(30.0);
    sc.model = VehicleModelType::Kinematic;
    sc.controller.compensate_understeer = false;
    sc.road = makeRoad("s_curve_ay2", 30.0);
    sc.sensors = makeSensors("nominal");
    sc.tags = {"kinematic"};
    add(sc, "kinematic_highway_s_curve");
  }
  {  // Short camera blackout on a straight: controller must dead-reckon.
    Scenario sc = baseScenario(25.0);
    sc.road = makeRoad("straight", 25.0);
    sc.sensors = makeSensors("nominal");
    sc.sensors.camera.blackout_start = 8.0;
    sc.sensors.camera.blackout_duration = 0.8;
    sc.tags = {"blackout"};
    add(sc, "camera_blackout_straight");
  }
  {  // Short blackout in the middle of a curve.
    Scenario sc = baseScenario(25.0);
    sc.road = curveRoad(25.0, 2.0);
    sc.sensors = makeSensors("nominal");
    sc.sensors.camera.blackout_start = 9.0;
    sc.sensors.camera.blackout_duration = 0.8;
    sc.tags = {"blackout", "curve"};
    add(sc, "camera_blackout_curve");
  }
  {  // Long blackout: controller must declare itself unavailable (handover).
    Scenario sc = baseScenario(25.0);
    sc.road = makeRoad("straight", 25.0);
    sc.sensors = makeSensors("nominal");
    sc.sensors.camera.blackout_start = 8.0;
    sc.sensors.camera.blackout_duration = 3.0;
    sc.tags = {"blackout", "handover"};
    add(sc, "long_blackout_handover");
  }
  {
    Scenario sc = baseScenario(25.0);
    sc.road = curveRoad(25.0, 2.0);
    sc.sensors = makeSensors("nominal");
    sc.sensors.camera.latency = 0.2;
    sc.tags = {"latency"};
    add(sc, "high_camera_latency");
  }
  {
    Scenario sc = baseScenario(30.0);
    sc.road = makeRoad("straight", 30.0);
    sc.sensors = makeSensors("nominal");
    sc.disturbances.push_back({10.0, 0.3, 2.0, 900.0});
    sc.tags = {"gust"};
    add(sc, "strong_crosswind");
  }
  {  // Starts with the tyre 12 cm from the lane line.
    Scenario sc = baseScenario(20.0);
    sc.road = makeRoad("straight", 20.0);
    sc.sensors = makeSensors("nominal");
    sc.initial_offset = 0.8;
    sc.envelope.max_abs_lateral_error = 0.9;
    sc.tags = {"recovery"};
    add(sc, "recover_from_lane_line");
  }
  {
    Scenario sc = baseScenario(25.0);
    sc.road = curveRoad(25.0, 1.5);
    sc.sensors = makeSensors("nominal");
    sc.sensors.camera.offset_noise.bias = 0.15;
    sc.tags = {"bias"};
    add(sc, "camera_offset_bias");
  }
  {
    Scenario sc = baseScenario(30.0);
    sc.road = curveRoad(30.0, 2.0);
    sc.sensors = makeSensors("nominal");
    sc.actuator.time_constant = 0.25;
    sc.actuator.max_rate = 0.15;
    sc.tags = {"actuator"};
    add(sc, "sluggish_actuator");
  }
  {
    Scenario sc = baseScenario(20.0);
    sc.road = curveRoad(20.0, 2.0);
    sc.sensors = makeSensors("nominal");
    sc.sensors.camera.dropout_probability = 0.2;
    sc.sensors.camera.dropout_burst_frames = 3;
    sc.tags = {"dropout"};
    add(sc, "heavy_dropout_curve");
  }
  return out;
}

std::vector<Scenario> ScenarioLibrary::all() {
  std::vector<Scenario> out = parametric();
  std::vector<Scenario> edges = edgeCases();
  out.insert(out.end(), std::make_move_iterator(edges.begin()),
             std::make_move_iterator(edges.end()));
  return out;
}

std::vector<Scenario> ScenarioLibrary::filtered(const std::string& filter) {
  std::vector<Scenario> out;
  for (auto& sc : all()) {
    if (filter.empty() || sc.name.find(filter) != std::string::npos) out.push_back(std::move(sc));
  }
  return out;
}

std::optional<Scenario> ScenarioLibrary::find(const std::string& name) {
  for (auto& sc : all()) {
    if (sc.name == name) return std::move(sc);
  }
  return std::nullopt;
}

}  // namespace lanelock
