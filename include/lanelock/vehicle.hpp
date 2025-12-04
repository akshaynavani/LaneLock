#pragma once

#include <memory>
#include <string>

#include "lanelock/road.hpp"

namespace lanelock {

/// Physical parameters of a mid-size passenger car (defaults).
struct VehicleParams {
  double mass = 1600.0;              // kg
  double yaw_inertia = 2700.0;       // kg m^2
  double lf = 1.2;                   // CG to front axle, m
  double lr = 1.6;                   // CG to rear axle, m
  double cornering_stiffness_front = 80000.0;  // N/rad (both tyres)
  double cornering_stiffness_rear = 90000.0;   // N/rad (both tyres)
  double width = 1.85;               // m
  double max_road_wheel_angle = 0.5;  // rad

  double wheelbase() const { return lf + lr; }

  /// Understeer gradient K_us [rad / (m/s^2)]; steady-state delta = L*k + K_us * a_y.
  double understeerGradient() const;
};

/// Vehicle state expressed in road-relative (Frenet) coordinates.
struct VehicleState {
  double s = 0.0;          // arc length along the lane centre, m
  double e_y = 0.0;        // lateral offset from lane centre (+ = left), m
  double e_psi = 0.0;      // heading relative to lane tangent, rad
  double vx = 20.0;        // longitudinal speed (body frame), m/s
  double vy = 0.0;         // lateral speed (body frame), m/s
  double yaw_rate = 0.0;   // rad/s
  double lateral_accel = 0.0;  // m/s^2, derived output of the last step
};

/// Interface for a lateral vehicle model integrated in Frenet coordinates.
class VehicleModel {
 public:
  explicit VehicleModel(VehicleParams params) : params_(params) {}
  virtual ~VehicleModel() = default;

  /// Advances `state` by `dt` with road-wheel angle `steer` (rad) and an external lateral
  /// force `lateral_force` (N, + = pushes left) such as a crosswind. Uses RK4.
  virtual void step(VehicleState& state, const Road& road, double steer, double lateral_force,
                    double dt) const = 0;

  virtual std::string name() const = 0;
  const VehicleParams& params() const { return params_; }

 protected:
  VehicleParams params_;
};

/// Kinematic bicycle (no tyre slip). Lateral disturbance forces are mapped to a steady-state
/// sideslip drift so the model still reacts to crosswind.
class KinematicBicycleModel : public VehicleModel {
 public:
  using VehicleModel::VehicleModel;
  void step(VehicleState& state, const Road& road, double steer, double lateral_force,
            double dt) const override;
  std::string name() const override { return "kinematic"; }
};

/// Linear-tyre dynamic bicycle model (states vy, r) — valid for small slip angles.
class DynamicBicycleModel : public VehicleModel {
 public:
  using VehicleModel::VehicleModel;
  void step(VehicleState& state, const Road& road, double steer, double lateral_force,
            double dt) const override;
  std::string name() const override { return "dynamic"; }
};

enum class VehicleModelType { Kinematic, Dynamic };

std::unique_ptr<VehicleModel> makeVehicleModel(VehicleModelType type, const VehicleParams& params);
const char* toString(VehicleModelType type);

}  // namespace lanelock
