#include "lanelock/vehicle.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

#include "lanelock/math.hpp"

namespace lanelock {

namespace {

constexpr double kMinSpeed = 1.0;  // m/s, guards slip-angle division at very low speed

// Integration state: s, e_y, e_psi, vy, r
using State5 = std::array<double, 5>;

State5 addScaled(const State5& a, const State5& b, double k) {
  State5 out{};
  for (std::size_t i = 0; i < a.size(); ++i) out[i] = a[i] + k * b[i];
  return out;
}

template <typename Deriv>
State5 rk4(const State5& x, double dt, Deriv&& f) {
  const State5 k1 = f(x);
  const State5 k2 = f(addScaled(x, k1, 0.5 * dt));
  const State5 k3 = f(addScaled(x, k2, 0.5 * dt));
  const State5 k4 = f(addScaled(x, k3, dt));
  State5 out{};
  for (std::size_t i = 0; i < x.size(); ++i) {
    out[i] = x[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }
  return out;
}

/// Frenet kinematics shared by both models.
/// s_dot   = (vx cos e_psi - vy sin e_psi) / (1 - k e_y)
/// e_y_dot =  vx sin e_psi + vy cos e_psi
/// e_psi_dot = r - k s_dot
void frenetRates(double vx, double vy, double r, double e_y, double e_psi, double k,
                 double& s_dot, double& e_y_dot, double& e_psi_dot) {
  const double denom = std::max(1.0 - k * e_y, 1e-3);
  s_dot = (vx * std::cos(e_psi) - vy * std::sin(e_psi)) / denom;
  e_y_dot = vx * std::sin(e_psi) + vy * std::cos(e_psi);
  e_psi_dot = r - k * s_dot;
}

}  // namespace

double VehicleParams::understeerGradient() const {
  const double L = wheelbase();
  return mass / L * (lr / cornering_stiffness_front - lf / cornering_stiffness_rear);
}

// ---------------------------------------------------------------------------- Kinematic

void KinematicBicycleModel::step(VehicleState& state, const Road& road, double steer,
                                 double lateral_force, double dt) const {
  const double delta = clamp(steer, -params_.max_road_wheel_angle, params_.max_road_wheel_angle);
  const double vx = state.vx;
  const double L = params_.wheelbase();
  const double yaw_rate = vx / L * std::tan(delta);
  // Disturbance -> steady-state sideslip: beta = F / (Cf + Cr); vy = vx * beta.
  const double c_total = params_.cornering_stiffness_front + params_.cornering_stiffness_rear;
  const double drift_vy = vx * lateral_force / c_total;

  auto deriv = [&](const State5& x) {
    State5 dx{};
    const double k = road.curvatureAt(x[0]);
    frenetRates(vx, drift_vy, yaw_rate, x[1], x[2], k, dx[0], dx[1], dx[2]);
    return dx;
  };

  State5 x{state.s, state.e_y, state.e_psi, drift_vy, yaw_rate};
  x = rk4(x, dt, deriv);

  state.s = x[0];
  state.e_y = x[1];
  state.e_psi = wrapAngle(x[2]);
  state.vy = drift_vy;
  state.yaw_rate = yaw_rate;
  state.lateral_accel = vx * yaw_rate;
}

// ---------------------------------------------------------------------------- Dynamic

void DynamicBicycleModel::step(VehicleState& state, const Road& road, double steer,
                               double lateral_force, double dt) const {
  const double delta = clamp(steer, -params_.max_road_wheel_angle, params_.max_road_wheel_angle);
  const double vx = std::max(state.vx, kMinSpeed);
  const double m = params_.mass;
  const double iz = params_.yaw_inertia;
  const double lf = params_.lf;
  const double lr = params_.lr;
  const double cf = params_.cornering_stiffness_front;
  const double cr = params_.cornering_stiffness_rear;

  auto lateralForces = [&](double vy, double r, double& fyf, double& fyr) {
    const double alpha_f = delta - (vy + lf * r) / vx;
    const double alpha_r = -(vy - lr * r) / vx;
    fyf = cf * alpha_f;
    fyr = cr * alpha_r;
  };

  auto deriv = [&](const State5& x) {
    State5 dx{};
    const double vy = x[3];
    const double r = x[4];
    const double k = road.curvatureAt(x[0]);
    frenetRates(vx, vy, r, x[1], x[2], k, dx[0], dx[1], dx[2]);
    double fyf = 0.0;
    double fyr = 0.0;
    lateralForces(vy, r, fyf, fyr);
    dx[3] = (fyf * std::cos(delta) + fyr + lateral_force) / m - vx * r;
    dx[4] = (lf * fyf * std::cos(delta) - lr * fyr) / iz;
    return dx;
  };

  State5 x{state.s, state.e_y, state.e_psi, state.vy, state.yaw_rate};
  x = rk4(x, dt, deriv);

  state.s = x[0];
  state.e_y = x[1];
  state.e_psi = wrapAngle(x[2]);
  state.vy = x[3];
  state.yaw_rate = x[4];

  // Lateral acceleration at the new state: a_y = vy_dot + vx r.
  double fyf = 0.0;
  double fyr = 0.0;
  lateralForces(state.vy, state.yaw_rate, fyf, fyr);
  state.lateral_accel = (fyf * std::cos(delta) + fyr + lateral_force) / m;
}

// ---------------------------------------------------------------------------- Factory

std::unique_ptr<VehicleModel> makeVehicleModel(VehicleModelType type, const VehicleParams& params) {
  switch (type) {
    case VehicleModelType::Kinematic:
      return std::make_unique<KinematicBicycleModel>(params);
    case VehicleModelType::Dynamic:
      return std::make_unique<DynamicBicycleModel>(params);
  }
  throw std::invalid_argument("unknown vehicle model type");
}

const char* toString(VehicleModelType type) {
  switch (type) {
    case VehicleModelType::Kinematic:
      return "kinematic";
    case VehicleModelType::Dynamic:
      return "dynamic";
  }
  return "unknown";
}

}  // namespace lanelock
