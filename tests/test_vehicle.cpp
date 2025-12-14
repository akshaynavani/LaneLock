#include <gtest/gtest.h>

#include <cmath>

#include "lanelock/vehicle.hpp"

using namespace lanelock;

namespace {

VehicleState startState(double v) {
  VehicleState s;
  s.vx = v;
  return s;
}

void simulate(const VehicleModel& model, VehicleState& state, const Road& road, double steer,
              double force, double seconds, double dt = 0.01) {
  const int steps = static_cast<int>(std::lround(seconds / dt));
  for (int i = 0; i < steps; ++i) model.step(state, road, steer, force, dt);
}

const Road kStraight = Road::Builder().straight(5000.0).build();

}  // namespace

TEST(Vehicle, DefaultCarUndersteers) {
  VehicleParams p;
  EXPECT_DOUBLE_EQ(p.wheelbase(), 2.8);
  EXPECT_GT(p.understeerGradient(), 0.0);
}

TEST(Vehicle, FactoryCreatesRequestedModel) {
  EXPECT_EQ(makeVehicleModel(VehicleModelType::Kinematic, {})->name(), "kinematic");
  EXPECT_EQ(makeVehicleModel(VehicleModelType::Dynamic, {})->name(), "dynamic");
  EXPECT_STREQ(toString(VehicleModelType::Dynamic), "dynamic");
}

class BothModels : public ::testing::TestWithParam<VehicleModelType> {};

TEST_P(BothModels, DrivesStraightWithoutSteering) {
  auto model = makeVehicleModel(GetParam(), {});
  VehicleState s = startState(25.0);
  simulate(*model, s, kStraight, 0.0, 0.0, 4.0);
  EXPECT_NEAR(s.s, 100.0, 1e-6);
  EXPECT_NEAR(s.e_y, 0.0, 1e-9);
  EXPECT_NEAR(s.e_psi, 0.0, 1e-9);
}

TEST_P(BothModels, LeftSteeringMovesVehicleLeft) {
  auto model = makeVehicleModel(GetParam(), {});
  VehicleState s = startState(20.0);
  simulate(*model, s, kStraight, 0.01, 0.0, 1.0);
  EXPECT_GT(s.e_psi, 0.0);
  EXPECT_GT(s.e_y, 0.0);
  EXPECT_GT(s.yaw_rate, 0.0);
}

TEST_P(BothModels, PositiveLateralForcePushesLeft) {
  auto model = makeVehicleModel(GetParam(), {});
  VehicleState s = startState(30.0);
  simulate(*model, s, kStraight, 0.0, 800.0, 1.0);
  EXPECT_GT(s.e_y, 0.0);
}

TEST_P(BothModels, SteeringIsSaturatedAtMaxRoadWheelAngle) {
  VehicleParams p;
  auto model = makeVehicleModel(GetParam(), p);
  VehicleState a = startState(10.0);
  VehicleState b = startState(10.0);
  simulate(*model, a, kStraight, p.max_road_wheel_angle, 0.0, 0.5);
  simulate(*model, b, kStraight, 10.0 * p.max_road_wheel_angle, 0.0, 0.5);
  EXPECT_NEAR(a.e_psi, b.e_psi, 1e-12);
  EXPECT_NEAR(a.e_y, b.e_y, 1e-12);
}

INSTANTIATE_TEST_SUITE_P(Vehicle, BothModels,
                         ::testing::Values(VehicleModelType::Kinematic, VehicleModelType::Dynamic),
                         [](const ::testing::TestParamInfo<VehicleModelType>& info) {
                           return std::string(toString(info.param));
                         });

TEST(Vehicle, KinematicYawRateMatchesBicycleGeometry) {
  VehicleParams p;
  KinematicBicycleModel model(p);
  VehicleState s = startState(20.0);
  const double delta = 0.01;
  simulate(model, s, kStraight, delta, 0.0, 1.0);
  const double expected_rate = 20.0 / p.wheelbase() * std::tan(delta);
  EXPECT_NEAR(s.yaw_rate, expected_rate, 1e-12);
  EXPECT_NEAR(s.e_psi, expected_rate * 1.0, 1e-9);
}

TEST(Vehicle, KinematicTracksArcWithGeometricSteering) {
  VehicleParams p;
  KinematicBicycleModel model(p);
  const double k = 1.0 / 150.0;
  const Road arc = Road::Builder().arc(2000.0, k).build();
  VehicleState s = startState(15.0);
  simulate(model, s, arc, std::atan(p.wheelbase() * k), 0.0, 8.0);
  EXPECT_NEAR(s.e_y, 0.0, 1e-3);
  EXPECT_NEAR(s.e_psi, 0.0, 1e-4);
  EXPECT_NEAR(s.lateral_accel, 15.0 * 15.0 * k, 1e-3);
}

TEST(Vehicle, DynamicSteadyStateYawRateShowsUndersteer) {
  VehicleParams p;
  DynamicBicycleModel model(p);
  const double v = 20.0;
  const double delta = 0.01;
  VehicleState s = startState(v);
  simulate(model, s, kStraight, delta, 0.0, 6.0);
  // Linear bicycle steady state: r = v delta / (L + K_us v^2).
  const double expected = v * delta / (p.wheelbase() + p.understeerGradient() * v * v);
  EXPECT_NEAR(s.yaw_rate, expected, 0.01 * expected);
  EXPECT_LT(s.yaw_rate, v / p.wheelbase() * delta);  // less than the kinematic yaw rate
  EXPECT_NEAR(s.lateral_accel, v * s.yaw_rate, 0.01 * v * expected);
}

TEST(Vehicle, DynamicModelHasSideslipInSteadyCornering) {
  VehicleParams p;
  DynamicBicycleModel model(p);
  const double v = 30.0;
  VehicleState s = startState(v);
  simulate(model, s, kStraight, 0.01, 0.0, 6.0);
  // At highway speed the CG slides outwards (negative sideslip in a left turn).
  EXPECT_LT(s.vy, 0.0);
}
