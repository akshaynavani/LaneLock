#include <gtest/gtest.h>

#include <cmath>

#include "lanelock/math.hpp"
#include "lanelock/road.hpp"

using namespace lanelock;

TEST(Road, StraightRoadPoseIsLinear) {
  const Road road = Road::Builder().straight(500.0).build();
  EXPECT_DOUBLE_EQ(road.length(), 500.0);
  const Pose2D p = road.poseAt(123.0);
  EXPECT_NEAR(p.x, 123.0, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
  EXPECT_NEAR(p.heading, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(road.curvatureAt(250.0), 0.0);
}

TEST(Road, QuarterCircleArcEndsAtExpectedPose) {
  const double R = 100.0;
  const Road road = Road::Builder().arc(kPi * R / 2.0, 1.0 / R).build();
  const Pose2D end = road.poseAt(road.length());
  EXPECT_NEAR(end.x, R, 1e-4);
  EXPECT_NEAR(end.y, R, 1e-4);
  EXPECT_NEAR(end.heading, kPi / 2.0, 1e-9);
}

TEST(Road, RightHandArcHasNegativeCurvature) {
  const double R = 200.0;
  const Road road = Road::Builder().arc(kPi * R / 2.0, -1.0 / R).build();
  const Pose2D end = road.poseAt(road.length());
  EXPECT_NEAR(end.x, R, 1e-4);
  EXPECT_NEAR(end.y, -R, 1e-4);
  EXPECT_NEAR(end.heading, -kPi / 2.0, 1e-9);
}

TEST(Road, ClothoidCurvatureVariesLinearly) {
  const Road road = Road::Builder().straight(50.0).clothoid(100.0, 0.0, 0.01).build();
  EXPECT_NEAR(road.curvatureAt(50.0), 0.0, 1e-12);
  EXPECT_NEAR(road.curvatureAt(100.0), 0.005, 1e-12);
  EXPECT_NEAR(road.curvatureAt(149.999), 0.01, 1e-6);
  // Heading change of a clothoid from 0 to k over L is k L / 2.
  EXPECT_NEAR(road.poseAt(150.0).heading, 0.5, 1e-9);
}

TEST(Road, CurveBuilderAddsSymmetricTransitions) {
  const double k = 0.004;
  const Road road = Road::Builder().curve(200.0, k, 60.0).build();
  ASSERT_EQ(road.segments().size(), 3u);
  EXPECT_DOUBLE_EQ(road.length(), 320.0);
  EXPECT_NEAR(road.maxAbsCurvature(), k, 1e-12);
  // Total heading change = k * (arc + transition).
  EXPECT_NEAR(road.poseAt(road.length()).heading, k * 260.0, 1e-9);
}

TEST(Road, ContinuesStraightPastTheEnd) {
  const Road road = Road::Builder().arc(100.0, 0.005).build();
  const Pose2D end = road.poseAt(road.length());
  const Pose2D beyond = road.poseAt(road.length() + 50.0);
  EXPECT_DOUBLE_EQ(road.curvatureAt(road.length() + 10.0), 0.0);
  EXPECT_NEAR(beyond.heading, end.heading, 1e-12);
  EXPECT_NEAR(beyond.x, end.x + 50.0 * std::cos(end.heading), 1e-9);
  EXPECT_NEAR(beyond.y, end.y + 50.0 * std::sin(end.heading), 1e-9);
}

TEST(Road, PreviewCurvatureAveragesAhead) {
  const Road road = Road::Builder().straight(100.0).arc(100.0, 0.01).build();
  EXPECT_NEAR(road.previewCurvature(0.0, 50.0), 0.0, 1e-12);
  EXPECT_NEAR(road.previewCurvature(120.0, 50.0), 0.01, 1e-12);
  const double partial = road.previewCurvature(80.0, 40.0, 41);
  EXPECT_GT(partial, 0.003);
  EXPECT_LT(partial, 0.007);
}

TEST(Road, RejectsInvalidInput) {
  EXPECT_THROW(Road::Builder().straight(-1.0), std::invalid_argument);
  EXPECT_THROW(Road({}, 0.0), std::invalid_argument);
}
