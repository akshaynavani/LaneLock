#pragma once

#include <string>
#include <vector>

namespace lanelock {

/// A point on the road centre-line in world coordinates.
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;  // rad, counter-clockwise from +x
};

/// One road piece whose curvature varies linearly with arc length.
///  - straight:  k_start == k_end == 0
///  - arc:       k_start == k_end != 0
///  - clothoid:  k_start != k_end (transition curve)
/// Positive curvature turns left.
struct RoadSegment {
  double length = 0.0;   // m
  double k_start = 0.0;  // 1/m
  double k_end = 0.0;    // 1/m

  double curvatureAt(double local_s) const;
};

/// Piecewise road centre-line. The road is extended as a straight line past its last segment
/// so a vehicle never "falls off" the end of the map during a simulation.
class Road {
 public:
  class Builder {
   public:
    explicit Builder(double lane_width = 3.7) : lane_width_(lane_width) {}

    Builder& straight(double length);
    Builder& arc(double length, double curvature);
    Builder& clothoid(double length, double k_start, double k_end);
    /// Clothoid entry, constant arc and clothoid exit back to straight.
    Builder& curve(double arc_length, double curvature, double transition_length);
    Builder& name(std::string road_name);

    Road build() const;

   private:
    double lane_width_;
    std::string name_ = "road";
    std::vector<RoadSegment> segments_;
  };

  Road() = default;
  Road(std::vector<RoadSegment> segments, double lane_width, std::string name = "road");

  double curvatureAt(double s) const;
  Pose2D poseAt(double s) const;

  /// Average absolute curvature over [s, s + horizon]; useful as a preview signal.
  double previewCurvature(double s, double horizon, int samples = 10) const;

  /// Maximum absolute curvature anywhere on the road.
  double maxAbsCurvature() const;

  double length() const { return total_length_; }
  double laneWidth() const { return lane_width_; }
  const std::string& name() const { return name_; }
  const std::vector<RoadSegment>& segments() const { return segments_; }

 private:
  /// Index of the segment containing s, or -1 if s is beyond the last segment.
  int segmentIndex(double s) const;
  static Pose2D integrateSegment(const Pose2D& start, const RoadSegment& seg, double local_s);

  std::vector<RoadSegment> segments_;
  std::vector<double> segment_start_s_;
  std::vector<Pose2D> segment_start_pose_;
  double total_length_ = 0.0;
  double lane_width_ = 3.7;
  std::string name_ = "road";
};

}  // namespace lanelock
