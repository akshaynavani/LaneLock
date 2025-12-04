#include "lanelock/road.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "lanelock/math.hpp"

namespace lanelock {

double RoadSegment::curvatureAt(double local_s) const {
  if (length <= 0.0) return k_end;
  const double t = clamp(local_s / length, 0.0, 1.0);
  return k_start + (k_end - k_start) * t;
}

// ---------------------------------------------------------------------------- Builder

Road::Builder& Road::Builder::straight(double length) { return clothoid(length, 0.0, 0.0); }

Road::Builder& Road::Builder::arc(double length, double curvature) {
  return clothoid(length, curvature, curvature);
}

Road::Builder& Road::Builder::clothoid(double length, double k_start, double k_end) {
  if (length <= 0.0) throw std::invalid_argument("road segment length must be positive");
  segments_.push_back(RoadSegment{length, k_start, k_end});
  return *this;
}

Road::Builder& Road::Builder::curve(double arc_length, double curvature, double transition_length) {
  if (transition_length > 0.0) clothoid(transition_length, 0.0, curvature);
  if (arc_length > 0.0) arc(arc_length, curvature);
  if (transition_length > 0.0) clothoid(transition_length, curvature, 0.0);
  return *this;
}

Road::Builder& Road::Builder::name(std::string road_name) {
  name_ = std::move(road_name);
  return *this;
}

Road Road::Builder::build() const { return Road(segments_, lane_width_, name_); }

// ---------------------------------------------------------------------------- Road

Road::Road(std::vector<RoadSegment> segments, double lane_width, std::string name)
    : segments_(std::move(segments)), lane_width_(lane_width), name_(std::move(name)) {
  if (lane_width_ <= 0.0) throw std::invalid_argument("lane width must be positive");
  Pose2D pose;
  double s = 0.0;
  segment_start_s_.reserve(segments_.size());
  segment_start_pose_.reserve(segments_.size());
  for (const auto& seg : segments_) {
    segment_start_s_.push_back(s);
    segment_start_pose_.push_back(pose);
    pose = integrateSegment(pose, seg, seg.length);
    s += seg.length;
  }
  total_length_ = s;
}

int Road::segmentIndex(double s) const {
  if (segments_.empty() || s >= total_length_) return -1;
  if (s <= 0.0) return 0;
  const auto it = std::upper_bound(segment_start_s_.begin(), segment_start_s_.end(), s);
  return static_cast<int>(std::distance(segment_start_s_.begin(), it)) - 1;
}

double Road::curvatureAt(double s) const {
  const int idx = segmentIndex(s);
  if (idx < 0) return 0.0;  // straight extension past the end
  const auto i = static_cast<std::size_t>(idx);
  return segments_[i].curvatureAt(s - segment_start_s_[i]);
}

Pose2D Road::integrateSegment(const Pose2D& start, const RoadSegment& seg, double local_s) {
  // heading(u) = h0 + k0 u + 0.5 dk u^2 is exact; position is integrated with composite Simpson.
  const double dk = seg.length > 0.0 ? (seg.k_end - seg.k_start) / seg.length : 0.0;
  auto heading = [&](double u) { return start.heading + seg.k_start * u + 0.5 * dk * u * u; };

  Pose2D out = start;
  out.heading = wrapAngle(heading(local_s));
  if (local_s <= 0.0) return out;

  int n = std::max(2, static_cast<int>(std::ceil(local_s / 1.0)));  // ~1 m resolution
  if (n % 2 != 0) ++n;
  const double h = local_s / n;
  double sx = 0.0;
  double sy = 0.0;
  for (int i = 0; i <= n; ++i) {
    const double w = (i == 0 || i == n) ? 1.0 : (i % 2 == 1 ? 4.0 : 2.0);
    const double th = heading(i * h);
    sx += w * std::cos(th);
    sy += w * std::sin(th);
  }
  out.x = start.x + sx * h / 3.0;
  out.y = start.y + sy * h / 3.0;
  return out;
}

Pose2D Road::poseAt(double s) const {
  s = std::max(0.0, s);
  const int idx = segmentIndex(s);
  if (idx >= 0) {
    const auto i = static_cast<std::size_t>(idx);
    return integrateSegment(segment_start_pose_[i], segments_[i], s - segment_start_s_[i]);
  }
  // Past the end: continue straight from the final pose.
  Pose2D end;
  if (!segments_.empty()) {
    end = integrateSegment(segment_start_pose_.back(), segments_.back(), segments_.back().length);
  }
  const double extra = s - total_length_;
  end.x += extra * std::cos(end.heading);
  end.y += extra * std::sin(end.heading);
  return end;
}

double Road::previewCurvature(double s, double horizon, int samples) const {
  if (horizon <= 0.0 || samples <= 1) return curvatureAt(s);
  double sum = 0.0;
  for (int i = 0; i < samples; ++i) {
    sum += curvatureAt(s + horizon * i / (samples - 1));
  }
  return sum / samples;
}

double Road::maxAbsCurvature() const {
  double k = 0.0;
  for (const auto& seg : segments_) k = std::max({k, std::abs(seg.k_start), std::abs(seg.k_end)});
  return k;
}

}  // namespace lanelock
