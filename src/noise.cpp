#include "lanelock/noise.hpp"

#include <algorithm>
#include <cmath>

namespace lanelock {

std::uint64_t deriveSeed(std::uint64_t parent, std::uint64_t stream) {
  std::uint64_t z = parent + 0x9E3779B97F4A7C15ULL * (stream + 1);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

// ---------------------------------------------------------------------------- NoiseModel

NoiseModel::NoiseModel(NoiseParams params, std::uint64_t seed) : params_(params), rng_(seed) {}

void NoiseModel::reset(std::uint64_t seed) {
  rng_.seed(seed);
  normal_.reset();
  drift_ = 0.0;
}

double NoiseModel::apply(double true_value, double dt) {
  if (params_.drift_rate_stddev > 0.0 && dt > 0.0) {
    drift_ += params_.drift_rate_stddev * std::sqrt(dt) * normal_(rng_);
  }
  double value = true_value + params_.bias + drift_;
  if (params_.stddev > 0.0) value += params_.stddev * normal_(rng_);
  if (params_.quantization > 0.0) {
    value = std::round(value / params_.quantization) * params_.quantization;
  }
  return value;
}

// ---------------------------------------------------------------------------- DropoutModel

DropoutModel::DropoutModel(double probability, int burst_length, std::uint64_t seed)
    : probability_(std::clamp(probability, 0.0, 1.0)),
      burst_length_(std::max(1, burst_length)),
      rng_(seed) {}

void DropoutModel::reset(std::uint64_t seed) {
  rng_.seed(seed);
  uniform_.reset();
  remaining_ = 0;
}

bool DropoutModel::dropped() {
  if (remaining_ > 0) {
    --remaining_;
    return true;
  }
  if (probability_ <= 0.0) return false;
  if (uniform_(rng_) < probability_) {
    remaining_ = burst_length_ - 1;
    return true;
  }
  return false;
}

}  // namespace lanelock
