#pragma once

#include <cstdint>
#include <random>

namespace lanelock {

/// Parameters of an additive measurement error model.
///   measured = quantise(true + bias + drift(t) + N(0, stddev^2))
struct NoiseParams {
  double stddev = 0.0;            // white Gaussian noise standard deviation
  double bias = 0.0;              // constant offset
  double drift_rate_stddev = 0.0;  // random-walk drift intensity, units / sqrt(s)
  double quantization = 0.0;      // resolution; 0 disables quantisation

  static NoiseParams none() { return {}; }
};

/// Stateful noise generator. Each instance owns its RNG so sensors are independent and every
/// run is reproducible from a seed.
class NoiseModel {
 public:
  NoiseModel(NoiseParams params, std::uint64_t seed);

  /// Applies the error model to `true_value` and advances the drift by `dt`.
  double apply(double true_value, double dt);

  double drift() const { return drift_; }
  const NoiseParams& params() const { return params_; }
  void reset(std::uint64_t seed);

 private:
  NoiseParams params_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  double drift_ = 0.0;
};

/// Seeded Bernoulli source for dropouts, with optional burst (Gilbert-Elliott style) behaviour:
/// once a dropout starts it persists for `burst_length` samples.
class DropoutModel {
 public:
  DropoutModel(double probability, int burst_length, std::uint64_t seed);

  /// Returns true if the current sample is dropped.
  bool dropped();

  void reset(std::uint64_t seed);

 private:
  double probability_;
  int burst_length_;
  int remaining_ = 0;
  std::mt19937_64 rng_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
};

/// Derives a statistically independent child seed from a parent seed and a stream id
/// (SplitMix64 finaliser).
std::uint64_t deriveSeed(std::uint64_t parent, std::uint64_t stream);

}  // namespace lanelock
