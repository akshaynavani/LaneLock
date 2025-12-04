# LaneLock — Implementation Plan

**LaneLock** is a C++17 closed-loop simulation framework for testing ADAS lane-keeping
control algorithms. A vehicle model drives along a procedurally defined road, a noisy lane
camera measures the vehicle's pose relative to the lane, a PID-based lateral controller
computes a steering command, and a steering actuator model applies it back to the vehicle.
Every run produces safety metrics (peak lateral error, lane departure, steering rate, lateral
acceleration). A scenario library generates 500+ safety-critical scenarios which are executed
as Google Test regression suites.

## Key design decisions

| Decision | Choice | Rationale |
|---|---|---|
| Language / build | C++17, CMake >= 3.16 | Portable, standard for automotive simulation tooling |
| Test framework | Google Test (fetched via `FetchContent`) | No system install needed; `TEST_P` parameterised suites map naturally to scenarios |
| Vehicle state frame | Frenet (road-relative) coordinates `(s, e_y, e_psi)` | Lateral/heading error come straight out of the state, no projection search needed |
| Vehicle models | Kinematic bicycle + linear-tire dynamic bicycle | Kinematic for low speed / fast tests, dynamic for realistic high-speed behaviour |
| Integrator | Fixed-step RK4 at 100 Hz | Deterministic and accurate enough for vehicle dynamics |
| Sensor noise | Gaussian noise, constant bias, random-walk drift, dropouts, latency, quantisation | Covers the common failure modes of a camera lane detector |
| Determinism | Every scenario carries a seed; one `std::mt19937_64` per sensor | Regression results are bit-for-bit reproducible |
| Controller | PID on a look-ahead error + curvature feed-forward, anti-windup, filtered derivative | Classic, explainable lateral controller that is easy to tune and test |
| Pass / fail | Explicit safety envelope per scenario (no lane departure, bounded lateral error, steering rate, lateral acceleration) | Turns each simulation into an assertable regression check |

## Phases

### Phase 1 — Foundation: build, math, road geometry, vehicle models
Deliverables
- CMake project with a core static library `lanelock_core`, CLI target and test target.
- Math helpers (angle wrapping, clamping, sign, first-order low-pass filter).
- Road model built from piecewise segments of linearly varying curvature
  (straights, constant arcs, clothoids) with lane width; query curvature/pose at arc length `s`.
- Vehicle parameters and `VehicleModel` interface; kinematic bicycle and dynamic bicycle models
  integrated in Frenet coordinates with RK4, including an external lateral-force disturbance.

Files: `CMakeLists.txt`, `include/lanelock/math.hpp`, `include/lanelock/road.hpp`,
`src/road.cpp`, `include/lanelock/vehicle.hpp`, `src/vehicle.cpp`

### Phase 2 — Sensors and actuation
Deliverables
- `NoiseModel` (Gaussian, bias, random-walk drift, quantisation) with seeded RNG.
- `LaneCamera` producing `LaneMeasurement {lateral_offset, heading_error, curvature, valid}`
  with configurable latency (ring buffer), dropout probability and burst dropouts.
- Speed and yaw-rate sensors.
- `SteeringActuator` with first-order lag, rate limit, angle saturation and dead time.

Files: `include/lanelock/noise.hpp`, `src/noise.cpp`, `include/lanelock/sensors.hpp`,
`src/sensors.cpp`, `include/lanelock/actuator.hpp`, `src/actuator.cpp`

### Phase 3 — Lateral control
Deliverables
- Generic `Pid` class: derivative-on-measurement with low-pass filter, integrator clamping and
  back-calculation anti-windup, output saturation.
- `LateralController` interface and `PidLaneKeepingController`: speed-scheduled look-ahead
  error, curvature feed-forward with understeer compensation, measurement-hold on camera
  dropout and a degraded/handover state when dropout exceeds a timeout.

Files: `include/lanelock/pid.hpp`, `src/pid.cpp`, `include/lanelock/controller.hpp`,
`src/controller.cpp`

### Phase 4 — Closed-loop simulator, metrics, scenarios and CLI
Deliverables
- `Simulator` wiring road + vehicle + sensors + controller + actuator at a fixed step with a
  per-step trace and disturbance schedule (wind gusts).
- `MetricsAccumulator` and `SafetyEnvelope` evaluation (pass/fail with reasons).
- `ScenarioLibrary`: deterministic generator of 540 scenarios across speed, road geometry,
  sensor degradation, initial offset and disturbance dimensions; plus named edge cases.
- CSV trace writer and baseline (metrics) reader/writer.
- `lanelock_sim` CLI: list scenarios, run one with a CSV trace, run the full suite with a summary,
  write a metrics baseline.

Files: `include/lanelock/simulator.hpp`, `src/simulator.cpp`, `include/lanelock/metrics.hpp`,
`src/metrics.cpp`, `include/lanelock/scenario.hpp`, `src/scenario.cpp`,
`include/lanelock/io.hpp`, `src/io.cpp`, `apps/lanelock_sim.cpp`

### Phase 5 — Google Test regression suites and CI
Deliverables
- Unit tests: math, road geometry, vehicle models, noise/sensors, actuator, PID, controller.
- Closed-loop tests: straight-road convergence, curve tracking, gust rejection, dropout handling.
- Parameterised regression suite running all 500+ library scenarios against the safety envelope,
  determinism test, and optional comparison against a recorded metrics baseline.
- GitHub Actions workflow building and running `ctest`.

Files: `tests/*.cpp`, `tests/CMakeLists.txt`, `.github/workflows/ci.yml`

### Wrap-up
`.gitignore`, `README.md` with architecture diagram, run and test instructions.
