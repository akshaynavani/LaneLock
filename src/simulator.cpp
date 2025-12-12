#include "lanelock/simulator.hpp"

#include <cmath>
#include <stdexcept>

namespace lanelock {

Simulator::Simulator(const Scenario& scenario)
    : Simulator(scenario, std::make_unique<PidLaneKeepingController>(scenario.controller,
                                                                     scenario.vehicle)) {}

Simulator::Simulator(const Scenario& scenario, std::unique_ptr<LateralController> controller)
    : scenario_(scenario),
      model_(makeVehicleModel(scenario.model, scenario.vehicle)),
      sensors_(scenario.sensors, scenario.seed),
      controller_(std::move(controller)),
      actuator_(scenario.actuator),
      metrics_(scenario.road.laneWidth(), scenario.vehicle.width) {
  if (!controller_) throw std::invalid_argument("controller must not be null");
  if (scenario_.dt <= 0.0) throw std::invalid_argument("dt must be positive");
  state_.s = 0.0;
  state_.e_y = scenario_.initial_offset;
  state_.e_psi = scenario_.initial_heading;
  state_.vx = scenario_.speed;
  total_steps_ = std::lround(scenario_.duration / scenario_.dt);
  controller_->reset();
}

bool Simulator::done() const { return step_count_ >= total_steps_; }

void Simulator::step() {
  const double dt = scenario_.dt;
  const Road& road = scenario_.road;

  const SensorFrame frame = sensors_.update(time_, dt, state_, road);
  const double command = controller_->computeSteering(frame, dt);
  const double steer = actuator_.update(command, dt);
  const double force = scenario_.disturbanceAt(time_);

  // Inputs are those applied during the step; state fields are at the end of the step.
  TraceSample& s = last_;
  s.road_curvature = road.curvatureAt(state_.s);
  s.steer_command = command;
  s.steer_actual = steer;
  s.meas_offset = frame.lane.lateral_offset;
  s.meas_valid = frame.lane.valid;
  s.disturbance = force;
  s.mode = controller_->mode();
  s.controller = controller_->debug();

  model_->step(state_, road, steer, force, dt);
  ++step_count_;
  time_ = step_count_ * dt;

  s.time = time_;
  s.s = state_.s;
  s.e_y = state_.e_y;
  s.e_psi = state_.e_psi;
  s.lateral_accel = state_.lateral_accel;

  metrics_.add(time_, dt, state_, steer, actuator_.lastRate(), controller_->mode());
}

void Simulator::fillWorldPose(TraceSample& sample) const {
  const Pose2D p = scenario_.road.poseAt(sample.s);
  // Lateral offset is measured along the left-pointing road normal.
  sample.x = p.x - sample.e_y * std::sin(p.heading);
  sample.y = p.y + sample.e_y * std::cos(p.heading);
}

SimulationResult Simulator::run(const TraceCallback& trace, bool world_pose) {
  while (!done()) {
    step();
    if (trace) {
      if (world_pose) fillWorldPose(last_);
      trace(last_);
    }
  }
  SimulationResult result;
  result.metrics = metrics_.finish();
  result.verdict = evaluate(result.metrics, scenario_.envelope);
  result.camera_frames = sensors_.camera().framesCaptured();
  result.camera_frames_dropped = sensors_.camera().framesDropped();
  return result;
}

SimulationResult runScenario(const Scenario& scenario) {
  Simulator sim(scenario);
  return sim.run();
}

}  // namespace lanelock
