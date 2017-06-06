#include <memory>

#include <gflags/gflags.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/simple_car.h"
#include "drake/common/call_matlab.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
"Number of seconds to simulate.");

namespace drake {
namespace automotive {
namespace {


static std::unique_ptr<AutomotiveSimulator<double>>
SetupSimulator(bool is_playback_mode,
               const Eigen::MatrixXd* states = nullptr) {
  if (is_playback_mode) DRAKE_DEMAND(states != nullptr);
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry =
      std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"Dircol Test Dragway"}),
          1 , 100. , 2. , 0.);
  auto simulator = (is_playback_mode)
      ? std::make_unique<AutomotiveSimulator<double>>(
          std::make_unique<lcm::DrakeLcm>())
      : std::make_unique<AutomotiveSimulator<double>>();

  auto simulator_road = simulator->SetRoadGeometry(std::move(road_geometry));
  auto dragway_road_geometry =
      dynamic_cast<const maliput::dragway::RoadGeometry*>(simulator_road);

  const int lane_index = 0;

  // These defaults are ignored by the solver.
  if (is_playback_mode) {
    std::cout << " all_states(0) " << (*states)(0) << std::endl;
    std::cout << " all_states(1) " << (*states)(1) << std::endl;
    std::cout << " all_states(2) " << (*states)(2) << std::endl;
    std::cout << " all_states(3) " << (*states)(3) << std::endl;
    // DRAKE_ABORT();
  }

  // TODO: We can replace these values with garbage or just delete them
  // altogether.
  const double start_position_follower = (is_playback_mode) ? (*states)(2) : 5.;
  const double start_speed_follower = (is_playback_mode) ? (*states)(3) : 20.;
  const double start_position_leader = (is_playback_mode) ? (*states)(0) : 20.;
  const double speed_leader = (is_playback_mode) ? (*states)(1) : 10.;

  const auto& params_follower = CreateTrajectoryParamsForDragway(
      *dragway_road_geometry, lane_index, start_speed_follower,
      start_position_follower);
  simulator->AddIdmControlledPriusTrajectoryCar("following_trajectory_car",
                                                std::get<0>(params_follower),
                                                start_speed_follower,
                                                start_position_follower);
  const auto& params_leader = CreateTrajectoryParamsForDragway(
      *dragway_road_geometry, lane_index, speed_leader,
      start_position_leader);
  simulator->AddPriusTrajectoryCar("leading_trajectory_car",
                                   std::get<0>(params_leader),
                                   speed_leader,
                                   start_position_leader);

  // TODO(jadecastro): Double Check:
  return std::unique_ptr<AutomotiveSimulator<double>>(simulator.release());
}

int DoMain(void) {
  auto simulator = SetupSimulator(false /* is_playback_mode */);

  simulator->BuildAndInitialize();

  const auto& plant = simulator->GetDiagram();
  auto context = plant.CreateDefaultContext();

  // Set up a direct-collocation feasibility problem.
  const double duration = 5.;  // seconds
  const int kNumTimeSamples = 10;

  systems::DircolTrajectoryOptimization prog(&plant, *context, kNumTimeSamples,
                                             duration, duration);

  // Ensure that time intervals are evenly spaced.
  prog.AddTimeIntervalBounds(duration / (kNumTimeSamples - 1),
                             duration / (kNumTimeSamples - 1));

  prog.AddLinearConstraint(prog.initial_state()(0) >= 15.);  // s traffic0
  prog.AddLinearConstraint(prog.initial_state()(1) == 10.);  // s_dot traffic0
  prog.AddLinearConstraint(prog.initial_state()(2) == 0.5);  // s ego
  prog.AddLinearConstraint(prog.initial_state()(3) >= 20.);  // s_dot ego

  // Set state constaints for all time steps; constraints on state() with
  // indeterminate time-steps does not seem to work??
  for (int i{0}; i < kNumTimeSamples; ++i) {
    prog.AddLinearConstraint(prog.state(i)(0) >= 15.);  // s traffic0
    prog.AddLinearConstraint(prog.state(i)(1) == 10.);  // s_dot traffic0
    prog.AddLinearConstraint(prog.state(i)(2) >= 0.5);  // s ego
    prog.AddLinearConstraint(prog.state(i)(3) >= 5.);  // s_dot ego <-- should
                                                       // be epsilon.
  }

  prog.AddLinearConstraint(prog.final_state()(0) <= prog.final_state()(2) + 5.);

  EXPECT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);

  // Extract the initial context from prog.

  // Plot the solution using MATLAB.
  // Note: see lcm_call_matlab.h for instructions on viewing the plot.
  Eigen::MatrixXd inputs;
  Eigen::MatrixXd states;
  std::vector<double> times_out;
  prog.GetResultSamples(&inputs, &states, &times_out);
  common::CallMatlab("plot", states.row(0), states.row(0) - states.row(2));
  common::CallMatlab("xlabel", "s lead (m)");
  common::CallMatlab("ylabel", "s lead - s ego (m)");

  // Build another simulator with LCM capability and run in play-back mode.
  auto simulator_lcm = SetupSimulator(true /* is_playback_mode */, &states);
  simulator_lcm->Build();

  // Pipe the offending initial condition into AutomotiveSimulator.
  const auto& plant_lcm = simulator_lcm->GetDiagram();
  auto context_lcm = plant_lcm.CreateDefaultContext();
  context_lcm->get_mutable_continuous_state()->SetFromVector(states.col(0));

  simulator_lcm->Start(1. /* target realtime rate (seconds) */,
                       std::move(context_lcm) /* initial context */);
  simulator_lcm->StepBy(
      std::numeric_limits<double>::infinity() /* simulation time*/);

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::automotive::DoMain();
}
