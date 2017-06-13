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

static constexpr int kNumTrafficCarsForLaneChange = 2;
static constexpr int kNumDragwayLanesForLaneChange = 2;
static constexpr double kLaneWidth = 3.;

static std::unique_ptr<AutomotiveSimulator<double>>
SetupSimulator(bool is_playback_mode, bool is_lane_change = false) {
  const int num_dragway_lanes =
      is_lane_change ? kNumDragwayLanesForLaneChange : 1;
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry =
      std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"Dircol Test Dragway"}),
          num_dragway_lanes , 100. /* dragway length */, kLaneWidth,
          0. /* shoulder width */, 5. /* maximum_height */,
          std::numeric_limits<double>::epsilon() /* linear_tolerance */,
          std::numeric_limits<double>::epsilon() /* angular_tolerance */);

  auto simulator = (is_playback_mode)
      ? std::make_unique<AutomotiveSimulator<double>>(
          std::make_unique<lcm::DrakeLcm>())
      : std::make_unique<AutomotiveSimulator<double>>();

  auto simulator_road = simulator->SetRoadGeometry(std::move(road_geometry));
  auto dragway_road_geometry =
      dynamic_cast<const maliput::dragway::RoadGeometry*>(simulator_road);

  const int lane_index = 0;
  const maliput::api::Lane* to_lane = is_lane_change
      ? dragway_road_geometry->junction(0)->segment(0)->lane(lane_index)
      : nullptr;

  // TODO: We can just delete these altogether.
  const double start_position_follower = 5.;
  const double start_speed_follower = 20.;

  const auto& params_follower = CreateTrajectoryParamsForDragway(
      *dragway_road_geometry, lane_index, start_speed_follower,
      start_position_follower);
  simulator->AddIdmControlledCar("following_trajectory_car",
                                 std::get<0>(params_follower),
                                 start_speed_follower,
                                 start_position_follower,
                                 to_lane);
  if (is_lane_change) {
    for (int i{0}; i < kNumTrafficCarsForLaneChange; ++i) {
      const double start_position_traffic = 20.;
      const double speed_traffic = 10.;
      const int lane_index_traffic = i % kNumDragwayLanesForLaneChange;

      const auto& params_traffic = CreateTrajectoryParamsForDragway(
          *dragway_road_geometry, lane_index_traffic, speed_traffic,
          start_position_traffic);
      simulator->AddPriusTrajectoryCar(
          "traffic_trajectory_car_" + std::to_string(i),
          std::get<0>(params_traffic), speed_traffic, start_position_traffic);
    }
  } else {
    const double start_position_leader = 20.;
    const double speed_leader = 10.;

    const auto& params_leader = CreateTrajectoryParamsForDragway(
        *dragway_road_geometry, lane_index, speed_leader,
        start_position_leader);
    simulator->AddPriusTrajectoryCar("leading_trajectory_car",
                                     std::get<0>(params_leader),
                                     speed_leader,
                                     start_position_leader);
  }
  // TODO(jadecastro): Double Check!
  return std::unique_ptr<AutomotiveSimulator<double>>(simulator.release());
}

int DoMain(void) {

  const bool is_lane_change = true;
  auto simulator = SetupSimulator(false /* is_playback_mode */, is_lane_change);

  simulator->BuildAndInitialize();

  const auto& plant = simulator->GetDiagram();
  auto context = plant.CreateDefaultContext();

  // Set up a direct-collocation feasibility problem.
  const double duration = 3.5;  // seconds
  const int kNumTimeSamples = 30;

  systems::DircolTrajectoryOptimization prog(&plant, *context, kNumTimeSamples,
                                             duration, duration);

  // Ensure that time intervals are evenly spaced.
  prog.AddTimeIntervalBounds(duration / (kNumTimeSamples - 1),
                             duration / (kNumTimeSamples - 1));

  //.TODO(jadecastro): For lane change case, verify that the initial conditions
  // satisfy the preconditions for the "move to the left lane" action in MOBIL.

  // TODO(jadecastro): How best to obtain correspondences for all the indices?
  if (is_lane_change) {
    DRAKE_DEMAND(kNumDragwayLanesForLaneChange == 2);
    DRAKE_DEMAND(kNumTrafficCarsForLaneChange == 2);
    // Begin with a reasonable spacing between cars.
    // Traffic Car 1 (left lane)
    prog.AddLinearConstraint(prog.initial_state()(0) >= 5.);  // s traffic1
    //prog.AddLinearConstraint(prog.initial_state()(0) <= 47.5);  // s traffic1
    prog.AddLinearConstraint(prog.initial_state()(1) >= 5.);  // s_dot traffic1
    // Traffic Car 0 (right lane)
    prog.AddLinearConstraint(prog.initial_state()(2) >= 60.);  // s traffic0
    prog.AddLinearConstraint(prog.initial_state()(3) == 5.);  // s_dot traffic0
    // Ego Car (right lane)
    prog.AddLinearConstraint(prog.initial_state()(4) == 50.);  // x ego
    prog.AddLinearConstraint(prog.initial_state()(5) == -0.5 * kLaneWidth);
                                                               // y ego
    //  ... Ego starts out in the right lane.
    prog.AddLinearConstraint(prog.initial_state()(6) == 0.);  // heading ego
    prog.AddLinearConstraint(prog.initial_state()(7) == 5.5);  // velocity ego

    // Set state constaints for all time steps; constraints on state() with
    // indeterminate time-steps does not seem to work??
    for (int i{0}; i < kNumTimeSamples; ++i) {
      //prog.AddLinearConstraint(prog.state()(0) >= prog.state()(2));
      // Traffic Car 1 (left lane)
      prog.AddLinearConstraint(prog.state(i)(0) >= 5.);  // s traffic0
      prog.AddLinearConstraint(prog.state(i)(1) >= 5.);  // s_dot traffic0
      // Traffic Car 0 (right lane)
      prog.AddLinearConstraint(prog.state(i)(2) >= 60.);  // s traffic1
      prog.AddLinearConstraint(prog.state(i)(3) == 5.);  // s_dot traffic1
      // Ego Car
      prog.AddLinearConstraint(prog.state(i)(4) >= 50.);  // x ego
      prog.AddLinearConstraint(prog.state(i)(5) >= -0.5 * kLaneWidth);  // y ego
      //prog.AddLinearConstraint(prog.state(i)(6) >= 0.);  // heading ego
      prog.AddLinearConstraint(prog.state(i)(7) <= 7.);  // velocity ego
      prog.AddLinearConstraint(prog.state(i)(7) >= 4.);  // velocity ego
    }
    // Unsafe criterion: Find a collision with the traffic car in left lane.
    // y ego >= 0. && x ego <= x traffic1 + 2.5 && x ego >= x traffic1 - 2.5
    prog.AddLinearConstraint(prog.final_state()(5) >= 0.);
    prog.AddLinearConstraint(prog.final_state()(4) >=
                             prog.final_state()(0) - 2.5);
    prog.AddLinearConstraint(prog.final_state()(4) <=
                             prog.final_state()(0) + 2.5);
  } else {
    // Begin with a reasonable spacing between cars.
    prog.AddLinearConstraint(prog.initial_state()(0) >=
                             prog.initial_state()(2) + 2.6);
    // Traffic Car
    //prog.AddLinearConstraint(prog.initial_state()(0) >= 15.);  // s traffic0
    prog.AddLinearConstraint(prog.initial_state()(1) == 1.);  // s_dot traffic0
    // Ego Car
    prog.AddLinearConstraint(prog.initial_state()(2) == 0.5);  // s ego
    prog.AddLinearConstraint(prog.initial_state()(3) >= 20.);  // s_dot ego

    // Set state constaints for all time steps; constraints on state() with
    // indeterminate time-steps does not seem to work??
    for (int i{0}; i < kNumTimeSamples; ++i) {
      //prog.AddLinearConstraint(prog.state()(0) >= prog.state()(2));
      // Traffic Car
      prog.AddLinearConstraint(prog.state(i)(0) >= 2.);  // s traffic0
      prog.AddLinearConstraint(prog.state(i)(1) == 1.);  // s_dot traffic0
      // Ego Car
      prog.AddLinearConstraint(prog.state(i)(2) >= 0.5);  // s ego
      prog.AddLinearConstraint(prog.state(i)(3) >= 0.5);  // s_dot ego <--
                                                          // should be epsilon.
    }
    prog.AddLinearConstraint(prog.final_state()(0) <=
                             prog.final_state()(2) + 2.5);
  }

  EXPECT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);

  // Extract the initial context from prog.

  // Plot the solution using MATLAB.
  // Note: see lcm_call_matlab.h for instructions on viewing the plot.
  Eigen::MatrixXd inputs;
  Eigen::MatrixXd states;
  std::vector<double> times_out;
  prog.GetResultSamples(&inputs, &states, &times_out);
  if (is_lane_change) {
    common::CallMatlab("figure");
    common::CallMatlab("plot", states.row(4), states.row(5));
    common::CallMatlab("xlabel", "x ego (m)");
    common::CallMatlab("ylabel", "y ego (m)");
    common::CallMatlab("figure");
    common::CallMatlab("plot", states.row(0), states.row(2));
    common::CallMatlab("xlabel", "x traffic1 (m)");
    common::CallMatlab("ylabel", "x traffic0 (m)");
  } else {
    common::CallMatlab("plot", states.row(0), states.row(0) - states.row(2));
    common::CallMatlab("xlabel", "s lead (m)");
    common::CallMatlab("ylabel", "s lead - s ego (m)");
  }

  for (int i{0}; i < states.size(); ++i) {
    std::cout << " states(" << i << ") " << states(i) << std::endl;
  }

  // Build another simulator with LCM capability and run in play-back mode.
  auto simulator_lcm = SetupSimulator(true /* is_playback_mode */,
                                      is_lane_change);
  simulator_lcm->Build();

  // Pipe the offending initial condition into AutomotiveSimulator.
  
  const auto& plant_lcm = simulator_lcm->GetDiagram();
  auto context_lcm = plant_lcm.CreateDefaultContext();
  context_lcm->get_mutable_continuous_state()->SetFromVector(states.col(0));

  simulator_lcm->Start(1., std::move(context_lcm));
  simulator_lcm->StepBy(10.);
  
  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::automotive::DoMain();
}
