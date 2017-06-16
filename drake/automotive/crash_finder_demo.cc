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
static constexpr double kLaneWidth = 3.;
static constexpr double kDragwayLength = 100.;

static std::unique_ptr<AutomotiveSimulator<double>>
SetupSimulator(bool is_playback_mode, int num_dragway_lanes = 1) {
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry =
      std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"Dircol Test Dragway"}),
          num_dragway_lanes , kDragwayLength, kLaneWidth,
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
  const maliput::api::Lane* to_lane = (num_dragway_lanes > 1)
      ? dragway_road_geometry->junction(0)->segment(0)->lane(lane_index)
      : nullptr;

  // TODO: We can just delete these altogether.
  const double start_position_ego = 5.;
  const double start_speed_ego = 20.;
  const auto& params_ego = CreateTrajectoryParamsForDragway(
      *dragway_road_geometry, lane_index, start_speed_ego,
      start_position_ego);
  simulator->AddIdmControlledCar("ego_car",
                                 std::get<0>(params_ego),
                                 start_speed_ego,
                                 start_position_ego,
                                 to_lane);
  if (num_dragway_lanes > 1) {
    for (int i{0}; i < 2; ++i) {
      const double start_position_traffic = 20.;
      const double speed_traffic = 10.;
      const int lane_index_traffic = i % num_dragway_lanes;
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

  if (num_dragway_lanes == 3) {
    // Instantiate a third traffic car: a SimpleCar governed by a
    // PurePursuitController.
    /*
    const double start_position_lane_changer = 5.;
    const double start_speed_lane_changer = 10.;
    const auto& params_lane_changer = CreateTrajectoryParamsForDragway(
        *dragway_road_geometry, lane_index, start_speed_lane_changer,
        start_position_lane_changer);
    simulator->AddIdmControlledCar("lane_changing_traffic_car",
                                   std::get<0>(params_lane_changer),
                                   start_speed_lane_changer,
                                   start_position_lane_changer,
                                   to_lane);
    */
  }


  // TODO(jadecastro): Double Check!
  return std::unique_ptr<AutomotiveSimulator<double>>(simulator.release());
}

int DoMain(void) {

  const int kNumLanes = 2;  // Number of lanes in the scenario.

  auto simulator = SetupSimulator(false /* is_playback_mode */, kNumLanes);

  simulator->BuildAndInitialize();

  const auto& plant = simulator->GetDiagram();
  auto context = plant.CreateDefaultContext();

  // Set up a direct-collocation feasibility problem.
  const double guess_duration = 3.5;  // seconds
  const int kNumTimeSamples = 30;
  const double kTrajectoryTimeLowerBound = 0.8 * guess_duration;
  const double kTrajectoryTimeUpperBound = 1.2 * guess_duration;

  systems::DircolTrajectoryOptimization prog(&plant, *context, kNumTimeSamples,
                                             kTrajectoryTimeLowerBound,
                                             kTrajectoryTimeUpperBound);

  // Ensure that time intervals are evenly spaced.
  prog.AddTimeIntervalBounds(kTrajectoryTimeLowerBound / (kNumTimeSamples - 1),
                             kTrajectoryTimeUpperBound / (kNumTimeSamples - 1));

  //.TODO(jadecastro): For lane change case, verify that the initial conditions
  // satisfy the preconditions for the "move to the left lane" action in MOBIL.

  // Most of the following constraints are fairness conditions that ensure the
  // optimizer does not return the trivial result where the cars are already in
  // collision.

  // Note: Some constraints, such as limits on the ego car velocity, are added
  // for the sole purpose of preventing infeasible solutions that would
  // otherwise result.  This is often a result of the optimizer encountering a
  // state in one of the car model/planner subsystems that give rise to
  // behaviors for which the AutoDiffXd derivatives() are undefined.  Other
  // constraints, such as the limits on relative position and speed of Traffic
  // Car 1 in the three-lane example, are applied to ensure that the scenario is
  // consistent with conjunctive failure conditions (i.e. the final "goal" set
  // is a polytope).

  // Note: The following scenarios are limited to scenarios with continuous
  // states (and thus free of discrete decisions and hybrid continuous/discrete
  // states).  For instance, MOBIL with politeness parameter `p` set to zero
  // qualifies, provided that the initial conditions follow from
  // MOBIL-admissible preconditions (i.e. the desired lane is not the current
  // one over the provided range initial conditions).

  // TODO(jadecastro): Caution: The following is a super-hackish way to fix the
  // constraints for each scenario.  How best to obtain correspondences for all
  // the indices?
  VectorX<double> vect0(8);
  VectorX<double> vectf(8);
  if (kNumLanes == 3) {
    vect0 << 5., 5., 60., 5., 50., -0.5 * kLaneWidth, 0., 5.5;
    vectf << 60., 5., 80., 5., 60., 0.5 * kLaneWidth, 0., 5.5;

    // Begin with a reasonable spacing between cars.
    // -- Traffic Car 2 (left lane)
    /*
    prog.AddLinearConstraint(prog.initial_state()(0) >=
                             prog.initial_state()(4));  // x traffic2
    prog.AddLinearConstraint(prog.initial_state()(0) <= kDragwayLength);
                                                        // x traffic2
    prog.AddLinearConstraint(prog.initial_state()(1) == kLaneWidth);
                                                              // y traffic2
    prog.AddLinearConstraint(prog.initial_state()(2) == 0.);  // heading
                                                              // traffic2
    prog.AddLinearConstraint(prog.initial_state()(3) ==
                             10.);  // velocity traffic2
    */
    // TESTING:
    // prog.AddLinearConstraint(prog.initial_state()(0) == vect0(0));  // x ego
    //prog.AddLinearConstraint(prog.initial_state()(1) == -kLaneWidth);  // y ego
    //prog.AddLinearConstraint(prog.initial_state()(2) == 0.);  // heading ego
    // prog.AddLinearConstraint(prog.initial_state()(1) == vect0(1));
                                                                // velocity ego
    // -- Traffic Car 1 (middle lane)
    // prog.AddLinearConstraint(prog.initial_state()(4) + 5. <=
    //                          prog.initial_state()(8));  // s traffic1
    // prog.AddLinearConstraint(prog.initial_state()(4) <= 47.5);  // s traffic1
    //prog.AddLinearConstraint(prog.initial_state()(5) <= 5.);// s_dot traffic1
    prog.AddLinearConstraint(prog.initial_state()(0) >= vect0(0));
    prog.AddLinearConstraint(prog.initial_state()(1) >= vect0(1));
                                                              // s_dot traffic1
    // -- Traffic Car 0 (right lane)
    prog.AddLinearConstraint(prog.initial_state()(2) >= vect0(2));
                                                              // s traffic0
    prog.AddLinearConstraint(prog.initial_state()(3) == vect0(3));
                                                              // s_dot traffic0
    // -- Ego Car (right lane)
    prog.AddLinearConstraint(prog.initial_state()(4) == vect0(4));  // x ego
    prog.AddLinearConstraint(prog.initial_state()(5) == vect0(5));  // y ego
    prog.AddLinearConstraint(prog.initial_state()(6) == vect0(6));
                                                               // heading ego
    prog.AddLinearConstraint(prog.initial_state()(7) == vect0(7));
                                                               // velocity ego

    // Set state constaints for all time steps; constraints on state() with
    // indeterminate time-steps does not seem to work??
    for (int i{0}; i < kNumTimeSamples; ++i) {
      // -- Traffic Car 2 (moving to the middle lane) -- Always ahead of and
      // faster than Traffic Car 1.
      /*
      prog.AddLinearConstraint(prog.state(i)(0) >= prog.state(i)(4));
                                                           // x traffic2
      prog.AddLinearConstraint(prog.state(i)(0) <= kDragwayLength);
                                                           // x traffic2
      prog.AddLinearConstraint(prog.state(i)(1) <= kLaneWidth);  // y traffic2
      //prog.AddLinearConstraint(prog.state(i)(2) >= 0.);  // heading traffic2
      prog.AddLinearConstraint(prog.state(i)(3) <= 11.);  // velocity traffic2
      prog.AddLinearConstraint(prog.state(i)(3) >= 9.);  // velocity traffic2
      */
      // TESTING:
      // prog.AddLinearConstraint(prog.state(i)(0) >= 40.);  // x ego
      //prog.AddLinearConstraint(prog.state(i)(1) >= -kLaneWidth);  // y ego
      // prog.AddLinearConstraint(prog.state(i)(10) >= 0.);  // heading ego
      //prog.AddLinearConstraint(prog.state(i)(3) <= 7.);  // velocity ego
      // prog.AddLinearConstraint(prog.state(i)(1) == 0.);  // velocity ego

      // -- Traffic Car 1 (middle lane) -- Always behind the ego and speed
      // limited.
      //prog.AddLinearConstraint(prog.state(i)(4) + 3. <= prog.state(i)(8));
                                                         // s traffic1
      //prog.AddLinearConstraint(prog.state(i)(5) <= 5.);  // s_dot traffic1
      prog.AddLinearConstraint(prog.state(i)(0) >= 5.);  // s traffic1
      prog.AddLinearConstraint(prog.state(i)(1) >= 5.);  // s_dot traffic1
      // -- Traffic Car 0 (right lane)
      prog.AddLinearConstraint(prog.state(i)(2) >= 60.);  // s traffic0
      prog.AddLinearConstraint(prog.state(i)(3) == 5.);  // s_dot traffic0
      // -- Ego Car (moving to the middle lane)
      prog.AddLinearConstraint(prog.state(i)(4) >= 50.);  // x ego
      prog.AddLinearConstraint(prog.state(i)(5) >= 0.5 * kLaneWidth);  // y ego
      //prog.AddLinearConstraint(prog.state(i)(10) >= 0.);  // heading ego
      prog.AddLinearConstraint(prog.state(i)(7) <= 7.);  // velocity ego
      prog.AddLinearConstraint(prog.state(i)(7) >= 4.);  // velocity ego
    }

    // Unsafe criterion: Find a collision with Traffic Car 2 merging into the
    // middle lane.  Assume for now that, irrespective of its orientation, the
    // traffic car occupies a lane-aligned bounding box.

    // y ego >= y traffic 2 - 0.5 * kLaneWidth && x ego <= x traffic2 + 2.5 &&
    // x ego >= x traffic2 - 2.5
    prog.AddLinearConstraint(prog.final_state()(5) >= 0.5 * kLaneWidth);
                             //prog.final_state()(1) - 0.5 * kLaneWidth);
    prog.AddLinearConstraint(prog.final_state()(4) >=
                             prog.final_state()(0) - 2.5);
    prog.AddLinearConstraint(prog.final_state()(4) <=
                             prog.final_state()(0) + 2.5);

   } else if (kNumLanes == 2) {
    vect0 << 5., 5., 60., 5., 50., -kLaneWidth, 0., 5.5;
    vectf << 60., 5., 80., 5., 60., -0.5 * kLaneWidth, 0., 5.5;

    DRAKE_DEMAND(kNumTrafficCarsForLaneChange == 2);
    // Begin with a reasonable spacing between cars.
    // -- Traffic Car 1 (left lane)
    prog.AddLinearConstraint(prog.initial_state()(0) >= 5.);  // s traffic1
    //prog.AddLinearConstraint(prog.initial_state()(0) <= 47.5);  // s traffic1
    prog.AddLinearConstraint(prog.initial_state()(1) >= 5.);  // s_dot traffic1
    // -- Traffic Car 0 (right lane)
    prog.AddLinearConstraint(prog.initial_state()(2) >= 60.);  // s traffic0
    prog.AddLinearConstraint(prog.initial_state()(3) == 5.);  // s_dot traffic0
    // -- Ego Car (right lane)
    prog.AddLinearConstraint(prog.initial_state()(4) == 50.);  // x ego
    prog.AddLinearConstraint(prog.initial_state()(5) == -0.5 * kLaneWidth);
                                                               // y ego
    prog.AddLinearConstraint(prog.initial_state()(6) == 0.);  // heading ego
    prog.AddLinearConstraint(prog.initial_state()(7) == 5.5);  // velocity ego

    // Set state constaints for all time steps; constraints on state() with
    // indeterminate time-steps does not seem to work??
    for (int i{0}; i < kNumTimeSamples; ++i) {
      //prog.AddLinearConstraint(prog.state()(0) >= prog.state()(2));
      // -- Traffic Car 1 (left lane)
      prog.AddLinearConstraint(prog.state(i)(0) >= 5.);  // s traffic0
      prog.AddLinearConstraint(prog.state(i)(1) >= 5.);  // s_dot traffic0
      // -- Traffic Car 0 (right lane)
      prog.AddLinearConstraint(prog.state(i)(2) >= 60.);  // s traffic1
      prog.AddLinearConstraint(prog.state(i)(3) == 5.);  // s_dot traffic1
      // -- Ego Car (moving to the left lane)
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

   } else if (kNumLanes == 1) {
    // Begin with a reasonable spacing between cars.
    prog.AddLinearConstraint(prog.initial_state()(0) >=
                             prog.initial_state()(2) + 2.6);
    // -- Traffic Car
    //prog.AddLinearConstraint(prog.initial_state()(0) >= 15.);  // s traffic0
    prog.AddLinearConstraint(prog.initial_state()(1) == 1.);  // s_dot traffic0
    // -- Ego Car
    prog.AddLinearConstraint(prog.initial_state()(2) == 0.5);  // s ego
    prog.AddLinearConstraint(prog.initial_state()(3) >= 20.);  // s_dot ego

    // Set state constaints for all time steps; constraints on state() with
    // indeterminate time-steps does not seem to work??
    for (int i{0}; i < kNumTimeSamples; ++i) {
      //prog.AddLinearConstraint(prog.state()(0) >= prog.state()(2));
      // -- Traffic Car
      prog.AddLinearConstraint(prog.state(i)(0) >= 2.);  // s traffic0
      prog.AddLinearConstraint(prog.state(i)(1) == 1.);  // s_dot traffic0
      // -- Ego Car
      prog.AddLinearConstraint(prog.state(i)(2) >= 0.5);  // s ego
      prog.AddLinearConstraint(prog.state(i)(3) >= 0.5);  // s_dot ego <--
                                                          // should be epsilon.
    }

    // Unsafe criterion: Find a collision with the lead car.
    prog.AddLinearConstraint(prog.final_state()(0) <=
                             prog.final_state()(2) + 2.5);
  }

  systems::BasicVector<double> x0(vect0);
  systems::BasicVector<double> xf(vectf);
  auto guess_state_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      {0, guess_duration}, {x0.get_value(), xf.get_value()});

  const auto result = prog.SolveTraj(guess_duration,
                                     PiecewisePolynomial<double>(),
                                     guess_state_trajectory);

  // Extract the initial context from prog and plot the solution using MATLAB.
  // Note: see lcm_call_matlab.h for instructions on viewing the plot.
  Eigen::MatrixXd inputs;
  Eigen::MatrixXd states;
  std::vector<double> times_out;
  prog.GetResultSamples(&inputs, &states, &times_out);
  if (kNumLanes == 3) {
    common::CallMatlab("figure");
    common::CallMatlab("plot", states.row(4), states.row(5));
    common::CallMatlab("xlabel", "x ego (m)");
    common::CallMatlab("ylabel", "y ego (m)");
    common::CallMatlab("figure");
    common::CallMatlab("plot", states.row(0), states.row(1));
    common::CallMatlab("xlabel", "x traffic2 (m)");
    common::CallMatlab("ylabel", "y traffic2 (m)");
  } else if (kNumLanes == 2) {
    common::CallMatlab("figure");
    common::CallMatlab("plot", states.row(4), states.row(5));
    common::CallMatlab("xlabel", "x ego (m)");
    common::CallMatlab("ylabel", "y ego (m)");
    common::CallMatlab("figure");
    common::CallMatlab("plot", states.row(0), states.row(2));
    common::CallMatlab("xlabel", "x traffic1 (m)");
    common::CallMatlab("ylabel", "x traffic0 (m)");
  } else if (kNumLanes == 1) {
    common::CallMatlab("plot", states.row(0), states.row(0) - states.row(2));
    common::CallMatlab("xlabel", "s lead (m)");
    common::CallMatlab("ylabel", "s lead - s ego (m)");
  }

  // Dump the entire solution result to the screen.
  for (int i{0}; i < states.size(); ++i) {
    std::cout << " states(" << i << ") " << states(i) << std::endl;
  }

  std::cout << " SOLUTION RESULT: " << result << std::endl;

  if (result == solvers::SolutionResult::kSolutionFound) {
    // Build another simulator with LCM capability and run in play-back mode.
    auto simulator_lcm = SetupSimulator(true /* is_playback_mode */,
                                        kNumLanes);
    simulator_lcm->Build();

    // Pipe the offending initial condition into AutomotiveSimulator.
    // TODO(jadecastro): Set up a trajectory-playback Diagram.
    const auto& plant_lcm = simulator_lcm->GetDiagram();
    auto context_lcm = plant_lcm.CreateDefaultContext();
    context_lcm->get_mutable_continuous_state()->SetFromVector(states.col(0));

    simulator_lcm->Start(1., std::move(context_lcm));
    simulator_lcm->StepBy(10.);
  }

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::automotive::DoMain();
}
