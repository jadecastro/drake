#include <memory>

#include <gflags/gflags.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/simple_car.h"
#include "drake/common/proto/call_python.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
"Number of seconds to simulate.");

namespace drake {
namespace automotive {
namespace {

using common::CallPython;
using systems::trajectory_optimization::DirectCollocation;

static constexpr int kNumTrafficCarsForLaneChange = 2;
static constexpr double kLaneWidth = 3.;
static constexpr double kDragwayLength = 150.;

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
      : std::make_unique<AutomotiveSimulator<double>>(nullptr);

  auto simulator_road = simulator->SetRoadGeometry(std::move(road_geometry));
  auto dragway_road_geometry =
      dynamic_cast<const maliput::dragway::RoadGeometry*>(simulator_road);

  // TODO: We can just delete these altogether.
  // Set the initial states.
  const int kEgoStartLaneIndex = 0;
  const int kEgoGoalLaneIndex = 0;

  const maliput::api::Lane* ego_start_lane =
      dragway_road_geometry->junction(0)->segment(0)->lane(kEgoStartLaneIndex);
  const maliput::api::Lane* ego_goal_lane =
      dragway_road_geometry->junction(0)->segment(0)->lane(kEgoGoalLaneIndex);

  const double start_s_ego = 5.;
  const double start_speed_ego = 20.;
  const maliput::api::GeoPosition start_position_ego =
      ego_start_lane->ToGeoPosition({start_s_ego, 0., 0.});

  SimpleCarState<double> ego_initial_state;
  // The following presumes we are on a dragway, in which x -> s, y -> r.
  ego_initial_state.set_x(start_position_ego.x());
  ego_initial_state.set_y(start_position_ego.y());
  ego_initial_state.set_heading(0.);
  ego_initial_state.set_velocity(start_speed_ego);

  simulator->AddIdmControlledCar("ego_car",
                                 true /* move along the "s"-direction */,
                                 ego_initial_state, ego_goal_lane);

  if (num_dragway_lanes > 1) {
    for (int i{0}; i < num_dragway_lanes; ++i) {
      const int lane_index_traffic = i % num_dragway_lanes;

      const double start_s_traffic = 20.;
      const double speed_traffic = 10.;
      //const int lane_index_traffic = (i == 0) ? 0 : 2;

      const auto& params_traffic = CreateTrajectoryParamsForDragway(
          *dragway_road_geometry, lane_index_traffic, speed_traffic,
          start_s_traffic);
      simulator->AddPriusTrajectoryCar(
          "traffic_trajectory_car_" + std::to_string(i),
          std::get<0>(params_traffic), speed_traffic, start_s_traffic);
    }
  } else {
    const double start_s_leader = 1.;
    const double speed_leader = 10.;

    const auto& params_leader = CreateTrajectoryParamsForDragway(
        *dragway_road_geometry, kEgoStartLaneIndex, speed_leader,
        start_s_leader);
    simulator->AddPriusTrajectoryCar("leading_trajectory_car",
                                     std::get<0>(params_leader), speed_leader,
                                     start_s_leader);
  }

  if (num_dragway_lanes == 3) {
    // Instantiate a third traffic car: a SimpleCar governed by a
    // PurePursuitController.
    const double start_s_lc = 5.;
    const double start_speed_lc = 10.;
    const maliput::api::GeoPosition start_position_lc =
        ego_start_lane->ToGeoPosition({start_s_lc, 0., 0.});

    SimpleCarState<double> lc_initial_state;
    // The following presumes we are on a dragway, in which x -> s, y -> r.
    lc_initial_state.set_x(start_position_lc.x());
    lc_initial_state.set_y(start_position_lc.y());
    lc_initial_state.set_heading(0.);
    lc_initial_state.set_velocity(start_speed_lc);

    simulator->AddIdmControlledCar("lane_changer",
                                   true /* move along the "s"-direction */,
                                   lc_initial_state, ego_goal_lane);
  }


  // TODO(jadecastro): Double Check!
  return std::unique_ptr<AutomotiveSimulator<double>>(simulator.release());
}

int DoMain(void) {

  const int kNumLanes = 3;  // Number of lanes in the scenario.

  auto simulator = SetupSimulator(false /* is_playback_mode */, kNumLanes);

  simulator->BuildAndInitialize();

  const auto& plant = simulator->GetDiagram();
  auto context = plant.CreateDefaultContext();

  // Set up a direct-collocation feasibility problem.
  const double guess_duration = 2.;  // seconds
  const int kNumTimeSamples = 100;
  const double kTrajectoryTimeLowerBound = 0.8 * guess_duration;
  const double kTrajectoryTimeUpperBound = 1.2 * guess_duration;

  DirectCollocation prog(&plant, *context, kNumTimeSamples,
                         kTrajectoryTimeLowerBound, kTrajectoryTimeUpperBound);

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
  const double delta_y =  0.; //0.5 * kLaneWidth;
  VectorX<double> vect0(14);
  VectorX<double> vectf(14);
  if (kNumLanes == 3) {
    vect0 << 20., 1. * kLaneWidth + delta_y, 0., 10., 30., 5., 60., 8.,
        40., 5., 30., -1. * kLaneWidth + delta_y, 0., 5.5;
    vectf << 30., 0.4 * kLaneWidth + delta_y, 0., 10., 40., 5., 80., 8.,
        60., 5., 40., -0.6 * kLaneWidth + delta_y, 0., 5.5;

    // Begin with a reasonable spacing between cars.
    // -- Traffic Car 3 (left lane)
    prog.AddLinearConstraint(prog.initial_state()(0) >= vect0(0));
                             //prog.initial_state()(4));  // x traffic2
    //prog.AddLinearConstraint(prog.initial_state()(0) <= kDragwayLength);
                                                        // x traffic2
    prog.AddLinearConstraint(prog.initial_state()(1) == kLaneWidth + delta_y);
                                                              // y traffic2
    prog.AddLinearConstraint(prog.initial_state()(2) == vect0(2));  // heading
                                                              // traffic2
    prog.AddLinearConstraint(prog.initial_state()(3) >= vect0(3));
                                                           // velocity traffic2
    // -- Traffic Car 2 (left lane)
    prog.AddLinearConstraint(prog.initial_state()(4) >= vect0(4));
    prog.AddLinearConstraint(prog.initial_state()(4) <= vect0(4) + 5.);
    prog.AddLinearConstraint(prog.initial_state()(5) == vect0(5));
    // -- Traffic Car 1 (middle lane)
    // prog.AddLinearConstraint(prog.initial_state()(4) + 5. <=
    //                          prog.initial_state()(8));  // s traffic1
    // prog.AddLinearConstraint(prog.initial_state()(4) <= 47.5);  // s traffic1
    //prog.AddLinearConstraint(prog.initial_state()(5) <= 5.);// s_dot traffic1
    prog.AddLinearConstraint(prog.initial_state()(6) >= vect0(6));
    prog.AddLinearConstraint(prog.initial_state()(6) <= vect0(6) + 5.);
    prog.AddLinearConstraint(prog.initial_state()(7) >= vect0(7));
                                                              // s_dot traffic1
    // -- Traffic Car 0 (right lane)
    prog.AddLinearConstraint(prog.initial_state()(8) >= vect0(8));
                                                              // s traffic0
    prog.AddLinearConstraint(prog.initial_state()(8) <= vect0(8) + 5.);
    prog.AddLinearConstraint(prog.initial_state()(9) == vect0(9));
                                                              // s_dot traffic0
    // -- Ego Car (right lane)
    prog.AddLinearConstraint(prog.initial_state()(10) == vect0(10));  // x ego
    prog.AddLinearConstraint(prog.initial_state()(11) ==
                             -1. * kLaneWidth + delta_y);  // y ego
    prog.AddLinearConstraint(prog.initial_state()(12) == vect0(12));
                                                               // heading ego
    prog.AddLinearConstraint(prog.initial_state()(13) == vect0(13));
                                                               // velocity ego

    // Set state constaints for all time steps.
    for (int i{0}; i < kNumTimeSamples; ++i) {
      // -- Traffic Car 3 (moving to the middle lane)
      prog.AddLinearConstraint(prog.state(i)(0) >= 20.);
                                                           // x traffic2
      //prog.AddLinearConstraint(prog.state(i)(0) <= kDragwayLength);
                                                           // x traffic2
      prog.AddLinearConstraint(prog.state(i)(1) <= kLaneWidth + delta_y);
                                                           // y traffic2
      //prog.AddLinearConstraint(prog.state(i)(2) >= 0.);  // heading traffic2
      prog.AddLinearConstraint(prog.state(i)(3) <= 15.);  // velocity traffic2
      prog.AddLinearConstraint(prog.state(i)(3) >= 4.);  // velocity traffic2

      // -- Traffic Car 2 (left lane)
      prog.AddLinearConstraint(prog.state(i)(4) >= 20.);  // s traffic1
      prog.AddLinearConstraint(prog.state(i)(5) >= 5.);  // s_dot traffic1

      // -- Traffic Car 1 (middle lane)
      //prog.AddLinearConstraint(prog.state(i)(4) + 3. <= prog.state(i)(8));
                                                         // s traffic1
      //prog.AddLinearConstraint(prog.state(i)(5) <= 5.);  // s_dot traffic1
      prog.AddLinearConstraint(prog.state(i)(6) >= 70.);  // s traffic1
      prog.AddLinearConstraint(prog.state(i)(7) >= 5.);  // s_dot traffic1
      // -- Traffic Car 0 (right lane)
      prog.AddLinearConstraint(prog.state(i)(8) >= 40.);  // s traffic0
      prog.AddLinearConstraint(prog.state(i)(9) == 5.);  // s_dot traffic0
      // -- Ego Car (moving to the middle lane)
      prog.AddLinearConstraint(prog.state(i)(10) >= 30.);  // x ego
      prog.AddLinearConstraint(prog.state(i)(11) >= -kLaneWidth + delta_y);
                                                          // y ego
      //prog.AddLinearConstraint(prog.state(i)(10) >= 0.);  // heading ego
      prog.AddLinearConstraint(prog.state(i)(13) <= 7.);  // velocity ego
      prog.AddLinearConstraint(prog.state(i)(13) >= 4.);  // velocity ego
    }

    // Unsafe criterion: Find a collision with Traffic Car 2 merging into the
    // middle lane.  Assume for now that, irrespective of its orientation, the
    // traffic car occupies a lane-aligned bounding box.

    // y ego >= y traffic 2 - 0.5 * kLaneWidth && x ego <= x traffic2 + 2.5 &&
    // x ego >= x traffic2 - 2.5
    prog.AddLinearConstraint(prog.final_state()(11) >=
                             //-0.51 * kLaneWidth + delta_y);
                             prog.final_state()(1) - 0.5 * kLaneWidth);
    prog.AddLinearConstraint(prog.final_state()(10) >=
                             prog.final_state()(0) - 2.5);
    prog.AddLinearConstraint(prog.final_state()(10) <=
                             prog.final_state()(0) + 2.5);

    // Add a cost to encourage solutions that minimize the distance to a crash.
    //prog.AddRunningCost((prog.state()(4) - prog.state()(0)) *
    //                    (prog.state()(4) - prog.state()(0)) +
    //                    (prog.state()(5) - (-0.5 * kLaneWidth + delta_y)) *
    //                    (prog.state()(5) - (-0.5 * kLaneWidth + delta_y)));

   } else if (kNumLanes == 2) {
    vect0 << 5., 5., 60., 5., 50., -0.5 * kLaneWidth + delta_y, 0., 5.5;
    vectf << 60. - 2.5, 5., 80., 5., 60., -0.01 * kLaneWidth + delta_y, 0., 5.5;

    DRAKE_DEMAND(kNumTrafficCarsForLaneChange == 2);
    // Begin with a reasonable spacing between cars.
    // -- Traffic Car 1 (left lane)
    prog.AddLinearConstraint(prog.initial_state()(0) >= 5.);  // s traffic1
    //prog.AddLinearConstraint(prog.initial_state()(0) <= 47.5);  // s traffic1
    prog.AddLinearConstraint(prog.initial_state()(1) >= 5.);  // s_dot traffic1
    // -- Traffic Car 0 (right lane)
    prog.AddLinearConstraint(prog.initial_state()(2) == 60.);  // s traffic0
    prog.AddLinearConstraint(prog.initial_state()(3) == 5.);  // s_dot traffic0
    // -- Ego Car (right lane)
    prog.AddLinearConstraint(prog.initial_state()(4) == 50.);  // x ego
    prog.AddLinearConstraint(prog.initial_state()(5) ==
                             -0.5 * kLaneWidth + delta_y);
                                                               // y ego
    prog.AddLinearConstraint(prog.initial_state()(6) == 0.);  // heading ego
    prog.AddLinearConstraint(prog.initial_state()(7) == 5.5);  // velocity ego

    // Set state constaints for all time steps.
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
      prog.AddLinearConstraint(prog.state(i)(5) >=
                               -0.5 * kLaneWidth + delta_y);  // y ego
      //prog.AddLinearConstraint(prog.state(i)(6) >= 0.);  // heading ego
      prog.AddLinearConstraint(prog.state(i)(7) <= 7.);  // velocity ego
      prog.AddLinearConstraint(prog.state(i)(7) >= 4.);  // velocity ego
    }

    // Unsafe criterion: Find a collision with the traffic car in left lane.
    // y ego >= 0. && x ego <= x traffic1 + 2.5 && x ego >= x traffic1 - 2.5
    prog.AddLinearConstraint(prog.final_state()(5) >= -0.01 + delta_y);
    prog.AddLinearConstraint(prog.final_state()(4) >=
                             prog.final_state()(0) - 2.5);
    prog.AddLinearConstraint(prog.final_state()(4) <=
                             prog.final_state()(0) + 2.5);
  }

  systems::BasicVector<double> x0(vect0);
  systems::BasicVector<double> xf(vectf);
  auto guess_state_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      {0, guess_duration}, {x0.get_value(), xf.get_value()});

  prog.SetInitialTrajectory(PiecewisePolynomial<double>(),
                            guess_state_trajectory);

  const auto result = prog.Solve();

  // Extract the initial context from prog and plot the solution using MATLAB.
  // To view, type `bazel run //common/proto:call_python_client_cli`.
  Eigen::MatrixXd inputs = prog.GetInputSamples();
  Eigen::MatrixXd states = prog.GetStateSamples();
  Eigen::VectorXd times_out = prog.GetSampleTimes();
  const Eigen::VectorXd s0 = states.row(0);
  const Eigen::VectorXd s1 = states.row(1);
  const Eigen::VectorXd s10 = states.row(10);
  const Eigen::VectorXd s11 = states.row(11);
  std::cout << " states.row(0) " << states.row(0) << std::endl;
  std::cout << " states.row(1) " << states.row(1) << std::endl;
  if (kNumLanes == 3) {
    CallPython("figure", 1);
    CallPython("clf");
    CallPython("plot", s10, s11);
    CallPython("setvars", "s10", s10, "s11", s11);
    CallPython("plt.xlabel", "x ego (m)");
    CallPython("plt.ylabel", "y ego (m)");
    CallPython("figure", 2);
    CallPython("clf");
    CallPython("plot", s0, s1);
    CallPython("setvars", "s0", s0, "s1", s1);
    CallPython("plt.xlabel", "x traffic2 (m)");
    CallPython("plt.ylabel", "y traffic2 (m)");
  } else if (kNumLanes == 2) {
    CallPython("figure", 1);
    CallPython("clf");
    CallPython("plot", states.row(4), states.row(5));
    CallPython("plt.xlabel", "x ego (m)");
    CallPython("plt.ylabel", "y ego (m)");
    CallPython("figure", 2);
    CallPython("clf");
    CallPython("plot", states.row(0), states.row(2));
    CallPython("plt.xlabel", "x traffic1 (m)");
    CallPython("plt.ylabel", "x traffic0 (m)");
  } else if (kNumLanes == 1) {
    CallPython("figure", 1);
    CallPython("clf");
    CallPython("plot", states.row(0), states.row(0) - states.row(2));
    CallPython("plt.xlabel", "s lead (m)");
    CallPython("plt.ylabel", "s lead - s ego (m)");
  }

  // Dump the entire solution result to the screen.
  // for (int i{0}; i < states.size(); ++i) {
  //   std::cout << " states(" << i << ") " << states(i) << std::endl;
  // }

  std::cout << " SOLUTION RESULT: " << result << std::endl;

  //if (result == solvers::SolutionResult::kSolutionFound) {
  if (true) {
    // Build another simulator with LCM capability and run in play-back mode.
    auto simulator_lcm = SetupSimulator(true /* is_playback_mode */,
                                        kNumLanes);
    simulator_lcm->Build();

    // Pipe the offending initial condition into AutomotiveSimulator.
    // TODO(jadecastro): Set up a trajectory-playback Diagram.
    const auto& plant_lcm = simulator_lcm->GetDiagram();
    auto context_lcm = plant_lcm.CreateDefaultContext();
    context_lcm->get_mutable_continuous_state().SetFromVector(states.col(0));

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