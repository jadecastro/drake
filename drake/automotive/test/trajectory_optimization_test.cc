#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/simple_car.h"
#include "drake/common/call_matlab.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace automotive {
namespace {

/*
// Sets up a simple trajectory optimization problem that finds a series
// of DrivingCommand's that takes the SimpleCar from an initial condition
// off the x-axis back to the x-axis.
GTEST_TEST(TrajectoryOptimizationTest, SimpleCarDircolTest) {
  SimpleCar<double> plant;
  auto context = plant.CreateDefaultContext();

  SimpleCarState<double> x0, xf;

  x0.set_x(0.0);
  x0.set_y(1.0);
  x0.set_heading(-.02);
  x0.set_velocity(15.0);  // m/s = ~ 33mph

  const double initial_duration = 30.0;  // seconds
  xf.set_x(x0.x() + initial_duration * x0.velocity());
  xf.set_y(0.0);
  xf.set_heading(0.0);
  xf.set_velocity(x0.velocity());

  const int kNumTimeSamples = 10;

  // The solved trajectory may deviate from the initial guess at a reasonable
  // duration.
  const double kTrajectoryTimeLowerBound = 0.8 * initial_duration,
               kTrajectoryTimeUpperBound = 1.2 * initial_duration;

  systems::DircolTrajectoryOptimization prog(&plant, *context, kNumTimeSamples,
                                             kTrajectoryTimeLowerBound,
                                             kTrajectoryTimeUpperBound);

  // Input limits (note that the steering limit imposed by SimpleCar is larger).
  DrivingCommand<double> lower_limit, upper_limit;
  lower_limit.set_steering_angle(-M_PI_2);
  lower_limit.set_acceleration(-std::numeric_limits<double>::infinity());
  upper_limit.set_steering_angle(M_PI_2);
  upper_limit.set_acceleration(std::numeric_limits<double>::infinity());
  prog.AddInputBounds(lower_limit.get_value(), upper_limit.get_value());

  // Ensure that time intervals are (relatively) evenly spaced.
  prog.AddTimeIntervalBounds(kTrajectoryTimeLowerBound / (kNumTimeSamples - 1),
                             kTrajectoryTimeUpperBound / (kNumTimeSamples - 1));

  // Fix initial conditions.
  prog.AddLinearConstraint(prog.initial_state() == x0.get_value());

  // Fix final conditions.
  prog.AddLinearConstraint(prog.final_state() == xf.get_value());

  // Cost function: int_0^T [ u'u ] dt.
  prog.AddRunningCost(prog.input().transpose() * prog.input());

  // Initial guess is a straight line from the initial state to the final state.
  auto initial_state_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      {0, initial_duration}, {x0.get_value(), xf.get_value()});

  solvers::SolutionResult result =
      prog.SolveTraj(initial_duration, PiecewisePolynomial<double>(),
                     initial_state_trajectory);

  solvers::SolverType solver;
  int solver_result;
  prog.GetSolverResult(&solver, &solver_result);

  if (solver == solvers::SolverType::kIpopt) {
    EXPECT_EQ(result,
              solvers::SolutionResult::kIterationLimit);  // TODO(russt): Tune
                                                          // Ipopt for this
                                                          // example.
  } else {
    EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound);
  }

  // Plot the solution.
  // Note: see lcm_call_matlab.h for instructions on viewing the plot.
  Eigen::MatrixXd inputs;
  Eigen::MatrixXd states;
  std::vector<double> times_out;
  prog.GetResultSamples(&inputs, &states, &times_out);
  common::CallMatlab("plot", states.row(SimpleCarStateIndices::kX),
                     states.row(SimpleCarStateIndices::kY));
  common::CallMatlab("xlabel", "x (m)");
  common::CallMatlab("ylabel", "y (m)");

  // Checks that the input commands found are not too large.
  EXPECT_LE(inputs.row(0).lpNorm<1>(), 0.1);
  EXPECT_LE(inputs.row(1).lpNorm<1>(), 1);
}
*/

/*
// Sets up a simple trajectory optimization problem that finds a series
// of DrivingCommand's that takes the SimpleCar from an initial condition
// off the x-axis back to the x-axis.  This test loads SimpleCar
// via the AutomotiveSimulator class.
GTEST_TEST(TrajectoryOptimizationTest, AutomotiveSimulatorDircolTest) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  simulator->AddPriusSimpleCar("Model1");
  simulator->Build();
  const auto& plant = simulator->GetDiagram();
  auto context = plant.CreateDefaultContext();

  // Diagram state is only SimpleCarState
  SimpleCarState<double> x0;
  x0.set_x(0.0);
  x0.set_y(1.0);
  x0.set_heading(-.02);
  x0.set_velocity(15.0);  // m/s = ~ 33mph

  const double duration = 5.0;  // seconds
  const int kNumTimeSamples = 10;

  systems::DircolTrajectoryOptimization prog(&plant, *context, kNumTimeSamples,
                                             duration, duration);

  // Ensure that time intervals are evenly spaced.
  // TODO(russt): Add sugar to DirectTrajectoryOptimization for this.
  prog.AddTimeIntervalBounds(duration / (kNumTimeSamples - 1),
                             duration / (kNumTimeSamples - 1));

  prog.AddLinearConstraint( prog.initial_state() == x0.get_value() );

  EXPECT_EQ( prog.Solve(), solvers::SolutionResult::kSolutionFound);

}
*/

GTEST_TEST(TrajectoryOptimizationTest, AutomotiveSimulatorIdmTest) {
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry =
      std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"Dircol Test Dragway"}),
          1 , 100. , 2. , 0.);
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  auto simulator_road = simulator->SetRoadGeometry(std::move(road_geometry));
  auto dragway_road_geometry =
      dynamic_cast<const maliput::dragway::RoadGeometry*>(simulator_road);

  const int lane_index = 0;

  // These defaults should be ignored by the solver.
  const double start_speed_follower = 20.;
  const double start_position_follower = 5.;
  const double speed_leader = 10.;
  const double start_position_leader = 20.;

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
  simulator->BuildAndInitialize();

  const auto& plant = simulator->GetDiagram();
  auto context = plant.CreateDefaultContext();

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

  // Plot the solution.
  // Note: see lcm_call_matlab.h for instructions on viewing the plot.
  Eigen::MatrixXd inputs;
  Eigen::MatrixXd states;
  std::vector<double> times_out;
  prog.GetResultSamples(&inputs, &states, &times_out);
  common::CallMatlab("plot", states.row(0), states.row(0) - states.row(2));
  common::CallMatlab("xlabel", "s lead (m)");
  common::CallMatlab("ylabel", "s lead - s ego (m)");

  // TODO(jadecastro): Save the offending initial condition and replay that
  // using AutomotiveSimulator.

}

}  // namespace
}  // namespace automotive
}  // namespace drake
