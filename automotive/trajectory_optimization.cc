#include "drake/automotive/trajectory_optimization.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/common/proto/call_python.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace automotive {

using common::CallPython;
using maliput::api::Lane;
using maliput::api::RoadGeometry;
using systems::Diagram;
using systems::System;
using systems::trajectory_optimization::DirectCollocation;
using trajectories::PiecewisePolynomial;

static constexpr int kX = SimpleCarStateIndices::kX;
static constexpr int kY = SimpleCarStateIndices::kY;
static constexpr int kHeading = SimpleCarStateIndices::kHeading;
// static constexpr int kVelocity = SimpleCarStateIndices::kVelocity;

TrajectoryOptimization::TrajectoryOptimization(
    std::unique_ptr<Scenario> scenario, int num_time_samples,
    double min_time_step, double max_time_step,
    double initial_guess_duration_sec)
    : num_time_samples_(num_time_samples),
      min_time_step_(min_time_step),
      max_time_step_(max_time_step),
      initial_guess_duration_sec_(initial_guess_duration_sec),
      scenario_(std::move(scenario)) {
  // ** TODO ** Check that scenario_ is valid.
  const auto& plant = dynamic_cast<const systems::System<double>&>(
      scenario_->diagram());  // Need?

  // Set up a direct-collocation problem.
  const double kBoundingBoxLimit = 300.;
  auto context = plant.CreateDefaultContext();
  prog_ = std::make_unique<DirectCollocation>(
      &plant, *context, num_time_samples_, min_time_step_, max_time_step_);

  // Ensure that time intervals are evenly spaced.
  prog_->AddEqualTimeIntervalsConstraints();

  // Generous bounding box on all decision variables.
  prog_->AddBoundingBoxConstraint(-kBoundingBoxLimit, kBoundingBoxLimit,
                                  prog_->decision_variables());

  FixInitialConditions();
}

void TrajectoryOptimization::FixInitialConditions() {
  // Parse the initial conditions and set a constraint there.
  const systems::VectorBase<double>& initial_states_lb =
      scenario_->initial_context_lb().get_continuous_state_vector();
  prog_->AddLinearConstraint(prog_->initial_state() >=
                             initial_states_lb.CopyToVector());
  const systems::VectorBase<double>& initial_states_ub =
      scenario_->initial_context_ub().get_continuous_state_vector();
  prog_->AddLinearConstraint(prog_->initial_state() <=
                             initial_states_ub.CopyToVector());
}

void TrajectoryOptimization::SetLinearGuessTrajectory() {
  const Eigen::VectorXd x0_lb =
      scenario().initial_context_lb().get_continuous_state_vector().CopyToVector();
  const Eigen::VectorXd x0_ub =
      scenario().initial_context_ub().get_continuous_state_vector().CopyToVector();
  const Eigen::VectorXd x0 = 0.5 * (x0_lb + x0_ub);
  const Eigen::VectorXd xf =
      scenario().final_context().get_continuous_state_vector().CopyToVector();
  auto guess_state_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      {0, initial_guess_duration_sec_}, {x0, xf});
  prog_->SetInitialTrajectory(PiecewisePolynomial<double>(),
                              guess_state_trajectory);
}

// N.B. Assumes Dragway.
void TrajectoryOptimization::SetDragwayLaneBounds(
    const System<double>& subsystem,
    std::pair<const Lane*, const Lane*> lane_bounds) {
  using std::cos;
  using std::sin;

  std::vector<double> y_bounds{};
  if (dynamic_cast<const maliput::dragway::RoadGeometry*>(&scenario().road()) ==
      nullptr) {
    throw std::runtime_error("This function only works for Dragway.");
  }
  y_bounds.push_back(
      lane_bounds.first
          ->ToGeoPosition({0., lane_bounds.first->lane_bounds(0.).min(), 0.})
          .y());
  y_bounds.push_back(
      lane_bounds.first
          ->ToGeoPosition({0., lane_bounds.first->lane_bounds(0.).max(), 0.})
      .y());
  y_bounds.push_back(
      lane_bounds.second
          ->ToGeoPosition({0., lane_bounds.first->lane_bounds(0.).min(), 0.})
          .y());
  y_bounds.push_back(
      lane_bounds.second
          ->ToGeoPosition({0., lane_bounds.first->lane_bounds(0.).max(), 0.})
          .y());

  auto state = get_state(&subsystem);
  const double w = scenario().car_width();
  const double l = scenario().car_length();
  const auto y00 =
      state[kY] + (l / 2.) * sin(state[kHeading]) - (w / 2.) * cos(state[kHeading]);
  const auto y10 =
      state[kY] - (l / 2.) * sin(state[kHeading]) - (w / 2.) * cos(state[kHeading]);
  const auto y01 =
      state[kY] - (l / 2.) * sin(state[kHeading]) + (w / 2.) * cos(state[kHeading]);
  const auto y11 =
      state[kY] + (l / 2.) * sin(state[kHeading]) + (w / 2.) * cos(state[kHeading]);

  prog_->AddConstraintToAllKnotPoints(
      y00 >= *std::min_element(y_bounds.begin(), y_bounds.end()));
  prog_->AddConstraintToAllKnotPoints(
      y00 <= *std::max_element(y_bounds.begin(), y_bounds.end()));
  prog_->AddConstraintToAllKnotPoints(
      y01 >= *std::min_element(y_bounds.begin(), y_bounds.end()));
  prog_->AddConstraintToAllKnotPoints(
      y01 <= *std::max_element(y_bounds.begin(), y_bounds.end()));
  prog_->AddConstraintToAllKnotPoints(
      y10 >= *std::min_element(y_bounds.begin(), y_bounds.end()));
  prog_->AddConstraintToAllKnotPoints(
      y10 <= *std::max_element(y_bounds.begin(), y_bounds.end()));
  prog_->AddConstraintToAllKnotPoints(
      y11 >= *std::min_element(y_bounds.begin(), y_bounds.end()));
  prog_->AddConstraintToAllKnotPoints(
      y11 <= *std::max_element(y_bounds.begin(), y_bounds.end()));
}

void TrajectoryOptimization::AddFinalCollisionConstraints(
    const System<double>& subsystem) {
  using std::cos;
  using std::sin;

  auto ego_state = get_final_state(scenario_->ego_alias());
  const auto x_ego = ego_state[kX];
  const auto y_ego = ego_state[kY];
  const auto heading_ego = ego_state[kHeading];
  auto ado_state = get_final_state(&subsystem);
  const auto x_ado = ado_state[kX];
  const auto y_ado = ado_state[kY];
  const auto heading_ado = ado_state[kHeading];

  // Compute a bisecting point, use it to approximately declare collision
  // between cars.
  const auto x_bisect = 0.5 * (x_ego + x_ado);
  const auto y_bisect = 0.5 * (y_ego + y_ado);

  const double w = scenario().car_width();
  const double l = scenario().car_length();

  const auto a_lat_x_ego = -sin(heading_ego);
  const auto a_lat_y_ego = cos(heading_ego);
  const auto b_lat_lo_ego =
      -x_ego * sin(heading_ego) +
      y_ego * cos(heading_ego) - w / 2.;
  const auto b_lat_hi_ego =
      -x_ego * sin(heading_ego) +
      y_ego * cos(heading_ego) + w / 2.;
  const auto a_lon_x_ego = cos(heading_ego);
  const auto a_lon_y_ego = sin(heading_ego);
  const auto b_lon_lo_ego =
      x_ego * cos(heading_ego) +
      y_ego * sin(heading_ego) - l / 2.;
  const auto b_lon_hi_ego =
      x_ego * cos(heading_ego) +
      y_ego * sin(heading_ego) + l / 2.;

  const auto a_lat_x_ado = -sin(heading_ado);
  const auto a_lat_y_ado = cos(heading_ado);
  const auto b_lat_lo_ado =
      -x_ado * sin(heading_ado) +
      y_ado * cos(heading_ado) - w / 2.;
  const auto b_lat_hi_ado =
      -x_ado * sin(heading_ado) +
      y_ado * cos(heading_ado) + w / 2.;
  const auto a_lon_x_ado = cos(heading_ado);
  const auto a_lon_y_ado = sin(heading_ado);
  const auto b_lon_lo_ado =
      x_ado * cos(heading_ado) +
      y_ado * sin(heading_ado) - l / 2.;
  const auto b_lon_hi_ado =
      x_ado * cos(heading_ado) +
      y_ado * sin(heading_ado) + l / 2.;

  // Require the bisector to be in collision with the ego car.
  prog_->AddConstraint(
      a_lat_x_ego * x_bisect + a_lat_y_ego * y_bisect <= b_lat_hi_ego);
  prog_->AddConstraint(
      a_lat_x_ego * x_bisect + a_lat_y_ego * y_bisect >= b_lat_lo_ego);
  prog_->AddConstraint(
      a_lon_x_ego * x_bisect + a_lon_y_ego * y_bisect <= b_lon_hi_ego);
  prog_->AddConstraint(
      a_lon_x_ego * x_bisect + a_lon_y_ego * y_bisect >= b_lon_hi_ego);

  // Require the bisector to be in collision with the ado car.
  prog_->AddConstraint(
      a_lat_x_ado * x_bisect + a_lat_y_ado * y_bisect <= b_lat_hi_ado);
  prog_->AddConstraint(
      a_lat_x_ado * x_bisect + a_lat_y_ado * y_bisect >= b_lat_lo_ado);
  prog_->AddConstraint(
      a_lon_x_ado * x_bisect + a_lon_y_ado * y_bisect <= b_lon_hi_ado);
  prog_->AddConstraint(
      a_lon_x_ado * x_bisect + a_lon_y_ado * y_bisect >= b_lon_hi_ado);
}

void TrajectoryOptimization::AddGaussianCost() {
  const double kSigma = 50.;
  const double kCoeff = 1. / (2. * kSigma * kSigma);
  symbolic::Expression running_cost;
  for (int i{0}; i < prog_->input().size(); ++i) {
    std::cout << " prog->input()(i) " << prog_->input()(i) << std::endl;
    running_cost += kCoeff * prog_->input()(i) * prog_->input()(i);
  }
  prog_->AddRunningCost(running_cost);
}

void TrajectoryOptimization::SetEgoLinearConstraint(
    const Eigen::Ref<const Eigen::MatrixXd> A,
    const Eigen::Ref<const Eigen::VectorXd> b, double t) {
  DRAKE_DEMAND(min_time_step_ == max_time_step_);
  DRAKE_DEMAND(t >= 0 && t <= min_time_step_ * num_time_samples_);
  DRAKE_DEMAND(A.cols() == SimpleCarStateIndices::kNumCoordinates);

  // Assume a zero-order hold on the constraints application.
  const int index = std::ceil(t / min_time_step_);
  auto x_ego = get_state(index, scenario_->ego_alias());
  prog_->AddLinearConstraint(A, -b * std::numeric_limits<double>::infinity(), b,
                             x_ego);
  // prog_->AddLinearConstraint(A * x_ego <= b);  // ** TODO ** Enable this spelling.
}

void TrajectoryOptimization::Solve() {
  result_ = prog_->Solve();
  trajectory_.inputs = prog_->GetInputSamples();
  trajectory_.states = prog_->GetStateSamples();
  trajectory_.times = prog_->GetSampleTimes();

  std::cout << " Sample times: " << trajectory_.times << std::endl;
  std::cout << " Inputs for ego car: " << trajectory_.inputs.row(0)
            << std::endl;
  std::cout << "                     " << trajectory_.inputs.row(1)
            << std::endl;
  // TODO(jadecastro) Transform this cost into a probability distribution.
  std::cout << " Optimal Cost: " << prog_->GetOptimalCost() << std::endl;

  // Dump the entire solution result to the screen.
  // for (int i{0}; i < states.size(); ++i) {
  //   std::cout << " states(" << i << ") " << states(i) << std::endl;
  // }

  std::cout << " SOLUTION RESULT: " << result_ << std::endl;
}

solvers::VectorXDecisionVariable
TrajectoryOptimization::get_state(
    const System<double>* subsystem) const {
  DRAKE_DEMAND(subsystem != nullptr);
  const std::vector<int> indices = scenario_->GetStateIndices(*subsystem);
  return prog_->state().segment(
      indices[0], SimpleCarStateIndices::kNumCoordinates);
}

TrajectoryOptimization::SubVectorXDecisionVariable
TrajectoryOptimization::get_state(
    int index, const System<double>* subsystem) const {
  DRAKE_DEMAND(subsystem != nullptr);
  const std::vector<int> indices = scenario_->GetStateIndices(*subsystem);
  return prog_->state(index).segment(
      indices[0], SimpleCarStateIndices::kNumCoordinates);
}

TrajectoryOptimization::SubVectorXDecisionVariable
TrajectoryOptimization::get_initial_state(
    const System<double>* subsystem) const {
  DRAKE_DEMAND(subsystem != nullptr);
  const std::vector<int> indices = scenario_->GetStateIndices(*subsystem);
  return prog_->initial_state().segment(
      indices[0], SimpleCarStateIndices::kNumCoordinates);
}

TrajectoryOptimization::SubVectorXDecisionVariable
TrajectoryOptimization::get_final_state(
    const System<double>* subsystem) const {
  DRAKE_DEMAND(subsystem != nullptr);
  const std::vector<int> indices = scenario_->GetStateIndices(*subsystem);
  return prog_->final_state().segment(
      indices[0], SimpleCarStateIndices::kNumCoordinates);
}

double TrajectoryOptimization::GetSolutionTotalProbability() const {
  using std::log;
  using std::sqrt;

  // const double kSigma = 50.;
  // const double kP = 2. * num_time_samples_ * kSigma * kSigma *
  //                      log(1. / (sqrt(2. * M_PI) * kSigma)) -
  //                  2. * kSigma * kSigma * log(path_probability);
  //drake::unused(kP);
  //for (int i{0}; i < prog_->input().size(); ++i) {
  //  std::cout << " prog->input()(i) " << prog_->input()(i) << std::endl;
  //}
  return 0.;
}

void TrajectoryOptimization::PlotSolution() {
  int i{0};
  for (const auto& subsystem : scenario().aliases()) {
    const Eigen::VectorXd x =
        trajectory_.states.row(scenario().GetStateIndices(*subsystem).at(kX));
    const Eigen::VectorXd y =
        trajectory_.states.row(scenario().GetStateIndices(*subsystem).at(kY));
    CallPython("figure", i++ + 2);
    CallPython("clf");
    CallPython("plot", x, y);
    //    CallPython("setvars", "ado_x_" + std::to_string(i), ado_x,
    //           "ado_y_" + std::to_string(i), ado_y);
    CallPython("plt.xlabel", subsystem->get_name() + " x (m)");
    CallPython("plt.ylabel", subsystem->get_name() + " y (m)");
    CallPython("plt.show");
  }
}

void TrajectoryOptimization::AnimateSolution() const {
  const Eigen::MatrixXd inputs = prog_->GetInputSamples();
  const Eigen::MatrixXd states = prog_->GetStateSamples();
  const Eigen::VectorXd times_out = prog_->GetSampleTimes();

  // ** TODO ** Make an automotive::Trajectory() from the result, and replay it
  //            in AutomotiveSimulator.
  /*
  const double kRealTimeRate = 1.;
  if (true) {
    // Build another simulator with LCM capability and run in play-back mode.
    auto simulator = std::unique_ptr<AutomotiveSimulator>();
    for (int i{0}; i < states.cols(); i++) {
        simulator->AddTrajectoryFollower(states.col(i));
    }
    simulator->Build();
    simulator->Start(kRealTimeRate);
    simulator->StepBy(times.back());
  }
  */
}

}  // namespace automotive
}  // namespace drake
