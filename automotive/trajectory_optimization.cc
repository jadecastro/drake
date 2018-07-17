#include "drake/automotive/trajectory_optimization.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/common/proto/call_python.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace automotive {

using common::CallPython;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using systems::BasicVector;
using systems::Context;
using systems::Diagram;
using systems::System;
using systems::trajectory_optimization::DirectCollocation;
using trajectories::PiecewisePolynomial;

namespace {

// Calculate the Cartesian components of `substate` by evaluating the
// SimpleCarState output of `subsystem`.
template <typename T>
TrajectoryOptimization::Cartesian<T> CalcCartesian(const System<T>& subsystem,
                                                   const VectorX<T>& substate) {
  auto subcontext = subsystem.CreateDefaultContext();
  subcontext->get_mutable_continuous_state_vector().SetFromVector(substate);
  auto output = subsystem.AllocateOutput();
  auto car_state = dynamic_cast<const SimpleCarState<T>*>(
      output->get_vector_data(Scenario::kSimpleCarStatePort));
  DRAKE_DEMAND(car_state != nullptr);
  subsystem.CalcOutput(*subcontext, output.get());
  TrajectoryOptimization::Cartesian<T> cartesian;
  cartesian.x = car_state->x();
  cartesian.y = car_state->y();
  cartesian.heading = car_state->heading();
  return cartesian;
}

}  // namespace

TrajectoryOptimization::TrajectoryOptimization(
    std::unique_ptr<Scenario> scenario, int num_time_samples,
    double min_time_step, double max_time_step,
    double initial_guess_duration_sec, double bounding_box_limit = -1.)
    : num_time_samples_(num_time_samples),
      min_time_step_(min_time_step),
      max_time_step_(max_time_step),
      initial_guess_duration_sec_(initial_guess_duration_sec),
      scenario_(std::move(scenario)),
      initial_context_ub_(scenario_->context().Clone()),
      initial_context_lb_(scenario_->context().Clone()),
      final_context_ub_(scenario_->context().Clone()),
      final_context_lb_(scenario_->context().Clone()) {
  // ** TODO ** Check that scenario_ is valid.

  // Set up a direct-collocation problem.
  prog_ = std::make_unique<DirectCollocation>(
      &scenario_->diagram(), *initial_context_lb_, num_time_samples_,
      min_time_step_, max_time_step_);

  // Ensure that time intervals are evenly spaced.
  prog_->AddEqualTimeIntervalsConstraints();

  if (bounding_box_limit > 0.) {
    // Apply a uniform bounding box on all decision variables.
    prog_->AddBoundingBoxConstraint(-bounding_box_limit, bounding_box_limit,
                                    prog_->decision_variables());
  }
}

void TrajectoryOptimization::RegisterInitialBoxConstraint(
    const System<double>& subsystem,
    const BasicVector<double>& initial_state_lb,
    const BasicVector<double>& initial_state_ub) {
  DRAKE_DEMAND(!is_solved_);
  // ** TODO ** Check that the bounds are consistent.
  SetSubcontext(subsystem, initial_state_ub, initial_context_ub_.get());
  SetSubcontext(subsystem, initial_state_lb, initial_context_lb_.get());
}

void TrajectoryOptimization::RegisterInitialConstraint(
    const System<double>& subsystem, const BasicVector<double>& initial_state) {
  RegisterInitialBoxConstraint(subsystem, initial_state, initial_state);
}

void TrajectoryOptimization::RegisterFinalBoxConstraint(
    const System<double>& subsystem, const BasicVector<double>& final_state_lb,
    const BasicVector<double>& final_state_ub) {
  DRAKE_DEMAND(!is_solved_);
  // ** TODO ** Check that the bounds are consistent.
  SetSubcontext(subsystem, final_state_ub, final_context_ub_.get());
  SetSubcontext(subsystem, final_state_lb, final_context_lb_.get());
}

void TrajectoryOptimization::RegisterFinalConstraint(
    const System<double>& subsystem, const BasicVector<double>& final_state) {
  RegisterFinalBoxConstraint(subsystem, final_state, final_state);
}

void TrajectoryOptimization::AddInitialConstraints() {
  DRAKE_DEMAND(!is_solved_);
  const systems::VectorBase<double>& initial_states_lb =
      initial_context_lb_->get_continuous_state_vector();
  prog_->AddLinearConstraint(prog_->initial_state() >=
                             initial_states_lb.CopyToVector());
  const systems::VectorBase<double>& initial_states_ub =
      initial_context_ub_->get_continuous_state_vector();
  prog_->AddLinearConstraint(prog_->initial_state() <=
                             initial_states_ub.CopyToVector());
}

void TrajectoryOptimization::AddFinalConstraints() {
  DRAKE_DEMAND(!is_solved_);
  const systems::VectorBase<double>& initial_states_lb =
      initial_context_lb_->get_continuous_state_vector();
  prog_->AddLinearConstraint(prog_->initial_state() >=
                             initial_states_lb.CopyToVector());
  const systems::VectorBase<double>& initial_states_ub =
      initial_context_ub_->get_continuous_state_vector();
  prog_->AddLinearConstraint(prog_->initial_state() <=
                             initial_states_ub.CopyToVector());
}

void TrajectoryOptimization::SetLinearGuessTrajectory() {
  DRAKE_DEMAND(!is_solved_);
  const Eigen::VectorXd x0_lb =
      initial_context_lb_->get_continuous_state_vector().CopyToVector();
  const Eigen::VectorXd x0_ub =
      initial_context_ub_->get_continuous_state_vector().CopyToVector();
  const Eigen::VectorXd x0 = 0.5 * (x0_lb + x0_ub);
  const Eigen::VectorXd xf_lb =
      final_context_lb_->get_continuous_state_vector().CopyToVector();
  const Eigen::VectorXd xf_ub =
      final_context_ub_->get_continuous_state_vector().CopyToVector();
  const Eigen::VectorXd xf = 0.5 * (xf_lb + xf_ub);
  auto guess_state_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      {0, initial_guess_duration_sec_}, {x0, xf});
  prog_->SetInitialTrajectory(PiecewisePolynomial<double>(),
                              guess_state_trajectory);
}

// N.B. Assumes Dragway.
void TrajectoryOptimization::AddDragwayLaneConstraints(
    const System<double>& subsystem,
    std::pair<const Lane*, const Lane*> lane_bounds) {
  if (dynamic_cast<const maliput::dragway::RoadGeometry*>(&scenario_->road()) ==
      nullptr) {
    throw std::runtime_error("This function only works for Dragway.");
  }
  DRAKE_DEMAND(!is_solved_);

  using std::cos;
  using std::sin;

  // N.B. We have a dragway; we can safely assume each lane has the same width.
  const LanePosition min_lane_pos(0., lane_bounds.first->lane_bounds(0.).min(),
                                  0.);
  const LanePosition max_lane_pos(0., lane_bounds.first->lane_bounds(0.).max(),
                                  0.);

  std::vector<double> y_bounds{};
  y_bounds.push_back(lane_bounds.first->ToGeoPosition(min_lane_pos).y());
  y_bounds.push_back(lane_bounds.first->ToGeoPosition(max_lane_pos).y());
  y_bounds.push_back(lane_bounds.second->ToGeoPosition(min_lane_pos).y());
  y_bounds.push_back(lane_bounds.second->ToGeoPosition(max_lane_pos).y());
  const double y_min = *std::min_element(y_bounds.begin(), y_bounds.end());
  const double y_max = *std::max_element(y_bounds.begin(), y_bounds.end());

  const double w = scenario_->car_width();
  const double l = scenario_->car_length();

  auto state = get_state(subsystem);
  auto ego = get_cartesian(subsystem, state);

  // Compute the y-component of the four vertices of the body.
  const auto y00 =
      ego.y + (l / 2.) * sin(ego.heading) - (w / 2.) * cos(ego.heading);
  const auto y10 =
      ego.y - (l / 2.) * sin(ego.heading) - (w / 2.) * cos(ego.heading);
  const auto y01 =
      ego.y - (l / 2.) * sin(ego.heading) + (w / 2.) * cos(ego.heading);
  const auto y11 =
      ego.y + (l / 2.) * sin(ego.heading) + (w / 2.) * cos(ego.heading);

  prog_->AddConstraintToAllKnotPoints(y_min <= y00);
  prog_->AddConstraintToAllKnotPoints(y00 <= y_max);
  prog_->AddConstraintToAllKnotPoints(y_min <= y01);
  prog_->AddConstraintToAllKnotPoints(y01 <= y_max);
  prog_->AddConstraintToAllKnotPoints(y_min <= y10);
  prog_->AddConstraintToAllKnotPoints(y10 <= y_max);
  prog_->AddConstraintToAllKnotPoints(y_min <= y11);
  prog_->AddConstraintToAllKnotPoints(y11 <= y_max);
}

void TrajectoryOptimization::AddFinalCollisionConstraintsOld(
    const System<double>& subsystem1, const System<double>& subsystem2) {
  DRAKE_DEMAND(!is_solved_);
  using std::cos;
  using std::sin;

  // Compute a bounding box in the body frame (x'-y') and check for
  // intersections with a trial point.
  const double w = scenario_->car_width();
  const double l = scenario_->car_length();
  const Eigen::Vector2d xpyp_offset(w / 2., l / 2.);

  auto ego_final_state = get_final_state(subsystem1);
  auto ego = get_cartesian(subsystem1, ego_final_state);
  const Vector2<symbolic::Expression> xy_ego(ego.x, ego.y);

  auto ado_final_state = get_final_state(subsystem2);
  auto ado = get_cartesian(subsystem2, ado_final_state);
  const Vector2<symbolic::Expression> xy_ado(ado.x, ado.y);

  // Compute a bisecting point on the line connecting the two bodies, use it to
  // declare a (possibly conservative) necessary condition for collision between
  // them.
  const Vector2<symbolic::Expression> xy_bisect = 0.5 * (xy_ego + xy_ado);

  Matrix2<symbolic::Expression> A_ego;
  // clang-format off
  A_ego << -sin(ego.heading), cos(ego.heading),
            cos(ego.heading), sin(ego.heading);
  // clang-format on

  const Vector2<symbolic::Expression> b_hi_ego = A_ego * xy_ego + xpyp_offset;
  const Vector2<symbolic::Expression> b_lo_ego = A_ego * xy_ego - xpyp_offset;

  Matrix2<symbolic::Expression> A_ado;
  // clang-format off
  A_ado << -sin(ado.heading), cos(ado.heading),
            cos(ado.heading), sin(ado.heading);
  // clang-format on

  const Vector2<symbolic::Expression> b_hi_ado = A_ado * xy_ado + xpyp_offset;
  const Vector2<symbolic::Expression> b_lo_ado = A_ado * xy_ado - xpyp_offset;

  prog_->AddConstraint(b_lo_ego <= A_ego * xy_bisect);
  prog_->AddConstraint(A_ego * xy_bisect <= b_hi_ego);
  prog_->AddConstraint(b_lo_ado <= A_ado * xy_bisect);
  prog_->AddConstraint(A_ado * xy_bisect <= b_hi_ado);
}

void TrajectoryOptimization::AddFinalCollisionConstraints(
    const System<double>& subsystem1, const System<double>& subsystem2) {
  DRAKE_DEMAND(!is_solved_);
  using std::cos;
  using std::sin;

  // Compute a bounding box in the body frame (x'-y') and check for
  // intersections with a trial point.
  const double w = scenario_->car_width();
  const double l = scenario_->car_length();
  const Eigen::DiagonalMatrix<symbolic::Expression, 2> Q(l / 2., w / 2.);

  auto ego_final_state = get_final_state(subsystem1);
  auto ego = get_cartesian(subsystem1, ego_final_state);
  const Vector2<symbolic::Expression> z_ego(ego.x, ego.y);

  auto ado_final_state = get_final_state(subsystem2);
  auto ado = get_cartesian(subsystem2, ado_final_state);
  const Vector2<symbolic::Expression> z_ado(ado.x, ado.y);

  Matrix2<symbolic::Expression> R_ego;
  // clang-format off
  R_ego << -sin(ego.heading), cos(ego.heading),
            cos(ego.heading), sin(ego.heading);
  // clang-format on

  Matrix2<symbolic::Expression> R_ado;
  // clang-format off
  R_ado << -sin(ado.heading), cos(ado.heading),
            cos(ado.heading), sin(ado.heading);
  // clang-format on

  const Matrix2<symbolic::Expression> C_ego = R_ego * Q;
  const Matrix2<symbolic::Expression> C_ado = R_ado * Q;
  const Vector2<symbolic::Expression> z_tilde =
      C_ego.inverse() * (z_ado - z_ego);

  prog_->AddConstraint(z_tilde.transpose() * (C_ado + R_ado).inverse() *
                       (C_ado + R_ado) * z_tilde <= 1.);
}

void TrajectoryOptimization::AddGaussianCost(const System<double>& subsystem,
                                             const Eigen::MatrixXd& sigma) {
  DRAKE_DEMAND(!is_solved_);
  sigma_map_[&subsystem] = sigma;

  const auto input = get_input(subsystem);
  DRAKE_DEMAND(sigma.rows() == input.size());
  DRAKE_DEMAND(sigma.cols() == input.size());

  const Eigen::MatrixXd sigma_inv = sigma.inverse();
  prog_->AddRunningCost(input.transpose() * sigma_inv * input);
}

void TrajectoryOptimization::AddGaussianTotalProbabilityConstraint(
    const System<double>& subsystem) {
  throw std::runtime_error("Not implemented.");
}

void TrajectoryOptimization::AddLinearConstraint(
    const System<double>& subsystem, const Eigen::Ref<const Eigen::MatrixXd> A,
    const Eigen::Ref<const Eigen::VectorXd> b, double t) {
  DRAKE_DEMAND(!is_solved_);
  DRAKE_DEMAND(min_time_step_ == max_time_step_);
  DRAKE_DEMAND(A.cols() == SimpleCarStateIndices::kNumCoordinates);

  // Assume a zero-order hold on the constraints application.
  const VectorX<symbolic::Expression> state = get_state(t, subsystem);
  prog_->AddLinearConstraint(A * state <= b);
}

solvers::SolutionResult TrajectoryOptimization::Solve() {
  DRAKE_DEMAND(!is_solved_);

  result_ = prog_->Solve();

  trajectory_.inputs = prog_->GetInputSamples();
  trajectory_.states = prog_->GetStateSamples();
  trajectory_.times = prog_->GetSampleTimes();

  const int size = trajectory_.times.rows();
  for (const auto& subsystem : scenario_->aliases()) {
    trajectory_.x[subsystem].resize(size);
    trajectory_.y[subsystem].resize(size);
    trajectory_.heading[subsystem].resize(size);
    for (int i{0}; i < size; i++) {
      const std::vector<int> indices = scenario_->GetStateIndices(*subsystem);
      const Eigen::VectorXd substates =
          trajectory_.states.col(i).segment(indices[0], indices.size());
      const auto stateful_subsystem = scenario_->stateful_aliases()[subsystem];
      const TrajectoryOptimization::Cartesian<double> cartesian =
          CalcCartesian<double>(*stateful_subsystem, substates);
      trajectory_.x[subsystem](i) = cartesian.x;
      trajectory_.y[subsystem](i) = cartesian.y;
      trajectory_.heading[subsystem](i) = cartesian.heading;
    }
  }
  std::cout << " SOLUTION RESULT: " << result_ << std::endl;
  std::cout << " Sample times: " << trajectory_.times.transpose() << std::endl;
  is_solved_ = true;
  return result_;
}

solvers::VectorXDecisionVariable TrajectoryOptimization::get_input(
    const System<double>& subsystem) const {
  const std::vector<int> indices = scenario_->GetInputIndices(subsystem);
  return prog_->input().segment(indices[0], indices.size());
}

TrajectoryOptimization::SubVectorXDecisionVariable
TrajectoryOptimization::get_input(double t,
                                  const System<double>& subsystem) const {
  DRAKE_DEMAND(t >= 0 && t <= min_time_step_ * num_time_samples_);
  const int index = std::ceil(t / min_time_step_);
  const std::vector<int> indices = scenario_->GetInputIndices(subsystem);
  return prog_->input(index).segment(indices[0], indices.size());
}

solvers::VectorXDecisionVariable TrajectoryOptimization::get_state(
    const System<double>& subsystem) const {
  const std::vector<int> indices = scenario_->GetStateIndices(subsystem);
  return prog_->state().segment(indices[0], indices.size());
}

TrajectoryOptimization::SubVectorXDecisionVariable
TrajectoryOptimization::get_state(double t,
                                  const System<double>& subsystem) const {
  DRAKE_DEMAND(t >= 0 && t <= min_time_step_ * num_time_samples_);
  const int index = std::ceil(t / min_time_step_);
  const std::vector<int> indices = scenario_->GetStateIndices(subsystem);
  return prog_->state(index).segment(indices[0], indices.size());
}

TrajectoryOptimization::SubVectorXDecisionVariable
TrajectoryOptimization::get_initial_state(
    const System<double>& subsystem) const {
  const std::vector<int> indices = scenario_->GetStateIndices(subsystem);
  return prog_->initial_state().segment(indices[0], indices.size());
}

TrajectoryOptimization::SubVectorXDecisionVariable
TrajectoryOptimization::get_final_state(const System<double>& subsystem) const {
  const std::vector<int> indices = scenario_->GetStateIndices(subsystem);
  return prog_->final_state().segment(indices[0], indices.size());
}

TrajectoryOptimization::Cartesian<symbolic::Expression>
TrajectoryOptimization::get_cartesian(
    const System<double>& subsystem,
    const VectorX<symbolic::Expression>& substate) const {
  const auto subsystem_symb =
      scenario_->stateful_aliases()[&subsystem]->ToSymbolic();
  return CalcCartesian<symbolic::Expression>(*subsystem_symb, substate);
}

double TrajectoryOptimization::GetSolutionTotalLogPdf() const {
  DRAKE_DEMAND(is_solved_);
  using std::log;
  using std::pow;

  const int size = trajectory_.times.rows();
  const int all_input_size =
      sigma_map_.size() * DrivingCommandIndices::kNumCoordinates;
  // TODO: Replace above with std::accumulate on the sigma sizes.

  double result = -0.5 * size * all_input_size * log(2. * M_PI);
  for (const auto& subsystem : scenario_->aliases()) {
    if (sigma_map_.find(subsystem) == sigma_map_.end()) continue;
    const std::vector<int> indices = scenario_->GetInputIndices(*subsystem);
    const Eigen::MatrixXd sigma = sigma_map_.at(subsystem);
    const double sigma_det = sigma.determinant();
    result -= 0.5 * size * log(sigma_det);
    const Eigen::MatrixXd sigma_inv = sigma.inverse();
    for (int i{0}; i < size; i++) {
      const Eigen::VectorXd subinputs =
          trajectory_.inputs.col(i).segment(indices[0], indices.size());
      result -= 0.5 * subinputs.transpose() * sigma_inv * subinputs;
    }
  }
  return result;
}

double TrajectoryOptimization::GetSolutionTotalLogNormalizedPdf() const {
  DRAKE_DEMAND(is_solved_);
  using std::log;
  using std::pow;

  const int size = trajectory_.times.rows();
  const int all_input_size =
      sigma_map_.size() * DrivingCommandIndices::kNumCoordinates;
  // TODO: Replace above with std::accumulate on the sigma sizes.

  double result = -0.5 * size * all_input_size * log(2. * M_PI);
  for (const auto& subsystem : scenario_->aliases()) {
    if (sigma_map_.find(subsystem) == sigma_map_.end()) continue;
    const std::vector<int> indices = scenario_->GetInputIndices(*subsystem);
    const Eigen::MatrixXd sigma = sigma_map_.at(subsystem);
    const Eigen::MatrixXd sigma_inv = sigma.inverse();
    for (int i{0}; i < size; i++) {
      const Eigen::VectorXd subinputs =
          trajectory_.inputs.col(i).segment(indices[0], indices.size());
      result -= 0.5 * subinputs.transpose() * sigma_inv * subinputs;
    }
  }
  return result;
}

void TrajectoryOptimization::PlotSolution() {
  DRAKE_DEMAND(is_solved_);
  int i{0};
  for (const auto& subsystem : scenario_->aliases()) {
    const Eigen::VectorXd& x = trajectory_.x[subsystem];
    const Eigen::VectorXd& y = trajectory_.y[subsystem];
    CallPython("figure", i + 2);
    CallPython("clf");
    CallPython("plot", x, y);
    //    CallPython("setvars", "ado_x_" + std::to_string(i), ado_x,
    //           "ado_y_" + std::to_string(i), ado_y);
    CallPython("plt.xlabel", subsystem->get_name() + " x (m)");
    CallPython("plt.ylabel", subsystem->get_name() + " y (m)");
    CallPython("plt.show");
    i++;
  }
}

void TrajectoryOptimization::AnimateSolution() const {
  using Type = Trajectory::InterpolationType;

  DRAKE_DEMAND(is_solved_);
  const double kRealTimeRate = 1.;

  // Build another simulator with LCM capability and run in play-back mode.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  simulator->SetRoadGeometry(scenario_->get_road());
  auto* lcm = dynamic_cast<drake::lcm::DrakeLcm*>(simulator->get_lcm());
  DRAKE_DEMAND(lcm != nullptr);

  std::vector<double> times(trajectory_.times.rows());
  for (size_t i{0}; i < times.size(); i++) {
    times[i] = trajectory_.times(i);
  }
  for (const auto& subsystem : scenario_->aliases()) {
    std::vector<Eigen::Vector3d> translations{};
    std::vector<Quaternion<double>> rotations{};
    for (size_t i{0}; i < times.size(); i++) {
      translations.push_back(
          {trajectory_.x.at(subsystem)(i), trajectory_.y.at(subsystem)(i), 0.});
      const math::RollPitchYaw<double> rpy(
          Eigen::Vector3d(0., 0., trajectory_.heading.at(subsystem)(i)));
      rotations.push_back(rpy.ToQuaternion());
      rotations.back().normalize();  // TODO: Need?
    }
    const Trajectory trajectory =
        Trajectory::Make(times, rotations, translations, Type::kPchip);
    simulator->AddPriusTrajectoryFollower(subsystem->get_name(), trajectory);
  }
  lcm->StartReceiveThread();
  simulator->Start(kRealTimeRate);
  simulator->StepBy(times.back());
}

const TrajectoryOptimization::InputStateTrajectoryData&
TrajectoryOptimization::get_trajectory() const {
  DRAKE_DEMAND(is_solved_);
  return trajectory_;
}

void TrajectoryOptimization::SetSubcontext(
    const System<double>& subsystem, const BasicVector<double>& state_vector,
    Context<double>* context) {
  auto& subcontext =
      scenario_->diagram().GetMutableSubsystemContext(subsystem, context);
  systems::VectorBase<double>& state =
      subcontext.get_mutable_continuous_state_vector();
  state.SetFromVector(state_vector.get_value());
}

}  // namespace automotive
}  // namespace drake