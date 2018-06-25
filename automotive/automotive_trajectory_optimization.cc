#include "drake/automotive/automotive_trajectory_optimization.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/pure_pursuit_controller.h"
#include "drake/common/proto/call_python.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/rendering/pose_aggregator.h"

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
static constexpr int kVelocity = SimpleCarStateIndices::kVelocity;

static constexpr int kEgoPort =
    0;  // Identifier for both Demux and PoseAggregator.

// Model parameters.
static constexpr double kPeriodSec = 0.1;  // For IDM.
static constexpr ScanStrategy kScanStrategy = ScanStrategy::kPath;
static constexpr RoadPositionStrategy kRoadPositionStrategy =
    RoadPositionStrategy::kExhaustiveSearch;

namespace {

std::unique_ptr<RoadGeometry> MakeDragway(int num_lanes, double lane_width,
                                          double road_length) {
  return std::make_unique<maliput::dragway::RoadGeometry>(
      maliput::api::RoadGeometryId("dragway"), num_lanes, road_length,
      lane_width, 0. /* shoulder width */, 5. /* maximum_height */,
      std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance */);
}

}  // namespace

Scenario::Scenario(int num_lanes, double lane_width, double road_length,
                   double car_width, double car_length)
    : road_(MakeDragway(num_lanes, lane_width, road_length)),
      car_width_(car_width), car_length_(car_length) {}

const System<double>* Scenario::AddIdmSimpleCar(const std::string& name) {
  std::unique_ptr<systems::DiagramBuilder<double>> builder{
      std::make_unique<systems::DiagramBuilder<double>>()};

  const auto idm_controller =
      builder->template AddSystem<IdmController<double>>(
          *road_, kScanStrategy, kRoadPositionStrategy, kPeriodSec);
  idm_controller->set_name(name + "_idm_controller");

  const auto simple_car = builder->template AddSystem<SimpleCar<double>>();
  simple_car->set_name(name + "_simple_car");

  const auto pursuit =
      builder->template AddSystem<PurePursuitController<double>>();
  pursuit->set_name(name + "_pure_pursuit_controller");
  const auto mux = builder->template AddSystem<systems::Multiplexer<double>>(
      DrivingCommand<double>());
  mux->set_name(name + "_mux");

  // Wire up the simple car to IdmController.
  builder->Connect(simple_car->pose_output(), idm_controller->ego_pose_input());
  builder->Connect(simple_car->velocity_output(),
                   idm_controller->ego_velocity_input());

  // Wire up the simple car to PurePursuitController.
  builder->Connect(simple_car->pose_output(), pursuit->ego_pose_input());
  // Build DrivingCommand via a mux of two scalar outputs.
  builder->Connect(pursuit->steering_command_output(),
                   mux->get_input_port(DrivingCommandIndices::kSteeringAngle));
  builder->Connect(idm_controller->acceleration_output(),
                   mux->get_input_port(DrivingCommandIndices::kAcceleration));

  // TODO(jadecastro) For now, we don't need a model vector ctor for Adder,
  // but *will* need it once AutoDiff with named vectors is resolved.
  const auto adder = builder->template AddSystem<systems::Adder<double>>(2, 2);
  builder->Connect(adder->get_output_port(), simple_car->get_input_port(0));
  builder->Connect(mux->get_output_port(0), adder->get_input_port(0));

  // Export the i/o.
  noise_inport_ = builder->ExportInput(adder->get_input_port(1));
  traffic_inport_ = builder->ExportInput(idm_controller->traffic_input());
  lane_inport_ = builder->ExportInput(pursuit->lane_input());

  pose_outport_ = builder->ExportOutput(simple_car->pose_output());
  velocity_outport_ = builder->ExportOutput(simple_car->velocity_output());

  std::unique_ptr<systems::Diagram<double>> diagram = builder->Build();
  diagram->set_name(name);
  DRAKE_DEMAND(simple_car !=
               nullptr);  // ** TODO ** Remove once we're conviced it is.

  const System<double>* alias = diagram.get();
  ado_cars_.push_back(std::move(diagram));
  return alias;
}

const systems::System<double>* Scenario::AddSimpleCar(const std::string& name) {
  // ** TODO ** Add arbitrary number of ego cars?
  ego_car_ = std::make_unique<SimpleCar<double>>();
  ego_car_->set_name(name);
  ego_alias_ = ego_car_.get();
  return ego_alias_;
}

void Scenario::FixGoalLaneDirection(const System<double>& subsystem,
                                    const LaneDirection& lane_direction) {
  DRAKE_DEMAND(
      subsystem.get_num_input_ports() ==
      3);  // ** TODO ** More targeted way of checking if lane port exists?
  // ** TODO ** fail fast if key exists?
  goal_lane_map_.insert(std::make_pair(&subsystem, lane_direction));
}

void Scenario::Build() {
  // ** TODO ** Demand that all the lane input ports are wired up.
  // ** TODO ** Assert if the ego_car's inputs aren't the DrivingCommands we're
  //            expecting below.
  // ** TODO ** Assert if any of the ado_car's inputs aren't the DrivingCommands
  //            we're expecting below.
  const int num_cars = ado_cars_.size() + 1;
  DRAKE_DEMAND(kEgoPort < num_cars);

  std::unique_ptr<systems::DiagramBuilder<double>> builder{
      std::make_unique<systems::DiagramBuilder<double>>()};

  const int input_size = DrivingCommandIndices::kNumCoordinates * num_cars;
  const auto demux =
      builder->template AddSystem<systems::Demultiplexer<double>>(
          input_size, DrivingCommandIndices::kNumCoordinates);
  demux->set_name("demux");
  const auto aggregator =
      builder->template AddSystem<systems::rendering::PoseAggregator<double>>();
  aggregator->set_name("pose_aggregator");

  // Ego car.
  const auto ego_car_system = builder->AddSystem(std::move(ego_car_));
  aliases_.push_back(ego_car_system);  // ** TODO ** Check that this preserves
                                       // the Context ordering.
  const auto ego_car_alias =
      dynamic_cast<const automotive::SimpleCar<double>*>(ego_car_system);
  DRAKE_DEMAND(ego_car_alias != nullptr);
  builder->Connect(demux->get_output_port(kEgoPort),
                   ego_car_alias->get_input_port(0));

  auto ego_ports = aggregator->AddSinglePoseAndVelocityInput(
      ego_car_alias->get_name(), kEgoPort);
  builder->Connect(ego_car_alias->pose_output(), ego_ports.pose_descriptor);
  builder->Connect(ego_car_alias->velocity_output(),
                   ego_ports.velocity_descriptor);

  // Ado cars.
  // ** TODO ** Watch that the port ordering is aligned with the right system
  //            when called by index below!  Also should agree with
  //            PoseAggregator.
  for (int port{0}; port < num_cars; port++) {
    if (port == kEgoPort) continue;

    auto ado_car = std::move(*ado_cars_.begin());
    ado_cars_.erase(ado_cars_.begin());
    const auto ado_alias = builder->AddSystem(std::move(ado_car));
    aliases_.push_back(ado_alias);  // ** TODO ** Check that this preserves the
                                    // Context ordering.

    const LaneDirection lane_direction = goal_lane_map_.at(ado_alias);
    // ** TODO ** Throw or default to a certain LaneDirection?
    auto lane_source =
        builder->template AddSystem<systems::ConstantValueSource<double>>(
            systems::AbstractValue::Make<LaneDirection>(lane_direction));

    builder->Connect(demux->get_output_port(port),
                     ado_alias->get_input_port(noise_inport_));
    builder->Connect(aggregator->get_output_port(0),
                     ado_alias->get_input_port(traffic_inport_));
    auto ado_ports = aggregator->AddSinglePoseAndVelocityInput(
        ado_alias->get_name(), port);
    builder->Connect(ado_alias->get_output_port(pose_outport_),
                     ado_ports.pose_descriptor);
    builder->Connect(ado_alias->get_output_port(velocity_outport_),
                     ado_ports.velocity_descriptor);
    builder->Connect(lane_source->get_output_port(0),
                     ado_alias->get_input_port(lane_inport_));
  }
  DRAKE_DEMAND(ado_cars_.size() == 0);

  // Export the stacked input.
  builder->ExportInput(demux->get_input_port(0));

  scenario_diagram_ = builder->Build();
  scenario_diagram_->set_name("multi_car_scenario");
  initial_context_lb_ = scenario_diagram_->CreateDefaultContext();
  initial_context_ub_ = scenario_diagram_->CreateDefaultContext();
  final_context_ = scenario_diagram_->CreateDefaultContext();
}

void Scenario::SetInitialSubsystemStateBounds(
    const System<double>& subsystem,
    const SimpleCarState<double>& lb_value,
    const SimpleCarState<double>& ub_value) {
  // ** TODO ** Check that the bounds are consistent.

  DRAKE_DEMAND(scenario_diagram_ != nullptr);
  DRAKE_DEMAND(initial_context_lb_ != nullptr);
  DRAKE_DEMAND(initial_context_ub_ != nullptr);
  auto& subcontext_lb = scenario_diagram_->GetMutableSubsystemContext(
      subsystem, initial_context_lb_.get());
  systems::VectorBase<double>& state_lb =
      subcontext_lb.get_mutable_continuous_state_vector();
  state_lb.SetFromVector(lb_value.get_value());
  auto& subcontext_ub = scenario_diagram_->GetMutableSubsystemContext(
      subsystem, initial_context_ub_.get());
  systems::VectorBase<double>& state_ub =
      subcontext_ub.get_mutable_continuous_state_vector();
  state_ub.SetFromVector(ub_value.get_value());
  // ** TODO ** Collapse with SetFinal..?
}

void Scenario::SetFinalSubsystemState(const System<double>& subsystem,
                                      const SimpleCarState<double>& value) {
  DRAKE_DEMAND(scenario_diagram_ != nullptr);
  DRAKE_DEMAND(final_context_ != nullptr);
  auto& subcontext = scenario_diagram_->GetMutableSubsystemContext(
      subsystem, final_context_.get());
  systems::VectorBase<double>& state =
      subcontext.get_mutable_continuous_state_vector();
  state.SetFromVector(value.get_value());
  // ** TODO ** Collapse with SetInitial..?
}

std::vector<int> Scenario::GetStateIndices(
    const System<double>& subsystem) const {
  DRAKE_DEMAND(aliases_.size() > 0);
  const size_t index = std::distance(
      aliases_.begin(),
      std::find(aliases_.begin(), aliases_.end(), &subsystem));
  if (index == aliases_.size()) {
    throw std::runtime_error("The provided subsystem was not found.");
  }
  auto result = std::vector<int>(SimpleCarStateIndices::kNumCoordinates);
  std::iota(std::begin(result), std::end(result),
            index * SimpleCarStateIndices::kNumCoordinates);
  return result;
}

AutomotiveTrajectoryOptimization::AutomotiveTrajectoryOptimization(
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

void AutomotiveTrajectoryOptimization::FixInitialConditions() {
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

void AutomotiveTrajectoryOptimization::SetLinearGuessTrajectory() {
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
void AutomotiveTrajectoryOptimization::SetDragwayLaneBounds(
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

  const SimpleCarSymbolicState state = get_state(&subsystem);
  const double w = scenario().car_width();
  const double l = scenario().car_length();
  const auto y00 =
      state.y + (l / 2.) * sin(state.heading) - (w / 2.) * cos(state.heading);
  const auto y10 =
      state.y - (l / 2.) * sin(state.heading) - (w / 2.) * cos(state.heading);
  const auto y01 =
      state.y - (l / 2.) * sin(state.heading) + (w / 2.) * cos(state.heading);
  const auto y11 =
      state.y + (l / 2.) * sin(state.heading) + (w / 2.) * cos(state.heading);

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

void AutomotiveTrajectoryOptimization::AddFinalCollisionConstraints(
    const System<double>& subsystem) {
  using std::cos;
  using std::sin;

  const std::vector<int> ego_indices =
      scenario_->GetStateIndices(*scenario_->ego_alias());
  auto ego_state = prog_->final_state().segment(
      ego_indices[0], SimpleCarStateIndices::kNumCoordinates);
  const auto x_ego = ego_state[kX];
  const auto y_ego = ego_state[kY];
  const auto heading_ego = ego_state[kHeading];
  const std::vector<int> ado_indices = scenario_->GetStateIndices(subsystem);
  auto ado_state = prog_->final_state().segment(
      ado_indices[0], SimpleCarStateIndices::kNumCoordinates);
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

void AutomotiveTrajectoryOptimization::AddGaussianCost() {
  const double kSigma = 50.;
  const double kCoeff = 1. / (2. * kSigma * kSigma);
  symbolic::Expression running_cost;
  for (int i{0}; i < prog_->input().size(); ++i) {
    std::cout << " prog->input()(i) " << prog_->input()(i) << std::endl;
    running_cost += kCoeff * prog_->input()(i) * prog_->input()(i);
  }
  prog_->AddRunningCost(running_cost);
}

void AutomotiveTrajectoryOptimization::SetEgoLinearConstraint(
    const Eigen::Ref<const Eigen::MatrixXd> A,
    const Eigen::Ref<const Eigen::VectorXd> b, double t) {
  DRAKE_DEMAND(min_time_step_ == max_time_step_);
  DRAKE_DEMAND(t >= 0 && t <= min_time_step_ * num_time_samples_);
  DRAKE_DEMAND(A.cols() == SimpleCarStateIndices::kNumCoordinates);

  // Assume a zero-order hold on the constraints application.
  const int index = std::ceil(t / min_time_step_);
  const std::vector<int> ego_indices =
      scenario_->GetStateIndices(*scenario_->ego_alias());
  auto x_ego = prog_->state(index).segment(
      ego_indices[0], SimpleCarStateIndices::kNumCoordinates);
  prog_->AddLinearConstraint(A, -b * std::numeric_limits<double>::infinity(), b,
                             x_ego);
  // prog_->AddLinearConstraint(A * x_ego <= b);  // ** TODO ** Enable this spelling.
}

void AutomotiveTrajectoryOptimization::Solve() {
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

SimpleCarSymbolicState AutomotiveTrajectoryOptimization::get_state(
    const System<double>* subsystem) const {
  DRAKE_DEMAND(subsystem != nullptr);
  const std::vector<int> car_indices = scenario().GetStateIndices(*subsystem);
  SimpleCarSymbolicState car_state;
  car_state.x = prog_->state()(car_indices.at(kX));
  car_state.y = prog_->state()(car_indices.at(kY));
  car_state.heading = prog_->state()(car_indices.at(kHeading));
  car_state.velocity = prog_->state()(car_indices.at(kVelocity));
  return car_state;
}

SimpleCarSymbolicState AutomotiveTrajectoryOptimization::get_initial_state(
    const System<double>* subsystem) const {
  DRAKE_DEMAND(subsystem != nullptr);
  const std::vector<int> car_indices = scenario().GetStateIndices(*subsystem);
  SimpleCarSymbolicState car_state;
  car_state.x = prog_->initial_state()(car_indices.at(kX));
  car_state.y = prog_->initial_state()(car_indices.at(kY));
  car_state.heading = prog_->initial_state()(car_indices.at(kHeading));
  car_state.velocity = prog_->initial_state()(car_indices.at(kVelocity));
  return car_state;
}

SimpleCarSymbolicState AutomotiveTrajectoryOptimization::get_final_state(
    const System<double>* subsystem) const {
  DRAKE_DEMAND(subsystem != nullptr);
  const std::vector<int> car_indices = scenario().GetStateIndices(*subsystem);
  SimpleCarSymbolicState car_state;
  car_state.x = prog_->final_state()(car_indices.at(kX));
  car_state.y = prog_->final_state()(car_indices.at(kY));
  car_state.heading = prog_->final_state()(car_indices.at(kHeading));
  car_state.velocity = prog_->final_state()(car_indices.at(kVelocity));
  return car_state;
}

double AutomotiveTrajectoryOptimization::GetSolutionTotalProbability() const {
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

void AutomotiveTrajectoryOptimization::PlotSolution() {
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

void AutomotiveTrajectoryOptimization::AnimateSolution() const {
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
