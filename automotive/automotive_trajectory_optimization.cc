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

// Diagram wiring details.
static constexpr int kNoiseInputPort = 0;
static constexpr int kTrafficInputPort = 1;
static constexpr int kGoalLaneInputPort = 2;

static constexpr int kPoseOutputPort = 0;
static constexpr int kVelocityOutputPort = 1;

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

Scenario::Scenario(int num_lanes, double lane_width, double road_length)
    : road_(MakeDragway(num_lanes, lane_width, road_length)) {}

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
  const int noise_port = builder->ExportInput(adder->get_input_port(1));
  const int traffic_port =
      builder->ExportInput(idm_controller->traffic_input());
  const int lane_port = builder->ExportInput(pursuit->lane_input());

  const int pose_port = builder->ExportOutput(simple_car->pose_output());
  const int velocity_port =
      builder->ExportOutput(simple_car->velocity_output());

  // Seems silly. All of this should go away once this function becomes a member
  // of a class.
  DRAKE_DEMAND(noise_port == kNoiseInputPort);
  DRAKE_DEMAND(traffic_port == kTrafficInputPort);
  DRAKE_DEMAND(lane_port == kGoalLaneInputPort);
  DRAKE_DEMAND(pose_port == kPoseOutputPort);
  DRAKE_DEMAND(velocity_port == kVelocityOutputPort);

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
  return ego_car_.get();
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
  int demux_port{1};
  while (ado_cars_.size() > 0) {
    DRAKE_DEMAND(demux_port != kEgoPort);
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

    builder->Connect(demux->get_output_port(demux_port++),
                     ado_alias->get_input_port(kNoiseInputPort));
    builder->Connect(aggregator->get_output_port(0),
                     ado_alias->get_input_port(kTrafficInputPort));
    auto ado_ports = aggregator->AddSinglePoseAndVelocityInput(
        ado_alias->get_name(), kEgoPort);
    builder->Connect(ado_alias->get_output_port(kPoseOutputPort),
                     ado_ports.pose_descriptor);
    builder->Connect(ado_alias->get_output_port(kVelocityOutputPort),
                     ado_ports.velocity_descriptor);
    builder->Connect(lane_source->get_output_port(0),
                     ado_alias->get_input_port(kGoalLaneInputPort));
  }

  // Export the stacked input.
  builder->ExportInput(demux->get_input_port(0));

  scenario_diagram_ = builder->Build();
  scenario_diagram_->set_name("multi_car_scenario");
  initial_context_ = scenario_diagram_->CreateDefaultContext();
  final_context_ = initial_context_->Clone();
}

void Scenario::SetInitialSubsystemState(const System<double>& subsystem,
                                        const SimpleCarState<double>& value) {
  DRAKE_DEMAND(scenario_diagram_ != nullptr);
  DRAKE_DEMAND(initial_context_ != nullptr);
  auto& subcontext = scenario_diagram_->GetMutableSubsystemContext(
      subsystem, initial_context_.get());
  systems::VectorBase<double>& state =
      subcontext.get_mutable_continuous_state_vector();
  state.SetFromVector(value.get_value());
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
  const auto it = std::find(aliases_.begin(), aliases_.end(), &subsystem);
  const int index = it - aliases_.begin();
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
  const systems::VectorBase<double>& initial_states =
      scenario_->initial_context().get_continuous_state_vector();
  prog_->AddLinearConstraint(prog_->initial_state() ==
                             initial_states.CopyToVector());
}

void AutomotiveTrajectoryOptimization::SetLinearGuessTrajectory() {
  const Eigen::VectorXd x0 =
      scenario().initial_context().get_continuous_state_vector().CopyToVector();
  const Eigen::VectorXd xf =
      scenario().final_context().get_continuous_state_vector().CopyToVector();
  auto guess_state_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      {0, initial_guess_duration_sec_}, {x0, xf});
  prog_->SetInitialTrajectory(PiecewisePolynomial<double>(),
                              guess_state_trajectory);
}

// N.B. Assumes Dragway.
void AutomotiveTrajectoryOptimization::SetLateralLaneBounds(
    const System<double>* subsystem,
    std::pair<const Lane*, const Lane*> lane_bounds) {
  std::vector<double> y_bounds{};
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

  const std::vector<int> car_indices = scenario().GetStateIndices(*subsystem);
  prog_->AddConstraintToAllKnotPoints(
      prog_->state()(car_indices.at(kY)) >=
      *std::min_element(y_bounds.begin(), y_bounds.end()));
  prog_->AddConstraintToAllKnotPoints(
      prog_->state()(car_indices.at(kY)) <=
      *std::max_element(y_bounds.begin(), y_bounds.end()));
}

void AutomotiveTrajectoryOptimization::SetEgoLinearConstraint(
    const Eigen::Ref<const Eigen::MatrixXd> A,
    const Eigen::Ref<const Eigen::VectorXd> b, double t) {
  DRAKE_DEMAND(min_time_step_ == max_time_step_);
  // ** TODO ** Think about how to relax this constraint?

  // Assume a zero-order hold on the constraints application.
  // ** TODO ** Replace with std::find().
  int index{0};
  // while (t > index++ * min_time_step_) {}
  const std::vector<int> ego_indices =
      scenario_->GetStateIndices(*scenario_->ego_alias());
  auto x_ego = prog_->state(index).segment(
      ego_indices[0], SimpleCarStateIndices::kNumCoordinates);
  prog_->AddLinearConstraint(A, -b * std::numeric_limits<double>::infinity(), b,
                             x_ego);
  // prog_->AddLinearConstraint(A * x_ego <= b);
}

// Add cost representing the noise perturbation from a deterministic evolution
// of IDM/PurePursuit (nominal policy).  We assume that the policy has a
// state-independent, zero-mean Gaussian distribution.
// ** TODO ** Separate out Sigmas for steering and acceleration.
void AutomotiveTrajectoryOptimization::AddLogProbabilityCost() {
  const double kSigma = 50.;
  const double kCoeff = 1. / (2. * kSigma * kSigma);
  symbolic::Expression running_cost;
  for (int i{0}; i < prog_->input().size(); ++i) {
    std::cout << " prog->input()(i) " << prog_->input()(i) << std::endl;
    running_cost += kCoeff * prog_->input()(i) * prog_->input()(i);
  }
  prog_->AddRunningCost(running_cost);
}

// Add a log-probability contraints representing the deviation of the commands
// from a deterministic evolution of IDM/PurePursuit (nominal policy).  We
// assume that the policy has zero-mean Gaussian distribution.  ** TODO **
// Separate out Sigmas for steering and acceleration.
void AutomotiveTrajectoryOptimization::AddLogProbabilityChanceConstraint() {
  using std::log;
  using std::sqrt;

  const double kSigma = 50.;
  const double kSamplePathProb = 0.5;
  const double kP = 2. * num_time_samples_ * kSigma * kSigma *
                        log(1. / (sqrt(2. * M_PI) * kSigma)) -
                    2. * kSigma * kSigma * log(kSamplePathProb);
  drake::unused(kP);
  for (int i{0}; i < prog_->input().size(); ++i) {
    std::cout << " prog->input()(i) " << prog_->input()(i) << std::endl;
    // prog_->AddConstraint(prog_->input()(i) * prog_->input()(i) >= kP);
  }
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

void AutomotiveTrajectoryOptimization::PlotResult() {
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

void AutomotiveTrajectoryOptimization::SimulateResult() const {
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
