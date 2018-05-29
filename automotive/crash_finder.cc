#include <algorithm>
#include <memory>
#include <vector>

#include <gflags/gflags.h>

#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/pure_pursuit_controller.h"
#include "drake/automotive/simple_car.h"
#include "drake/common/proto/call_python.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/rendering/pose_aggregator.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace automotive {
namespace {

using common::CallPython;
using maliput::api::Lane;
using maliput::api::RoadGeometry;
using systems::Diagram;
using systems::System;
using systems::trajectory_optimization::DirectCollocation;
using trajectories::PiecewisePolynomial;

static constexpr int kX = SimpleCarStateIndices::kX;
static constexpr int kY = SimpleCarStateIndices::kY;

// Scenario parameters.
static constexpr int kNumAdoCars = 3;
static constexpr int kNumLanes = 3;  // Number of lanes in the scenario.
static constexpr double kLaneWidth = 3.;
static constexpr double kDragwayLength = 150.;

// Diagram wiring details.
static constexpr int kNoiseInputPort = 0;
static constexpr int kTrafficInputPort = 1;
static constexpr int kGoalLaneInputPort = 2;

static constexpr int kPoseOutputPort = 0;
static constexpr int kVelocityOutputPort = 1;

static constexpr int kEgoPort = 0;  // Identifier for both Demux and PoseAggregator.

// Model parameters.
static constexpr double kPeriodSec = 0.1;  // For IDM.
static constexpr ScanStrategy kScanStrategy = ScanStrategy::kPath;
static constexpr RoadPositionStrategy kRoadPositionStrategy =
    RoadPositionStrategy::kExhaustiveSearch;

// Solver settings.
static constexpr double kGuessDurationSec = 10.;  // seconds
static constexpr int kNumTimeSamples = 21;

std::unique_ptr<RoadGeometry> MakeDragway(int num_dragway_lanes) {
  return std::make_unique<maliput::dragway::RoadGeometry>(
      maliput::api::RoadGeometryId({"dragway"}), num_dragway_lanes,
      kDragwayLength, kLaneWidth, 0. /* shoulder width */,
      5. /* maximum_height */,
      std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance */);
}

// ** TODO ** Does it make sense for this to be a class rather than a function?
// (Here, we have to worry about tranferring unique_ptrs in the main function)
std::pair<std::unique_ptr<Diagram<double>>, const System<double>*>
MakeIdmSimpleCarSubsystem(const std::string& name, const RoadGeometry& road) {
  std::unique_ptr<systems::DiagramBuilder<double>> builder{
      std::make_unique<systems::DiagramBuilder<double>>()};

  const auto idm_controller = builder->template AddSystem<IdmController<double>>(
      road, kScanStrategy, kRoadPositionStrategy, kPeriodSec);
  idm_controller->set_name(name + "_idm_controller");

  const auto simple_car = builder->template AddSystem<SimpleCar<double>>();
  simple_car->set_name(name + "_simple_car");

  const auto pursuit = builder->template AddSystem<PurePursuitController<double>>();
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
  builder->Connect(mux->get_output_port(0), adder->get_input_port(0));
  builder->Connect(adder->get_output_port(), simple_car->get_input_port(0));

  // Export the i/o.
  const int noise_port = builder->ExportInput(adder->get_input_port(1));
  const int traffic_port =
      builder->ExportInput(idm_controller->traffic_input());
  const int lane_port = builder->ExportInput(pursuit->lane_input());

  const int pose_port = builder->ExportOutput(simple_car->pose_output());
  const int velocity_port = builder->ExportOutput(simple_car->velocity_output());

  // Seems silly, but all this will go away once this function becomes a member
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
  return std::make_pair(std::move(diagram), simple_car);
}

std::unique_ptr<Diagram<double>> MakeScenarioDiagram(
    const std::map<const Diagram<double>*, LaneDirection> goal_lane_map,
    std::vector<std::unique_ptr<Diagram<double>>>* diagrams,
    std::unique_ptr<System<double>> ego_car) {
  // ** TODO ** Demand that all the lane input ports are wired up.
  // ** TODO ** Assert if the ego_car's inputs aren't the DrivingCommands we're
  //            expecting below.
  // ** TODO ** Assert if any of the ado_car's inputs aren't the DrivingCommands
  //            we're expecting below.
  const int num_cars = diagrams->size() + 1;

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
  const auto ego_car_system = builder->AddSystem(std::move(ego_car));
  const auto ego_car_alias = dynamic_cast<const automotive::SimpleCar<double>*>(
      ego_car_system);
  DRAKE_DEMAND(ego_car_alias != nullptr);
  builder->Connect(demux->get_output_port(kEgoPort),
                   ego_car_alias->get_input_port(0));
  {
    auto ports = aggregator->AddSinglePoseAndVelocityInput(
        ego_car_alias->get_name(), kEgoPort);
    builder->Connect(ego_car_alias->pose_output(), ports.pose_descriptor);
    builder->Connect(ego_car_alias->velocity_output(), ports.velocity_descriptor);
  }

  // Ado cars.
  // ** TODO ** Watch that the port ordering is aligned with the right system
  //            when called by index below!  Also should agree with
  //            PoseAggregator.
  int demux_port{1};
  while (diagrams->size() > 0) {
    DRAKE_DEMAND(demux_port != kEgoPort);
    auto diagram = std::move(*diagrams->begin());
    diagrams->erase(diagrams->begin());
    const auto diagram_alias = builder->AddSystem(std::move(diagram));
    const LaneDirection lane_direction = goal_lane_map.at(diagram_alias);
    auto lane_source =
        builder->template AddSystem<systems::ConstantValueSource<double>>(
            systems::AbstractValue::Make<LaneDirection>(lane_direction));

    builder->Connect(demux->get_output_port(demux_port++),
                     diagram_alias->get_input_port(kNoiseInputPort));
    builder->Connect(aggregator->get_output_port(0),
                     diagram_alias->get_input_port(kTrafficInputPort));
    auto ports =
        aggregator->AddSinglePoseAndVelocityInput(
            diagram_alias->get_name(), kEgoPort);
    builder->Connect(diagram_alias->get_output_port(kPoseOutputPort),
                     ports.pose_descriptor);
    builder->Connect(diagram_alias->get_output_port(kVelocityOutputPort),
                     ports.velocity_descriptor);
    builder->Connect(lane_source->get_output_port(0),
                     diagram_alias->get_input_port(kGoalLaneInputPort));
  }

  // Export the stacked input.
  builder->ExportInput(demux->get_input_port(0));

  std::unique_ptr<systems::Diagram<double>> diagram = builder->Build();
  diagram->set_name("crash_finder_scenario");

  return diagram;
}

std::vector<int> GetEgoIndices(const System<double>* ego_car_alias,
                               std::vector<const System<double>*> systems) {
  int index{0};
  std::vector<int> result(SimpleCarStateIndices::kNumCoordinates);
  // ** TODO ** Use find?
  for (const auto& system : systems) {
    if (system == ego_car_alias) {
      std::iota(std::begin(result), std::end(result), index);
      break;
    }
    index += SimpleCarStateIndices::kNumCoordinates;
  }
  return result;
}

std::vector<std::vector<int>> GetAdoIndices(
    const std::vector<const System<double>*>& ado_car_aliases,
    std::vector<const System<double>*> systems) {
  std::vector<std::vector<int>> result{};
  // ** TODO ** Use std::find()
  for (const auto& alias : ado_car_aliases) {
    int index{0};
    std::vector<int> car_result(SimpleCarStateIndices::kNumCoordinates);
    for (const auto& system : systems) {
      if (system == alias) {
        std::iota(std::begin(car_result), std::end(car_result), index);
        break;
      }
      index += SimpleCarStateIndices::kNumCoordinates;
    }
    result.push_back(car_result);
  }
  return result;
}

void SetSubsystemState(const SimpleCarState<double>& value,
                       systems::Context<double>* context) {
  systems::VectorBase<double>& context_state =
      context->get_mutable_continuous_state_vector();
  SimpleCarState<double>* const state =
      dynamic_cast<SimpleCarState<double>*>(&context_state);
  DRAKE_ASSERT(state);
  state->set_value(value.get_value());
}

std::unique_ptr<DirectCollocation> InitializeFalsifier(
    systems::Diagram<double>& finalized_diagram) {
  const auto& plant =
      dynamic_cast<const systems::System<double>&>(finalized_diagram);  // Need?
  // Set up a direct-collocation problem.
  const double kMinTimeStep = 0.2 * kGuessDurationSec / (kNumTimeSamples - 1);
  const double kMaxTimeStep = 3. * kGuessDurationSec / (kNumTimeSamples - 1);
  const double kLimit = 100.;

  auto context = plant.CreateDefaultContext();
  std::unique_ptr<DirectCollocation> prog(new DirectCollocation(
      &plant, *context, kNumTimeSamples, kMinTimeStep, kMaxTimeStep));

  // Ensure that time intervals are evenly spaced.
  prog->AddEqualTimeIntervalsConstraints();

  // Generous bounding box on all decision variables.
  prog->AddBoundingBoxConstraint(-kLimit, kLimit, prog->decision_variables());

  return prog;
}

void FixInitialConditions(const systems::Context<double>& initial_context,
                          DirectCollocation* prog) {
  // Parse the initial conditions and set a constraint there.
  const systems::VectorBase<double>& initial_states =
      initial_context.get_continuous_state_vector();
  prog->AddLinearConstraint(prog->initial_state() ==
                            initial_states.CopyToVector());
}

void SetLinearGuessTrajectory(const systems::Context<double>& initial_context,
                              const systems::Context<double>& final_context,
                              DirectCollocation* prog) {
  const Eigen::VectorXd x0 =
      initial_context.get_continuous_state_vector().CopyToVector();
  const Eigen::VectorXd xf =
      final_context.get_continuous_state_vector().CopyToVector();
  auto guess_state_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      {0, kGuessDurationSec}, {x0, xf});
  prog->SetInitialTrajectory(PiecewisePolynomial<double>(),
                             guess_state_trajectory);
}

/*
// N.B. Assumes Dragway.
void SetLateralLaneBounds(const std::vector<int>& car_indices,
                          std::pair<const Lane*, const Lane*> lane_bounds,
                          DirectCollocation* prog) {
  std::vector<double> y_bounds{};
  y_bounds.push_back(lane_bounds.first->ToGeoPosition(
      {0., lane_bounds.first->lane_bounds(0.).min(), 0.}).y());
  y_bounds.push_back(lane_bounds.first->ToGeoPosition(
      {0., lane_bounds.first->lane_bounds(0.).max(), 0.}).y());
  y_bounds.push_back(lane_bounds.second->ToGeoPosition(
      {0., lane_bounds.first->lane_bounds(0.).min(), 0.}).y());
  y_bounds.push_back(lane_bounds.second->ToGeoPosition(
      {0., lane_bounds.first->lane_bounds(0.).max(), 0.}).y());
  prog->AddConstraintToAllKnotPoints(
      prog->state()(car_indices.at(kY)) >=
      *std::min_element(y_bounds.begin(), y_bounds.end()));
  prog->AddConstraintToAllKnotPoints(
      prog->state()(car_indices.at(kY)) <=
      *std::max_element(y_bounds.begin(), y_bounds.end()));
}
*/
// Add cost representing the noise perturbation from a deterministic evolution
// of IDM/PurePursuit (nominal policy).  We assume that the policy has a
// state-independent, zero-mean Gaussian distribution.
// ** TODO ** Separate out Sigmas for steering and acceleration.
void AddLogProbabilityCost(DirectCollocation* prog) {
  const double kSigma = 50.;
  const double kCoeff = 1. / (2. * kSigma * kSigma);
  symbolic::Expression running_cost;
  for (int i{0}; i < prog->input().size(); ++i) {
    std::cout << " prog->input()(i) " << prog->input()(i) << std::endl;
    running_cost += kCoeff * prog->input()(i) * prog->input()(i);
  }
  prog->AddRunningCost(running_cost);
}

// Add a log-probability contraints representing the deviation of the commands
// from a deterministic evolution of IDM/PurePursuit (nominal policy).  We
// assume that the policy has zero-mean Gaussian distribution.  ** TODO **
// Separate out Sigmas for steering and acceleration.
void AddLogProbabilityChanceConstraint(DirectCollocation* prog) {
  using std::log;
  using std::sqrt;

  const double kSigma = 50.;
  const double kSamplePathProb = 0.5;
  const double kP = 2. * kNumTimeSamples * kSigma * kSigma *
                        log(1. / (sqrt(2. * M_PI) * kSigma)) -
                    2. * kSigma * kSigma * log(kSamplePathProb);
  drake::unused(kP);
  for (int i{0}; i < prog->input().size(); ++i) {
    std::cout << " prog->input()(i) " << prog->input()(i) << std::endl;
    // prog->AddConstraint(prog->input()(i) * prog->input()(i) >= kP);
  }
}

void PlotResult(const std::vector<int>& ego_indices,
                const std::vector<std::vector<int>>& ado_indices,
                const solvers::SolutionResult& result,
                const DirectCollocation& prog) {
  const Eigen::MatrixXd inputs = prog.GetInputSamples();
  const Eigen::MatrixXd states = prog.GetStateSamples();
  const Eigen::VectorXd times_out = prog.GetSampleTimes();
  const Eigen::VectorXd ego_x = states.row(ego_indices.at(kX));
  const Eigen::VectorXd ego_y = states.row(ego_indices.at(kY));
  std::cout << " Sample times: " << times_out << std::endl;
  std::cout << " Inputs for ego car: " << inputs.row(0) << std::endl;
  std::cout << "                     " << inputs.row(1) << std::endl;
  // TODO(jadecastro) Transform this cost into a probability distribution.
  std::cout << " Optimal Cost: " << prog.GetOptimalCost() << std::endl;
  CallPython("figure", 1);
  CallPython("clf");
  CallPython("plot", ego_x, ego_y);
  CallPython("setvars", "ego_x", ego_x, "ego_y", ego_y);
  CallPython("plt.xlabel", "ego car x (m)");
  CallPython("plt.ylabel", "ego car y (m)");
  CallPython("plt.show");
  for (int i{0}; i < kNumAdoCars; ++i) {
    const Eigen::VectorXd ado_x = states.row(ado_indices[i].at(kX));
    const Eigen::VectorXd ado_y = states.row(ado_indices[i].at(kY));
    CallPython("figure", i + 2);
    CallPython("clf");
    CallPython("plot", ado_x, ado_y);
    CallPython("setvars", "ado_x_" + std::to_string(i), ado_x,
               "ado_y_" + std::to_string(i), ado_y);
    CallPython("plt.xlabel", "ado car " + std::to_string(i) + " x (m)");
    CallPython("plt.ylabel", "ado car " + std::to_string(i) + " y (m)");
    CallPython("plt.show");
  }

  // Dump the entire solution result to the screen.
  // for (int i{0}; i < states.size(); ++i) {
  //   std::cout << " states(" << i << ") " << states(i) << std::endl;
  // }

  std::cout << " SOLUTION RESULT: " << result << std::endl;
}

void SimulateResult(const DirectCollocation& prog) {
  const Eigen::MatrixXd inputs = prog.GetInputSamples();
  const Eigen::MatrixXd states = prog.GetStateSamples();
  const Eigen::VectorXd times_out = prog.GetSampleTimes();
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

int DoMain(void) {
  std::unique_ptr<RoadGeometry> road = MakeDragway(kNumLanes);
  const maliput::api::Segment* segment = road->junction(0)->segment(0);

  // Make three ado cars.
  std::vector<std::unique_ptr<Diagram<double>>> ado_car_subsystems(kNumAdoCars);
  std::vector<const System<double>*> ado_car_aliases(kNumAdoCars);
  std::tie(ado_car_subsystems[0], ado_car_aliases[0]) =
      MakeIdmSimpleCarSubsystem("ado_car_0", *road);
  std::tie(ado_car_subsystems[1], ado_car_aliases[1]) =
      MakeIdmSimpleCarSubsystem("ado_car_1", *road);
  std::tie(ado_car_subsystems[2], ado_car_aliases[2]) =
      MakeIdmSimpleCarSubsystem("ado_car_2", *road);

  // Make an ego car.
  auto ego_car = std::make_unique<SimpleCar<double>>();
  ego_car->set_name("ego_car");
  auto ego_car_alias = ego_car.get();

  // Fix the goal lanes for each of the ado cars.
  std::map<const Diagram<double>*, LaneDirection> goal_lane_map;
  goal_lane_map.insert(std::make_pair(
      ado_car_subsystems[0].get(), LaneDirection(segment->lane(0), true)));
  goal_lane_map.insert(std::make_pair(
      ado_car_subsystems[1].get(), LaneDirection(segment->lane(1), true)));
  goal_lane_map.insert(std::make_pair(
      ado_car_subsystems[2].get(), LaneDirection(segment->lane(2), true)));

  std::unique_ptr<Diagram<double>> scenario_diagram =
      MakeScenarioDiagram(goal_lane_map, &ado_car_subsystems, std::move(ego_car));
  std::vector<const System<double>*> systems = scenario_diagram->GetSystems();
  auto context = scenario_diagram->CreateDefaultContext();
  // auto diagram_context = dynamic_cast<systems::DiagramContext<double>*>(context.get());
  // DRAKE_DEMAND(diagram_context != nullptr);

  // Supply initial conditions (used as falsification constraints and for the
  // initial guess trajectory).
  auto initial_context = context->Clone();
  SimpleCarState<double> initial_conditions;
  initial_conditions.set_x(30.);
  initial_conditions.set_y(-1. * kLaneWidth);
  initial_conditions.set_heading(0.);
  initial_conditions.set_velocity(7.);
  SetSubsystemState(initial_conditions,
                    &scenario_diagram->GetMutableSubsystemContext(
                        *ego_car_alias, initial_context.get()));

  // ** TODO ** Collapse the above to two arguments, by making a class member.
  //            (Requires us to load ICs externally).

  initial_conditions.set_x(40.);
  initial_conditions.set_y(segment->lane(0)->ToGeoPosition({0., 0., 0.}).y());
  initial_conditions.set_heading(0.);
  initial_conditions.set_velocity(5.);
  SetSubsystemState(initial_conditions,
                    &scenario_diagram->GetMutableSubsystemContext(
                        *ado_car_aliases[0], initial_context.get()));

  initial_conditions.set_x(60.);
  initial_conditions.set_y(segment->lane(1)->ToGeoPosition({0., 0., 0.}).y());
  initial_conditions.set_heading(0.);
  initial_conditions.set_velocity(8.);
  SetSubsystemState(initial_conditions,
                    &scenario_diagram->GetMutableSubsystemContext(
                        *ado_car_aliases[1], initial_context.get()));

  initial_conditions.set_x(10.);
  initial_conditions.set_y(segment->lane(2)->ToGeoPosition({0., 0., 0.}).y());
  initial_conditions.set_heading(0.);
  initial_conditions.set_velocity(5.);
  SetSubsystemState(initial_conditions,
                    &scenario_diagram->GetMutableSubsystemContext(
                        *ado_car_aliases[2], initial_context.get()));

  // Supply final conditions (only used for the initial guess trajectory).
  auto final_context = context->Clone();
  SimpleCarState<double> final_conditions;
  final_conditions.set_x(40.);
  final_conditions.set_y(-0.6 * kLaneWidth);
  final_conditions.set_heading(0.);
  final_conditions.set_velocity(5.0);
  SetSubsystemState(final_conditions,
                    &scenario_diagram->GetMutableSubsystemContext(
                        *ego_car_alias, final_context.get()));

  final_conditions.set_x(60.);
  final_conditions.set_y(segment->lane(0)->ToGeoPosition({0., 0., 0.}).y());
  final_conditions.set_heading(0.);
  final_conditions.set_velocity(5.);
  SetSubsystemState(final_conditions,
                    &scenario_diagram->GetMutableSubsystemContext(
                        *ado_car_aliases[0], final_context.get()));

  final_conditions.set_x(80.);
  final_conditions.set_y(segment->lane(1)->ToGeoPosition({0., 0., 0.}).y());
  final_conditions.set_heading(0.);
  final_conditions.set_velocity(8.);
  SetSubsystemState(final_conditions,
                    &scenario_diagram->GetMutableSubsystemContext(
                        *ado_car_aliases[1], final_context.get()));

  final_conditions.set_x(40.);
  final_conditions.set_y(segment->lane(2)->ToGeoPosition({0., 0., 0.}).y());
  final_conditions.set_heading(0.);
  final_conditions.set_velocity(6.);
  SetSubsystemState(final_conditions,
                    &scenario_diagram->GetMutableSubsystemContext(
                        *ado_car_aliases[2], final_context.get()));

  /*
  std::cout << " initial states \n "
            << initial_context->get_continuous_state_vector().CopyToVector()
            << std::endl;
  std::cout << " final states \n"
            << final_context->get_continuous_state_vector().CopyToVector()
            << std::endl;
  */

  std::unique_ptr<DirectCollocation> prog =
      InitializeFalsifier(*scenario_diagram);

  FixInitialConditions(*initial_context, prog.get());

  // Getters for the subsystem (SimpleCar) states.
  const std::vector<int> ego_indices = GetEgoIndices(ego_car_alias, systems);
  const std::vector<std::vector<int>> ado_indices =
      GetAdoIndices(ado_car_aliases, systems);

  // Constraints keeping the cars on the road or in their lanes.
  std::pair<const Lane*, const Lane*> lane_bounds;
  lane_bounds.first = segment->lane(0);
  lane_bounds.second = segment->lane(2);
  // SetLateralLaneBounds(ego_indices, lane_bounds, prog.get());
  // SetLateralLaneBounds(ado_indices[0], lane_bounds, prog.get());
  // SetLateralLaneBounds(ado_indices[1], lane_bounds, prog.get());
  // SetLateralLaneBounds(ado_indices[2], lane_bounds, prog.get());

  // Solve a collision with one of the cars, in this case, Traffic Car 2 merging
  // into the middle lane.  Assume for now that, irrespective of its
  // orientation, the traffic car occupies a lane-aligned bounding box.
  prog->AddLinearConstraint(prog->final_state()(ado_indices[2].at(kY)) <=
                            prog->final_state()(ego_indices.at(kY)) +
                                0.5 * kLaneWidth);
  prog->AddLinearConstraint(prog->final_state()(ado_indices[2].at(kX)) >=
                            prog->final_state()(ego_indices.at(kX)) - 2.5);
  prog->AddLinearConstraint(prog->final_state()(ado_indices[2].at(kX)) <=
                            prog->final_state()(ego_indices.at(kX)) + 2.5);

  AddLogProbabilityCost(prog.get());

  AddLogProbabilityChanceConstraint(prog.get());

  SetLinearGuessTrajectory(*initial_context, *final_context, prog.get());

  const auto result = prog->Solve();

  // Extract the initial context from prog and plot the solution using MATLAB.
  // To view, type `bazel run //common/proto:call_python_client_cli`.
  PlotResult(ego_indices, ado_indices, result, *prog);

  SimulateResult(*prog);

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::automotive::DoMain();
}
