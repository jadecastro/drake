#include "drake/automotive/scenario.h"

#include <memory>
#include <vector>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/pure_pursuit_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/rendering/pose_aggregator.h"

namespace drake {
namespace automotive {

using maliput::api::Lane;
using maliput::api::RoadGeometry;
using systems::Diagram;
using systems::System;

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

}  // namespace automotive
}  // namespace drake
