#include "drake/automotive/scenario.h"

#include <memory>
#include <vector>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/driving_command_adder.h"
#include "drake/automotive/driving_command_demux.h"
#include "drake/automotive/driving_command_mux.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"

namespace drake {
namespace automotive {

using maliput::api::RoadGeometry;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::InputPort;
using systems::OutputPort;
using systems::System;
using systems::VectorBase;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseAggregator;
using systems::rendering::PoseVector;

// Model parameters.
static constexpr double kPeriodSec = 0.1;  // For IDM periodic update.
static constexpr ScanStrategy kScanStrategy = ScanStrategy::kPath;
static constexpr RoadPositionStrategy kRoadPositionStrategy =
    RoadPositionStrategy::kExhaustiveSearch;

IdmSimpleCar::IdmSimpleCar(std::string name, const RoadGeometry& road)
    : Diagram<double>() {
  set_name(name);
  DiagramBuilder<double> builder;

  idm_controller_ = builder.template AddSystem<IdmController<double>>(
      road, kScanStrategy, kRoadPositionStrategy, kPeriodSec);
  idm_controller_->set_name(name + "_idm_controller");

  simple_car_ = builder.template AddSystem<SimpleCar<double>>();
  simple_car_->set_name(name + "_simple_car");

  pursuit_ = builder.template AddSystem<PurePursuitController<double>>();
  pursuit_->set_name(name + "_pure_pursuit_controller");
  const auto mux = builder.template AddSystem<DrivingCommandMux<double>>();
  mux->set_name(name + "_mux");

  builder.Connect(simple_car_->pose_output(), idm_controller_->ego_pose_input());
  builder.Connect(simple_car_->velocity_output(),
                  idm_controller_->ego_velocity_input());
  builder.Connect(simple_car_->pose_output(), pursuit_->ego_pose_input());

  builder.Connect(pursuit_->steering_command_output(), mux->steering_input());
  builder.Connect(idm_controller_->acceleration_output(),
                  mux->acceleration_input());

  const auto adder = builder.template AddSystem<DrivingCommandAdder<double>>(
      2 /*num inputs*/);
  const int kAdderValuePort = 0;
  const int kAdderNoisePort = 1;
  builder.Connect(adder->get_output_port(), simple_car_->get_input_port(0));
  builder.Connect(mux->get_output_port(0),
                  adder->get_input_port(kAdderValuePort));
  // Export the I/O.
  disturbance_input_port_ =
      builder.ExportInput(adder->get_input_port(kAdderNoisePort));
  traffic_input_port_ = builder.ExportInput(idm_controller_->traffic_input());
  lane_input_port_ = builder.ExportInput(pursuit_->lane_input());

  state_output_port_ = builder.ExportOutput(simple_car_->state_output());
  pose_output_port_ = builder.ExportOutput(simple_car_->pose_output());
  velocity_output_port_ = builder.ExportOutput(simple_car_->velocity_output());

  builder.BuildInto(this);
}

void IdmSimpleCar::set_simple_car_params(
    const SimpleCarParams<double>& params, Context<double>* context) const {
  auto& subcontext = GetMutableSubsystemContext(*simple_car_, context);
  subcontext.get_mutable_numeric_parameter(0).SetFromVector(
      params.CopyToVector());
}

void IdmSimpleCar::set_idm_params(const IdmPlannerParameters<double>& params,
                                  Context<double>* context) const {
  auto& subcontext = GetMutableSubsystemContext(*idm_controller_, context);
  subcontext.get_mutable_numeric_parameter(0).SetFromVector(
      params.CopyToVector());
}

void IdmSimpleCar::set_pure_pursuit_params(
    const PurePursuitParams<double>& params, Context<double>* context) const {
  auto& subcontext = GetMutableSubsystemContext(*pursuit_, context);
  subcontext.get_mutable_numeric_parameter(0).SetFromVector(
      params.CopyToVector());
}

const InputPort<double>& IdmSimpleCar::disturbance_input_port()
    const {
  return get_input_port(disturbance_input_port_);
}
const InputPort<double>& IdmSimpleCar::lane_input_port() const {
  return get_input_port(lane_input_port_);
}
const InputPort<double>& IdmSimpleCar::traffic_input_port() const {
  return get_input_port(traffic_input_port_);
}
const OutputPort<double>& IdmSimpleCar::state_output_port() const {
  return get_output_port(state_output_port_);
}
const OutputPort<double>& IdmSimpleCar::pose_output_port() const {
  return get_output_port(pose_output_port_);
}
const OutputPort<double>& IdmSimpleCar::velocity_output_port() const {
  return get_output_port(velocity_output_port_);
}

Scenario::Scenario(std::unique_ptr<const RoadGeometry> road, double car_width,
                   double car_length)
    : road_(std::move(road)),
      geometry_(std::make_unique<CarGeometry>(car_width, car_length)),
      builder_(std::make_unique<DiagramBuilder<double>>()) {}

const System<double>* Scenario::AddSimpleCar(
    const std::string& name, const SimpleCarParams<double>& simple_car_params) {
  DRAKE_DEMAND(scenario_diagram_ == nullptr);

  // Wire up the subsystem to the scenario diagram.
  const auto simple_car = builder_->AddSystem<SimpleCar<double>>();
  simple_car->set_name(name + "_simple_car");

  // Store the parameters.
  contexts_[simple_car] = simple_car->CreateDefaultContext();
  contexts_[simple_car]->get_mutable_numeric_parameter(0).SetFromVector(
      simple_car_params.CopyToVector());

  // Register the subsystem.
  driving_command_ports_[simple_car] = &simple_car->get_input_port(0);
  state_ports_[simple_car] = &simple_car->state_output();
  pose_ports_[simple_car] = &simple_car->pose_output();
  velocity_ports_[simple_car] = &simple_car->velocity_output();
  aliases_.push_back(simple_car);  // ** TODO ** Check that this preserves
                                   //            the Context ordering.
  stateful_aliases_[simple_car] = simple_car;
  return simple_car;
}

const System<double>* Scenario::AddIdmSimpleCar(
    const std::string& name, const LaneDirection& goal_lane_direction,
    const SimpleCarParams<double>& simple_car_params,
    const IdmPlannerParameters<double>& idm_params,
    const PurePursuitParams<double>& pure_pursuit_params) {
  DRAKE_DEMAND(scenario_diagram_ == nullptr);

  // Wire up the subsystem to the scenario diagram.
  const auto idm_car = builder_->template AddSystem<IdmSimpleCar>(name, *road_);

  // Wire up a lane source.
  auto lane_source =
      builder_->template AddSystem<systems::ConstantValueSource<double>>(
          AbstractValue::Make<LaneDirection>(goal_lane_direction));
  builder_->Connect(lane_source->get_output_port(0),
                    idm_car->lane_input_port());

  // Store the parameters.
  contexts_[idm_car] = idm_car->CreateDefaultContext();
  idm_car->set_simple_car_params(simple_car_params, contexts_[idm_car].get());
  idm_car->set_idm_params(idm_params, contexts_[idm_car].get());
  idm_car->set_pure_pursuit_params(pure_pursuit_params,
                                   contexts_[idm_car].get());

  // Register the subsystem.
  driving_command_ports_[idm_car] = &idm_car->disturbance_input_port();
  traffic_ports_[idm_car] = &idm_car->traffic_input_port();
  state_ports_[idm_car] = &idm_car->state_output_port();
  pose_ports_[idm_car] = &idm_car->pose_output_port();
  velocity_ports_[idm_car] = &idm_car->velocity_output_port();
  aliases_.push_back(idm_car);  // ** TODO ** Check that this preserves the
                                //            Context ordering.
  stateful_aliases_[idm_car] = idm_car->get_simple_car();
  return idm_car;
}

void Scenario::Build() {
  const int num_subsystems = aliases_.size();
  const auto demux =
      builder_->template AddSystem<DrivingCommandDemux<double>>(num_subsystems);
  demux->set_name("demux");
  const auto aggregator =
      builder_->template AddSystem<PoseAggregator<double>>();
  aggregator->set_name("pose_aggregator");

  int port_id{0};
  for (const auto alias : aliases_) {
    // Inputs.
    if (driving_command_ports_.find(alias) != driving_command_ports_.end()) {
      DRAKE_DEMAND(driving_command_ports_[alias]->size() ==
                   DrivingCommandIndices::kNumCoordinates);
      builder_->Connect(demux->get_output_port(port_id++),
                        *driving_command_ports_[alias]);
    }
    if (traffic_ports_.find(alias) != traffic_ports_.end()) {
      DRAKE_DEMAND(traffic_ports_[alias]->get_data_type() ==
                   systems::kAbstractValued);
      builder_->Connect(aggregator->get_output_port(0), *traffic_ports_[alias]);
    }

    // Outputs.
    DRAKE_DEMAND(state_ports_[alias]->size() ==
                 SimpleCarStateIndices::kNumCoordinates);
    DRAKE_DEMAND(pose_ports_[alias]->size() == PoseVector<double>::kSize);
    DRAKE_DEMAND(velocity_ports_[alias]->size() ==
                 FrameVelocity<double>::kSize);
    auto ports =
        aggregator->AddSinglePoseAndVelocityInput(alias->get_name(), port_id);
    builder_->Connect(*pose_ports_[alias], ports.pose_input_port);
    builder_->Connect(*velocity_ports_[alias], ports.velocity_input_port);

    // States.
    state_sizes_[alias] =
        contexts_[alias]->get_continuous_state_vector().size();
  }

  // Export the stacked input.
  builder_->ExportInput(demux->get_input_port(0));

  scenario_diagram_ = builder_->Build();
  scenario_diagram_->set_name("scenario");
  scenario_context_ = scenario_diagram_->CreateDefaultContext();
  for (const auto alias : aliases_) {
    auto& subcontext = scenario_diagram_->GetMutableSubsystemContext(
        *alias, scenario_context_.get());
    for (int i{0}; i < subcontext.num_numeric_parameter_groups(); i++) {
      subcontext.get_mutable_numeric_parameter(i).SetFromVector(
          contexts_[alias]->get_numeric_parameter(i).CopyToVector());
    }
  }
}

std::vector<int> Scenario::GetStateIndices(
    const System<double>& subsystem) const {
  DRAKE_DEMAND(aliases_.size() > 0);
  const size_t index =
      std::distance(aliases_.begin(),
                    std::find(aliases_.begin(), aliases_.end(), &subsystem));
  if (index == aliases_.size()) {
    throw std::runtime_error("The provided subsystem was not found.");
  }
  const int state_index = std::accumulate(
      state_sizes_.begin(), std::next(state_sizes_.begin(), index), 0.,
      [](int value,
         const std::map<const System<double>*, int>::value_type& sizes) {
        return value + sizes.second;
      });
  auto result = std::vector<int>(state_sizes_.at(&subsystem));
  std::iota(std::begin(result), std::end(result), state_index);
  return result;
}

std::vector<int> Scenario::GetInputIndices(
    const System<double>& subsystem) const {
  DRAKE_DEMAND(aliases_.size() > 0);
  // Either it's a DrivingCommand port or there is no port.
  if (driving_command_ports_.find(&subsystem) == driving_command_ports_.end()) {
    return {};
  }
  const size_t index =
      std::distance(aliases_.begin(),
                    std::find(aliases_.begin(), aliases_.end(), &subsystem));
  if (index == aliases_.size()) {
    throw std::runtime_error("The provided subsystem was not found.");
  }
  auto result = std::vector<int>(DrivingCommandIndices::kNumCoordinates);
  std::iota(std::begin(result), std::end(result),
            index * DrivingCommandIndices::kNumCoordinates);
  return result;
}

}  // namespace automotive
}  // namespace drake
