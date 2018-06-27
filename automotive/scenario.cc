#include "drake/automotive/scenario.h"

#include <memory>
#include <vector>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/pure_pursuit_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"

namespace drake {
namespace automotive {

using maliput::api::RoadGeometry;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::System;
using systems::rendering::PoseAggregator;

// Model parameters.
static constexpr double kPeriodSec = 0.1;  // For IDM periodic update.
static constexpr ScanStrategy kScanStrategy = ScanStrategy::kPath;
static constexpr RoadPositionStrategy kRoadPositionStrategy =
    RoadPositionStrategy::kExhaustiveSearch;

Scenario::Scenario(std::unique_ptr<RoadGeometry> road, double car_width,
                   double car_length)
    : road_(std::move(road)),
      geometry_(std::make_unique<CarGeometry>(car_width, car_length)),
      builder_(std::make_unique<systems::DiagramBuilder<double>>()),
      aggregator_(builder_->template AddSystem<PoseAggregator<double>>()) {
  aggregator_->set_name("pose_aggregator");
}

const System<double>* Scenario::AddSimpleCar(const std::string& name) {
  DRAKE_DEMAND(scenario_diagram_ == nullptr);
  const auto simple_car = builder_->AddSystem<SimpleCar<double>>();
  simple_car->set_name(name);
  aliases_.push_back(simple_car);  // ** TODO ** Check that this preserves
                                   //            the Context ordering.

  // ** TODO ** Register x, y, heading indices/expressions for use by trajopt
  //            (so it doesn't have to assume simplecar). But perhaps cleaner
  //            yet, we should do subsystem.CalcOutput() for the state_port for
  //            a symbolic-transmogrified system (but how to disentangle from
  //            SimpleCar?).

  // Wire up the subsystem to the scenario diagram.
  const int port_id = aliases_.size() - 1;
  auto ports = aggregator_->AddSinglePoseAndVelocityInput(
      simple_car->get_name(), port_id);
  builder_->Connect(simple_car->pose_output(), ports.pose_descriptor);
  builder_->Connect(simple_car->velocity_output(), ports.velocity_descriptor);
  return simple_car;
}

const System<double>* Scenario::AddIdmSimpleCar(
    const std::string& name, const LaneDirection& goal_lane_direction) {
  DRAKE_DEMAND(scenario_diagram_ == nullptr);
  std::unique_ptr<DiagramBuilder<double>> car_builder{
      std::make_unique<DiagramBuilder<double>>()};

  const auto idm_controller =
      car_builder->template AddSystem<IdmController<double>>(
          *road_, kScanStrategy, kRoadPositionStrategy, kPeriodSec);
  idm_controller->set_name(name + "_idm_controller");

  const auto simple_car = car_builder->template AddSystem<SimpleCar<double>>();
  simple_car->set_name(name + "_simple_car");

  const auto pursuit =
      car_builder->template AddSystem<PurePursuitController<double>>();
  pursuit->set_name(name + "_pure_pursuit_controller");
  const auto mux =
      car_builder->template AddSystem<systems::Multiplexer<double>>(
          DrivingCommand<double>());
  // ** TODO ** Rebase and replace with DrivingCommandMux.
  // ** TODO ** (related) Apply SimpleCar coherence fix.
  mux->set_name(name + "_mux");

  // Wire up the simple car to IdmController.
  car_builder->Connect(simple_car->pose_output(),
                       idm_controller->ego_pose_input());
  car_builder->Connect(simple_car->velocity_output(),
                       idm_controller->ego_velocity_input());

  // Wire up the simple car to PurePursuitController.
  car_builder->Connect(simple_car->pose_output(), pursuit->ego_pose_input());
  // Build DrivingCommand via a mux of two scalar outputs.
  car_builder->Connect(
      pursuit->steering_command_output(),
      mux->get_input_port(DrivingCommandIndices::kSteeringAngle));
  car_builder->Connect(
      idm_controller->acceleration_output(),
      mux->get_input_port(DrivingCommandIndices::kAcceleration));

  const int input_size = DrivingCommandIndices::kNumCoordinates;
  const auto adder = car_builder->template AddSystem<systems::Adder<double>>(
      2 /*num inputs*/, input_size);
  const int kAdderValuePort = 0;
  const int kAdderNoisePort = 1;
  car_builder->Connect(adder->get_output_port(), simple_car->get_input_port(0));
  car_builder->Connect(mux->get_output_port(0),
                       adder->get_input_port(kAdderValuePort));

  // Export the i/o.
  const int noise_inport =
      car_builder->ExportInput(adder->get_input_port(kAdderNoisePort));
  DRAKE_DEMAND(noise_inport ==
               0);  // N.B. We rely on this for connecting to demux.
  // ** TODO ** Relax this by storing the port index.
  const int traffic_inport =
      car_builder->ExportInput(idm_controller->traffic_input());
  const int lane_inport = car_builder->ExportInput(pursuit->lane_input());

  const int state_outport = car_builder->ExportOutput(simple_car->pose_output());
  DRAKE_DEMAND(state_outport == simple_car->state_output().get_index());
  // N.B. We rely on this for converting state to SimpleCarState.
  // ** TODO ** Relax this by storing the port index.
  const int pose_outport = car_builder->ExportOutput(simple_car->pose_output());
  const int velocity_outport =
      car_builder->ExportOutput(simple_car->velocity_output());

  std::unique_ptr<Diagram<double>> diagram = car_builder->Build();
  diagram->set_name(name);

  // Wire up the subsystem to the scenario diagram.
  const auto idm_car = builder_->AddSystem(std::move(diagram));
  aliases_.push_back(idm_car);  // ** TODO ** Check that this preserves the
                                //            Context ordering.
  auto lane_source =
      builder_->template AddSystem<systems::ConstantValueSource<double>>(
          systems::AbstractValue::Make<LaneDirection>(goal_lane_direction));

  builder_->Connect(aggregator_->get_output_port(0),
                    idm_car->get_input_port(traffic_inport));

  const int port_id = aliases_.size() - 1;
  auto ports =
      aggregator_->AddSinglePoseAndVelocityInput(idm_car->get_name(), port_id);
  builder_->Connect(idm_car->get_output_port(pose_outport),
                    ports.pose_descriptor);
  builder_->Connect(idm_car->get_output_port(velocity_outport),
                    ports.velocity_descriptor);
  builder_->Connect(lane_source->get_output_port(0),
                    idm_car->get_input_port(lane_inport));
  return idm_car;
}

void Scenario::Build() {
  // ** TODO ** Assert if any of the aliases' inputs aren't the DrivingCommands
  //            we're expecting below.
  const int num_subsystems = aliases_.size();
  const int input_size =
      DrivingCommandIndices::kNumCoordinates * num_subsystems;
  const auto demux =
      builder_->template AddSystem<systems::Demultiplexer<double>>(
          input_size, DrivingCommandIndices::kNumCoordinates);
  demux->set_name("demux");

  // ** TODO ** Ordering-sensitive. Check that this has not accidentally been
  //            mis-wired.
  int port_id{0};
  for (const auto alias : aliases_) {
    builder_->Connect(demux->get_output_port(port_id++),
                      alias->get_input_port(0));
  }
  // Export the stacked input.
  builder_->ExportInput(demux->get_input_port(0));

  scenario_diagram_ = builder_->Build();
  scenario_diagram_->set_name("multi_car_scenario");
  builder_.reset();
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
  auto result = std::vector<int>(SimpleCarStateIndices::kNumCoordinates);
  std::iota(std::begin(result), std::end(result),
            index * SimpleCarStateIndices::kNumCoordinates);
  return result;
}

std::vector<int> Scenario::GetInputIndices(
    const System<double>& subsystem) const {
  DRAKE_DEMAND(aliases_.size() > 0);
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
