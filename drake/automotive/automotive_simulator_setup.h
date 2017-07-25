#pragma once

#include <memory>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace automotive {

template <typename T>
struct TrajectoryCarStruct {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TrajectoryCarStruct);

  TrajectoryCarStruct() = default;

  TrajectoryCarState<T>* states = nullptr;
  int lane_index;
};

namespace sim_setup {

// TODO(jadecastro): Needs a more specialized name.
template <typename T>
std::unique_ptr<AutomotiveSimulator<T>> SetupAutomotiveSimulator(
    std::unique_ptr<const maliput::api::RoadGeometry> road_geometry,
    int num_dragway_lanes, int ego_desired_lane_index,
    const std::vector<int> traffic_lane_indices) {
  // Construct without LCM enabled as it is not yet AutoDiff-supported.
  auto simulator = std::make_unique<AutomotiveSimulator<T>>();
  auto simulator_road = simulator->SetRoadGeometry(std::move(road_geometry));
  auto dragway_road_geometry =
      dynamic_cast<const maliput::dragway::RoadGeometry*>(simulator_road);
  DRAKE_DEMAND(dragway_road_geometry != nullptr);

  // Add one ego car (simple car) whose steering is governed by a controller
  // that moves it to a desired lane.
  // TODO(jadecastro): Replace with MobilControlledSimpleCar once trajectory
  // optimization can be formulated as a MI problem.
  const auto segment = dragway_road_geometry->junction(0)->segment(0);
  DRAKE_DEMAND(ego_desired_lane_index >= 0 &&
               ego_desired_lane_index < segment->num_lanes());
  const maliput::api::Lane* ego_desired_lane =
      segment->lane(ego_desired_lane_index);
  const auto& params_ego = CreateTrajectoryParamsForDragway(
      *dragway_road_geometry, ego_desired_lane_index, 0., 0.);
  simulator->AddIdmControlledCar("ego_car", std::get<0>(params_ego), 0.,
                                 0., ego_desired_lane);

  // Add traffic cars (trajectory cars).
  // TODO(jadecastro): Make these IDM-controlled.
  int i{0};
  for (int traffic_lane_index : traffic_lane_indices) {
    DRAKE_DEMAND(traffic_lane_index >= 0 &&
                 traffic_lane_index < num_dragway_lanes);
    const auto& params_traffic = CreateTrajectoryParamsForDragway(
        *dragway_road_geometry, traffic_lane_index, 0., 0.);
    simulator->AddPriusTrajectoryCar(std::to_string(i++),
                                     std::get<0>(params_traffic), 0., 0.);
  }

  simulator->BuildAndInitialize();
  return std::move(simulator);
}

template <typename T>
std::unique_ptr<systems::Simulator<T>> SetupSimulator(
    const systems::System<T>& plant,
    std::unique_ptr<systems::Context<T>> context, double simulator_time_step) {
  // Construct a simulator and intialize it.
  std::unique_ptr<systems::Simulator<T>> simulator =
      std::make_unique<systems::Simulator<T>>(plant);
  simulator->reset_context(std::move(context));
  simulator->set_target_realtime_rate(100.);
  simulator->get_mutable_integrator()->set_maximum_step_size(
      simulator_time_step);
  simulator->get_mutable_integrator()->set_requested_minimum_step_size(
      simulator_time_step);
  simulator->Initialize();

  return std::move(simulator);
}

template <typename T>
std::pair<SimpleCarState<T>*, std::vector<TrajectoryCarStruct<T>>>
    GetAutomotiveSubsystemStates(const systems::System<T>& plant,
                                 std::vector<int> traffic_lane_indices,
                                 systems::Context<T>* context) {
  const auto diagram = dynamic_cast<const systems::Diagram<T>*>(&plant);
  DRAKE_DEMAND(diagram != nullptr);
  auto diagram_context = dynamic_cast<systems::DiagramContext<T>*>(context);
  DRAKE_DEMAND(diagram_context != nullptr);

  // Define the initial context values.  Parse the cars, one by one.
  // Note: context is dual purpose: we write these, and then over-write them
  // with a vector containing the endowed with the desired partial derivatives.
  std::vector<TrajectoryCarStruct<T>> traffic_structs{};
  SimpleCarState<T>* ego_state = nullptr;
  std::vector<const systems::System<T>*> systems = diagram->GetSystems();
  for (int i{0}; i < (int)systems.size(); ++i) {
    systems::VectorBase<T>* state =
        diagram_context->GetMutableSubsystemContext(i)
            ->get_mutable_continuous_state_vector();
    if (state->size() == 0) continue;  // Skip any stateless systems.
    auto traffic_state = dynamic_cast<TrajectoryCarState<T>*>(state);
    ego_state = dynamic_cast<SimpleCarState<T>*>(state);
    if (traffic_state != nullptr) {
      TrajectoryCarStruct<T> traffic_struct{};
      traffic_struct.states = traffic_state;
      traffic_struct.lane_index = traffic_lane_indices[i];
      // TODO(jadecastro) Correspondence to vector indices is fragile - this
      // yearns for a more principled approach.
      traffic_structs.emplace_back(traffic_struct);
    } else if (ego_state != nullptr) {
      continue;
    } else {
      throw std::runtime_error(
          "System " + std::to_string(i) +
          " is stateful but does not match any automotive state vectors.");
    }
  }
  return std::make_pair(ego_state, traffic_structs);
}

}  // namespace sim_setup
}  // namespace automotive
}  // namespace drake
