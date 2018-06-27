#pragma once

#include <cmath>
#include <memory>
#include <vector>

#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/simple_car.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/pose_aggregator.h"

namespace drake {
namespace automotive {

/// Builds a scenario based on a RoadGeometry and a set of car subsystems
/// (Diagrams or LeafSystems). We require that all subsystems have
/// DrivingCommand inputs and SimpleCarState, and the required ports to connect
/// to PoseAggregator.
class Scenario final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Scenario)

  struct CarGeometry {
    CarGeometry() = delete;
    CarGeometry(double w, double l) : width(w), length(l) {}

    const double width{};
    const double length{};
  };

  Scenario(std::unique_ptr<maliput::api::RoadGeometry> road, double car_width,
           double car_length);

  /// Add a new open-loop SimpleCar system to the scenario.  Returns a reference
  /// to the (stateful) SimpleCar model.
  const systems::System<double>* AddSimpleCar(const std::string& name);

  /// Add a new SimpleCar controlled by a (stateless) IDM longitudinal
  /// controller and a (stateless) pure-pursuit lateral controller to the
  /// scenario.  The goal for the pure-pursuit controller is the center-line of
  /// the lane and its direction, as specified in `goal_lane_direction`.  The
  /// input is an addive DrivingCommand value to the IDM/pure-pursuit commands
  /// sent to the SimpleCar.  Returns a reference to the (stateful) SimpleCar
  /// model.
  //
  // TODO(jadecastro) Default to current lane if goal_lane_direction is
  // unspecified.
  const systems::System<double>* AddIdmSimpleCar(
      const std::string& name, const LaneDirection& goal_lane_direction);

  void Build();

  /// Returns a vector of indices corresponding to the states for a `subsystem`
  /// in the scenario.
  std::vector<int> GetStateIndices(
      const systems::System<double>& subsystem) const;

  /// Returns a vector of indices corresponding to the inputs for a `subsystem`
  /// in the scenario.
  std::vector<int> GetInputIndices(
      const systems::System<double>& subsystem) const;

  /// Accessor to the finalized scenario Diagram.
  const systems::Diagram<double>& diagram() const { return *scenario_diagram_; }

  /// Accessor to the stateful subsystems of the scenario Diagram.
  std::vector<const systems::System<double>*> aliases() const {
    return aliases_;
  }

  /// @name Road and geometry accessors.
  /// @{
  const maliput::api::RoadGeometry& road() const { return *road_; }
  double car_width() const { return geometry_->width; }
  double car_length() const { return geometry_->length; }
  /// @}

 private:
  std::unique_ptr<maliput::api::RoadGeometry> road_;
  std::unique_ptr<CarGeometry> geometry_;

  std::unique_ptr<systems::DiagramBuilder<double>> builder_;
  systems::rendering::PoseAggregator<double>* aggregator_{nullptr};

  std::vector<const systems::System<double>*>
      aliases_{};  // ** TODO ** Store as map with a BasicVector size?
  std::unique_ptr<systems::Diagram<double>> scenario_diagram_;
};

}  // namespace automotive
}  // namespace drake
