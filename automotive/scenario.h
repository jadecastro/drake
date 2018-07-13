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

class IdmSimpleCar : public systems::Diagram<double> {
 public:
  ///
  IdmSimpleCar(std::string name, const maliput::api::RoadGeometry& road);

  // TODO(jadecastro) Make IdmSimpleCar derive from an abstract class with these
  // as pure vituals.
  const systems::InputPortDescriptor<double>& disturbance_input_port() const;
  const systems::InputPortDescriptor<double>& lane_input_port() const;
  const systems::InputPortDescriptor<double>& traffic_input_port() const;
  const systems::OutputPort<double>& state_output_port() const;
  const systems::OutputPort<double>& pose_output_port() const;
  const systems::OutputPort<double>& velocity_output_port() const;

  /// TODO(jadecastro) Temporary getter until we've resolved symbolic support
  /// for this diagram.
  const SimpleCar<double>* get_simple_car() const { return simple_car_; }

 private:
  SimpleCar<double>* simple_car_{nullptr};

  int disturbance_input_port_{};
  int lane_input_port_{};
  int traffic_input_port_{};
  int state_output_port_{};
  int pose_output_port_{};
  int velocity_output_port_{};
};

/// Builds a system of cars with a common underlying RoadGeometry and a set of
/// car subsystems (Diagrams or LeafSystems).
///
/// All subsystems added via AddFoo() methods are assumed to have three output
/// ports, with the following ordering:
///
/// port 0: SimpleCarState
/// port 1: PoseVector
/// port 2: FrameVelocity
class Scenario final {
 public:
  using InputPortMap = std::map<const systems::System<double>*,
                                const systems::InputPortDescriptor<double>*>;
  using OutputPortMap = std::map<const systems::System<double>*,
                                 const systems::OutputPort<double>*>;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Scenario)

  Scenario(std::unique_ptr<maliput::api::RoadGeometry> road, double car_width,
           double car_length);

  static constexpr int kSimpleCarStatePort = 0;
  static constexpr int kPoseVectorPort = 1;
  static constexpr int kFrameVelocityPort = 2;
  // ** TODO ** Unit test to verify this for our AddFoo() subsystems.

  struct CarGeometry {
    CarGeometry() = delete;
    CarGeometry(double w, double l) : width(w), length(l) {}

    const double width{};
    const double length{};
  };

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

  /// Accessor to the subsystems of the scenario Diagram.
  std::vector<const systems::System<double>*> aliases() const {
    return aliases_;
  }

  /// *Temporary* accessor to the stateful sub-subsystems of the scenario
  /// Diagram.
  // TODO(jadecastro) Remove me once the subsystem diagrams are
  // symbolic-supported.
  std::map<const systems::System<double>*, const systems::System<double>*>
  stateful_aliases() const {
    return stateful_aliases_;
  }

  /// @name Road and geometry accessors.
  /// @{
  const maliput::api::RoadGeometry& road() const { return *road_; }
  std::unique_ptr<maliput::api::RoadGeometry> get_road() {
    return std::move(road_);
  }
  double car_width() const { return geometry_->width; }
  double car_length() const { return geometry_->length; }
  /// @}

 private:
  std::unique_ptr<maliput::api::RoadGeometry> road_;
  std::unique_ptr<CarGeometry> geometry_;

  std::unique_ptr<systems::DiagramBuilder<double>> builder_;

  InputPortMap driving_command_ports_;
  InputPortMap traffic_ports_;
  OutputPortMap state_ports_;
  OutputPortMap pose_ports_;
  OutputPortMap velocity_ports_;
  std::map<const systems::System<double>*, int> state_sizes_;

  std::vector<const systems::System<double>*> aliases_{};

  // Maps a subsystem to the corresponding sub-subsystem that has states.
  std::map<const systems::System<double>*, const systems::System<double>*>
      stateful_aliases_{};  // TODO(jadecastro) Remove me once subsystems are
                            // all symbolic-supported.

  std::unique_ptr<systems::Diagram<double>> scenario_diagram_;
};

}  // namespace automotive
}  // namespace drake
