#pragma once

#include <cmath>
#include <memory>
#include <vector>

#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/simple_car.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace automotive {

class Scenario final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Scenario)

  Scenario(int num_lanes, double lane_width, double road_length,
           double car_width, double car_length);

  // ** TODO ** These can only be called *before* Build().
  // ** TODO ** Put these into a subclass?
  // @{
  const systems::System<double>* AddIdmSimpleCar(const std::string& name);

  const systems::System<double>* AddSimpleCar(const std::string& name);

  void FixGoalLaneDirection(const systems::System<double>& subsystem,
                            const LaneDirection& lane_direction);
  // @}

  void Build();

  /// @name Road and geometry accessors.
  /// @{
  const maliput::api::RoadGeometry& road() const { return *road_; }
  double car_width() const { return car_width_; }
  double car_length() const { return car_length_; }
  /// @}

  // ** TODO ** The remainder of these public methods can only be called *after* Build().
  void SetInitialSubsystemStateBounds(
      const systems::System<double>& subsystem,
      const SimpleCarState<double>& lb_value,
      const SimpleCarState<double>& ub_value);

  void SetFinalSubsystemState(const systems::System<double>& subsystem,
                              const SimpleCarState<double>& value);

  /// Returns a vector of indices corresponding to a particular `subsystem` in
  /// the scenario.
  std::vector<int> GetStateIndices(
      const systems::System<double>& subsystem) const;

  /// @name System accessors.
  /// @{
  const systems::Diagram<double>& diagram() const { return *scenario_diagram_; }
  const systems::Context<double>& initial_context_lb() const {
    return *initial_context_lb_;
  }
  const systems::Context<double>& initial_context_ub() const {
    return *initial_context_ub_;
  }
  const systems::Context<double>& final_context() const {
    return *final_context_;
  }
  std::vector<const systems::System<double>*> aliases() const {
    return aliases_;
  }
  const systems::System<double>* ego_alias() const { return ego_alias_; }
  /// @}

 private:
  std::unique_ptr<maliput::api::RoadGeometry> road_;
  const double car_width_{0.};
  const double car_length_{0.};

  // Temporaries that move to DiagramBuilder once BuildScenario is called.
  std::vector<std::unique_ptr<systems::Diagram<double>>> ado_cars_{};
  std::unique_ptr<SimpleCar<double>> ego_car_;

  const systems::System<double>* ego_alias_{nullptr};
  std::vector<const systems::System<double>*>
      aliases_{};  // Double check if even needed!

  std::map<const systems::System<double>*, LaneDirection> goal_lane_map_;

  // Indices used for accessing the decision variables.
  std::vector<int> ego_indices_{};
  std::vector<std::vector<int>> ado_indices_{};

  std::unique_ptr<systems::Diagram<double>> scenario_diagram_;
  std::unique_ptr<systems::Context<double>> initial_context_lb_;
  std::unique_ptr<systems::Context<double>> initial_context_ub_;
  std::unique_ptr<systems::Context<double>> final_context_;

  int noise_inport_{};
  int traffic_inport_{};
  int lane_inport_{};
  int pose_outport_{};
  int velocity_outport_{};
};

}  // namespace automotive
}  // namespace drake
