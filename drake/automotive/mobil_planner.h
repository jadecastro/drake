#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/automotive/gen/mobil_planner_parameters.h"
#include "drake/automotive/idm_planner.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// MobilPlanner -- MOBIL (Minimizing Overall Braking Induced by Lane Changes)
/// is a planner .
///
/// IDM: Intelligent Driver Model:
///    https://en.wikipedia.org/wiki/Intelligent_driver_model
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
///
/// Inputs:
///   Port 0:
///      @p pose_ego PoseBundle for the ego car.
///   Port 1:
///      @p pose_agent PoseBundle for the traffic cars.
/// Outputs:
///   Port 0:
///      @p vdot_ego linear acceleration of the ego car (scalar) [m/s^2].
template <typename T>
class MobilPlanner : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilPlanner)

  /// @p v_ref desired velocity of the ego car in units of m/s.
  MobilPlanner(const maliput::api::RoadGeometry* road);
  ~MobilPlanner() override;

  /// Returns the port to the ego car input subvector.
  const systems::InputPortDescriptor<T>& ego_pose_input() const;
  const systems::InputPortDescriptor<T>& agent_pose_bundle_input() const;

  // System<T> overrides.
  // The output of this system is an algebraic relation of its inputs.
  bool has_any_direct_feedthrough() const override { return true; }

  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::OutputPortDescriptor<T>& descriptor) const override;

  std::unique_ptr<systems::Parameters<T>> AllocateParameters() const override;

  void SetDefaultParameters(const systems::LeafContext<T>& context,
                            systems::Parameters<T>* params) const override;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void ImplDoCalcReference(
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::PoseBundle<T>& agent_poses,
      const MobilPlannerParameters<T>& params,
      systems::BasicVector<T>* reference) const;

  void ImplDoCalcOutput(const systems::rendering::PoseVector<T>& ego_pose,
                        const systems::rendering::PoseBundle<T>& agent_poses,
                        const MobilPlannerParameters<T>& params,
                        DrivingCommand<T>* output) const;

  const std::pair<T, T> ComputeIncentives(
    const std::pair<const maliput::api::Lane*, const maliput::api::Lane*> lanes,
    const MobilPlannerParameters<T>& params,
    const systems::rendering::PoseVector<T>& ego_pose,
    const systems::rendering::PoseBundle<T>& agent_poses) const;

  const T EvaluateIdm(
    const MobilPlannerParameters<T>& params,
    const maliput::api::RoadPosition& ego_position,
    const maliput::api::RoadPosition& agent_position) const;

  const maliput::api::GeoPosition ComputeGoalPoint(
      const MobilPlannerParameters<T>& params, const maliput::api::Lane* lane,
      const maliput::api::RoadPosition& position) const;

  const maliput::api::RoadGeometry* road_;
};

}  // namespace automotive
}  // namespace drake
