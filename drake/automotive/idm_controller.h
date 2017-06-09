#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/automotive/idm_planner.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/pose_selector.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// IdmController implements the IDM (Intelligent Driver Model) planner,
/// computed based only on the nearest car ahead.  See IdmPlanner and
/// PoseSelector for details.  The output of this block is an acceleration value
/// passed as a command to the vehicle.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// Input Port 0: PoseVector for the ego car.
///   (InputPortDescriptor getter: ego_pose_input())
///
/// Input Port 1: FrameVelocity of the ego car.
///   (InputPortDescriptor getter: ego_velocity_input())
///
/// Input Port 2: PoseBundle for the traffic cars, possibly inclusive of the ego
///   car's pose.
///   (InputPortDescriptor getter: traffic_input())
///
/// Output Port 0: A BasicVector containing the acceleration request.
///   OutputPortDescriptor getter: acceleration_output())
///
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_controllers
template <typename T>
class IdmController : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IdmController)

  /// Constructor.
  /// @param road is the pre-defined RoadGeometry.
  explicit IdmController(const maliput::api::RoadGeometry& road);
  ~IdmController() override;

  /// See the class description for details on the following input ports.
  /// @{
  const systems::InputPortDescriptor<T>& ego_pose_input() const;
  const systems::InputPortDescriptor<T>& ego_velocity_input() const;
  const systems::InputPortDescriptor<T>& traffic_input() const;
  const systems::OutputPortDescriptor<T>& acceleration_output() const;
  /// @}

 protected:
  const maliput::api::RoadGeometry& road() const { return road_; }
  int ego_pose_index() const { return ego_pose_index_; }
  int ego_velocity_index() const { return ego_velocity_index_; }
  int traffic_index() const { return traffic_index_; }
  int acceleration_index() const { return acceleration_index_; }

  void ImplDoCalcOutput(
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::FrameVelocity<T>& ego_velocity,
      const systems::rendering::PoseBundle<T>& traffic_poses,
      const IdmPlannerParameters<T>& idm_params,
      systems::BasicVector<T>* command) const;

 private:
  template <typename T1 = T>
  void ComputePartials(
      const systems::rendering::PoseVector<
          std::enable_if_t<std::is_same<T1, double>::value, T1>>& ego_pose,
      const systems::rendering::FrameVelocity<
          std::enable_if_t<std::is_same<T1, double>::value, T1>>& ego_velocity,
      const RoadOdometry<std::enable_if_t<std::is_same<T1, double>::value, T1>>&
          lead_car_odom,
      const systems::rendering::PoseBundle<
          std::enable_if_t<std::is_same<T1, double>::value, T1>>& traffic_poses,
      const IdmPlannerParameters<std::enable_if_t<
          std::is_same<T1, double>::value, T1>>& params,
      double* s_dot_ego, double* s_dot_lead, double* headway_distance) const;

  template <typename T1 = T>
  void ComputePartials(
      const systems::rendering::PoseVector<
          std::enable_if_t<std::is_same<T1, AutoDiffXd>::value, T1>>& ego_pose,
      const systems::rendering::FrameVelocity<std::enable_if_t<
          std::is_same<T1, AutoDiffXd>::value, T1>>& ego_velocity,
      const RoadOdometry<std::enable_if_t<std::is_same<T1, AutoDiffXd>::value,
                                          T1>>& lead_car_odom,
      const systems::rendering::PoseBundle<std::enable_if_t<
          std::is_same<T1, AutoDiffXd>::value, T1>>& traffic_poses,
      const IdmPlannerParameters<std::enable_if_t<
          std::is_same<T1, AutoDiffXd>::value, T1>>& params,
      AutoDiffXd* s_dot_ego, AutoDiffXd* s_dot_lead,
      AutoDiffXd* headway_distance) const;

  // Converts @p pose into RoadPosition.
  const maliput::api::RoadPosition GetRoadPosition(
      const Isometry3<T>& pose) const;

  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  IdmController<AutoDiffXd>* DoToAutoDiffXd() const override;

  const maliput::api::RoadGeometry& road_;

  // Indices for the input / output ports.
  const int ego_pose_index_{};
  const int ego_velocity_index_{};
  const int traffic_index_{};
  const int acceleration_index_{};
};

}  // namespace automotive
}  // namespace drake
