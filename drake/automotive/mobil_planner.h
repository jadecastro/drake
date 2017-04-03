#pragma once

#include <memory>
#include <utility>

#include <Eigen/Geometry>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/automotive/gen/mobil_planner_parameters.h"
#include "drake/automotive/idm_planner.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/pose_selector.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// MOBIL (Minimizing Overall Braking Induced by Lane Changes) [1] is a planner
/// that maximizes acceleration incentive for the ego car and (per a weighting
/// factor) the acceleration incentives for any trailing cars in its immediate
/// neighborhood.  The planner selects cars using the PoseSelector logic,
/// outputs the incentive-maximizing lane, as a LaneDirection, that references a
/// valid lane in the provided @p RoadGeometry and direction of travel.
///
/// The planner also creates longitudinal DrivingCommands based on the IDM
/// equation (see IdmPlanner).
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// Input Port 0: @p ego_pose PoseVector for the ego car.
///   (InputPortDescriptor getter: ego_pose_input())
/// Input Port 1: @p traffic_poses PoseBundle for the traffic cars, possibly
///   including the ego car's pose.
///   (InputPortDescriptor getter: poses_bundle_input())
///
/// Output Port 0: A DrivingCommand with the following elements:
///   * steering angle (unused - outputs 0).
///   * acceleration.
///   (OutputPortDescriptor getter: driving_command_output())
/// Output Port 1: A LaneDirection consistent with the provided @p road.
///   (OutputPortDescriptor getter: lane_output())
///
/// @ingroup automotive_systems
///
/// [1] Arne Kesting, Martin Treiber and Dirk Helbing, MOBIL: General
///     Lane-Changing Model for Car-Following Models, Journal of the
///     Transportation Research Board, v1999, 2007, pp 86-94.
///     http://trrjournalonline.trb.org/doi/abs/10.3141/1999-10.
template <typename T>
class MobilPlanner : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilPlanner)

  /// Constructor.
  /// @param is road the pre-defined RoadGeometry.
  explicit MobilPlanner(const maliput::api::RoadGeometry& road,
                        const LaneDirection& initial_lane_direction);
  ~MobilPlanner() override;

  /// See the class description for details on the following input ports.
  /// @{
  const systems::InputPortDescriptor<T>& ego_pose_input() const;
  const systems::InputPortDescriptor<T>& ego_velocity_input() const;
  const systems::InputPortDescriptor<T>& traffic_input() const;
  const systems::OutputPortDescriptor<T>& driving_command_output() const;
  const systems::OutputPortDescriptor<T>& lane_output() const;
  /// @}

 private:
  typedef typename std::pair<const pose_selector::RoadOdometry<T>,
                             const pose_selector::RoadOdometry<T>>
      OdometryPair;

  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void ImplDoCalcLane(const systems::rendering::PoseVector<T>& ego_pose,
                      const systems::rendering::FrameVelocity<T>& ego_velocity,
                      const systems::rendering::PoseBundle<T>& traffic_poses,
                      const IdmPlannerParameters<T>& idm_params,
                      const MobilPlannerParameters<T>& mobil_params,
                      LaneDirection* lane_direction) const;

  void ImplDoCalcAccel(const systems::rendering::PoseVector<T>& ego_pose,
                       const systems::rendering::FrameVelocity<T>& ego_velocity,
                       const systems::rendering::PoseBundle<T>& traffic_poses,
                       const IdmPlannerParameters<T>& idm_params,
                       DrivingCommand<T>* output) const;

  // Computes a incentive measure pair for the provided neighboring lanes.
  const std::pair<T, T> ComputeIncentives(
      const std::pair<const maliput::api::Lane*, const maliput::api::Lane*>
          lanes,
      const IdmPlannerParameters<T>& idm_params,
      const MobilPlannerParameters<T>& mobil_params,
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::FrameVelocity<T>& ego_velocity,
      const systems::rendering::PoseBundle<T>& traffic_poses) const;

  // Computes an incentive measure considering the given closest leading and
  // trailing vehicles to the pre-computed result in the current lane.
  void CompareOutOfLane(const IdmPlannerParameters<T>& idm_params,
                        const MobilPlannerParameters<T>& mobil_params,
                        const OdometryPair& leading_trailing,
                        const pose_selector::RoadOdometry<T>& ego_odometry,
                        const double& ego_old_accel,
                        const double& trailing_delta_accel_this,
                        double* incentive) const;

  // Computes an acceleration based on the IDM equation (via a call to
  // IdmPlanner::Eval()).
  const T EvaluateIdm(
      const IdmPlannerParameters<T>& idm_params,
      const pose_selector::RoadOdometry<T>& ego_odometry,
      const pose_selector::RoadOdometry<T>& lead_car_odometry) const;

  const maliput::api::RoadGeometry& road_;
  bool with_s_{true};

  // Indices for the input / output ports.
  int ego_pose_index_;
  int ego_velocity_index_;
  int traffic_index_;
  int command_index_;
  int lane_index_;
};

}  // namespace automotive
}  // namespace drake
