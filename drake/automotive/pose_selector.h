#pragma once

#include <utility>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// PoseSelector -- Routines for selecting a pose of interest among a
/// PoseBundle.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::TaylorVarXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
template <typename T>
class PoseSelector {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseSelector)
  PoseSelector() = delete;

  /// Compares the Lane-space poses
  /// 
  /// If @p agent_lane is `nullptr`, the the ego car's current lane is used.
  ///
  /// N.B. Assumes that the two lanes are side-by-side and hence a comparison
  /// between the s-positions of the two cars is meaningful.
  static const std::pair<maliput::api::RoadPosition, maliput::api::RoadPosition>
  SelectClosestPositions(const maliput::api::RoadGeometry& road,
                         const maliput::api::Lane* agent_lane,
                         const systems::rendering::PoseVector<T>& ego_pose,
                         const systems::rendering::PoseBundle<T>& agent_poses);

  ///
  static const maliput::api::RoadPosition SelectClosestPositionAhead(
      const maliput::api::RoadGeometry& road,
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::PoseBundle<T>& agent_poses);

  ///
  static const maliput::api::RoadPosition GetRoadPosition(
      const maliput::api::RoadGeometry& road, const Isometry3<T>& pose);
};

}  // namespace automotive
}  // namespace drake
