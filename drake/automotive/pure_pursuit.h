#pragma once

#include "drake/automotive/gen/pure_pursuit_params.h"
#include "drake/automotive/gen/simple_car_params.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/pose_selector.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// PurePursuit computes the required steering angle to achieve a goal point on
/// an continuous planar curve.  The curve represents as the set of `r = 0`
/// positions along a Maliput lane.
///
/// See the corresponding .cc file for details.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
template <typename T>
class PurePursuit {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PurePursuit)
  PurePursuit() = delete;

  /// Evaluates the required steering angle in radians using the pure-pursuit
  /// method.  N.B. Assumes zero elevation and superelevation.
  static const T Evaluate(const PurePursuitParams<T>& pp_params,
                          const SimpleCarParams<T>& car_params,
                          const LaneDirection& lane_direction,
                          const maliput::api::RoadGeometry& road,
                          const systems::rendering::PoseVector<T>& ego_pose);

  /// Computes the goal point at a distance `s_lookahead` from the closest
  /// position on the curve in the intended direction of travel.
  static const maliput::api::GeoPosition ComputeGoalPoint(
      const T& s_lookahead, const LaneDirection& lane_direction,
      const maliput::api::RoadPosition& position);
};

}  // namespace automotive
}  // namespace drake
