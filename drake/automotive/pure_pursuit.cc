#include "drake/automotive/pure_pursuit.h"

#include <cmath>
#include <memory>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::PoseVector;

template <typename T>
T PurePursuit<T>::Evaluate(const PurePursuitParams<T>& pp_params,
                           const SimpleCarParams<T>& car_params,
                           bool with_s, const RoadGeometry& road,
                           const PoseVector<T>& pose) {
  DRAKE_DEMAND(pp_params.IsValid());
  DRAKE_DEMAND(car_params.IsValid());

  using std::atan;
  using std::cos;
  using std::pow;
  using std::sin;

  const GeoPosition goal_position = ComputeGoalPoint(
      pp_params.s_lookahead(), with_s, road, pose);

  const T x = pose.get_translation().translation().x();
  const T y = pose.get_translation().translation().y();
  const T heading = pose.get_rotation().z();

  const T delta_r = -(goal_position.x - x) * sin(heading) +
                    (goal_position.y - y) * cos(heading);
  const T curvature = 2 * delta_r / pow(pp_params.s_lookahead(), 2.);

  // Return the steering angle.
  return atan(car_params.wheelbase() * curvature);
}

template <typename T>
const GeoPosition PurePursuit<T>::ComputeGoalPoint(
    const T& s_lookahead, bool with_s, const RoadGeometry& road,
    const PoseVector<T>& pose) {
  const RoadPosition position =
      pose_selector::CalcRoadPosition(road, pose.get_isometry());

  // Loop through all default branches, terminating once a lane containing the
  // goal point is found.
  // TODO(jadecastro): Relax the need to have default branches specified for
  // every lane.
  double s_new = (with_s) ? (position.pos.s + s_lookahead)
      : (position.pos.s - s_lookahead);
  std::unique_ptr<LaneEnd> branch;
  const Lane* lane{position.lane};
  while (s_new < 0 || s_new > lane->length()) {
    if (with_s) {
      branch = lane->GetDefaultBranch(LaneEnd::kFinish);
    } else {
      branch = lane->GetDefaultBranch(LaneEnd::kStart);
    }
    if (branch == nullptr) {
      DRAKE_ABORT_MSG("PurePursuit::ComputeGoalPoint: "
                      "No default branch exists.");
    }
    lane = branch->lane;
    const double s_overrun = (with_s) ? s_new - lane->length()
        : s_new - lane->length();
    if (branch->end == LaneEnd::kStart) {
      s_new = s_overrun;
    } else {
      s_new = lane->length() - s_overrun;
      with_s = false;
    }
  }

  return lane->ToGeoPosition(LanePosition(s_new, 0., 0.));
}

// These instantiations must match the API documentation in pure_pursuit.h.
// The only scalar type supported is double.
template class PurePursuit<double>;

}  // namespace automotive
}  // namespace drake
