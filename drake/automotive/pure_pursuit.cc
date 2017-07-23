#include "drake/automotive/pure_pursuit.h"

#include <cmath>
#include <memory>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_utils.h"
#include "drake/common/extract_double.h"
#include "drake/common/symbolic_formula.h"
#include "drake/math/saturate.h"

namespace drake {
namespace automotive {

using common::autodiff::PadWithZeroPartials;
using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LanePosition;
using systems::rendering::PoseVector;

template <typename T>
T PurePursuit<T>::Evaluate(const PurePursuitParams<T>& pp_params,
                           const SimpleCarParams<T>& car_params,
                           const LaneDirection& lane_direction,
                           const PoseVector<T>& pose) {
  DRAKE_DEMAND(pp_params.IsValid());
  DRAKE_DEMAND(car_params.IsValid());

  using std::atan2;
  using std::cos;
  using std::pow;
  using std::sin;

  const GeoPosition goal =
      ComputeGoalPoint(pp_params.s_lookahead(), lane_direction, pose);

  const T x = pose.get_translation().translation().x();
  const T y = pose.get_translation().translation().y();
  const T heading = pose.get_rotation().z();

  const T delta_r =
      -(T(goal.x()) - x) * sin(heading) + (T(goal.y()) - y) * cos(heading);
  const T wheelbase_inv = T(1.) / T(car_params.wheelbase());
  const T curvature = PadWithZeroPartials(
      2. * delta_r / pow(T(pp_params.s_lookahead()), 2.), wheelbase_inv);

  // Return the steering angle.
  return atan2(curvature, wheelbase_inv);
}

template <typename T>
GeoPosition PurePursuit<T>::ComputeGoalPoint(
    const T& s_lookahead, const LaneDirection& lane_direction,
    const PoseVector<T>& pose) {
  const Lane* const lane = lane_direction.lane;
  const bool with_s = lane_direction.with_s;
  const LanePosition position = lane->ToLanePosition({
      ExtractDoubleOrThrow(pose.get_isometry().translation().x()),
      ExtractDoubleOrThrow(pose.get_isometry().translation().y()),
      ExtractDoubleOrThrow(pose.get_isometry().translation().z())},
    nullptr, nullptr);
  const double s_new = ExtractDoubleOrThrow(
      cond(with_s, position.s() + s_lookahead, position.s() - s_lookahead));
  const double s_goal =
      ExtractDoubleOrThrow(math::saturate(s_new, T(0.), T(lane->length())));
  // TODO(jadecastro): Add support for locating goal points in ongoing lanes.
  return lane->ToGeoPosition({s_goal, 0., position.h()});
  // TODO(jadecastro) Use an autodiff version of `Lane::ToLanePosition()`and
  // `Lane::ToGeoPosition()` once #6614 lands.
}

// These instantiations must match the API documentation in pure_pursuit.h.
// The only scalar type supported is double.
template class PurePursuit<double>;
template class PurePursuit<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
