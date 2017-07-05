#include "drake/automotive/pure_pursuit.h"

#include <cmath>
#include <memory>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/pose_selector.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_formula.h"
#include "drake/math/saturate.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::LanePositionWithAutoDiff;
using systems::rendering::PoseVector;

static void SetZeroPartials(const AutoDiffXd& model_value, AutoDiffXd* x) {
  std::cout << "  x partials " << x->derivatives() << std::endl;
  const int num_partials = model_value.derivatives().size();
  auto& derivs = (*x).derivatives();
  if (derivs.size() == 0) {
    derivs.resize(num_partials);
    derivs.setZero();
  }
}

static void SetZeroPartials(const double& model_value, double* x) {}

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

  T goal_x, goal_y;
  std::tie(goal_x, goal_y) =
      ComputeGoalPoint(pp_params.s_lookahead(), lane_direction, pose);

  const T x = pose.get_translation().translation().x();
  const T y = pose.get_translation().translation().y();
  const T heading = pose.get_rotation().z();

  const T delta_r = -(goal_x - x) * sin(heading) + (goal_y - y) * cos(heading);
  const T curvature = T(2.) * delta_r / T(pow(pp_params.s_lookahead(), 2.));
  T wheelbase_inverse = T(1.) / T(car_params.wheelbase());
  SetZeroPartials(curvature, &wheelbase_inverse);

  // Return the steering angle.
  return atan2(curvature, wheelbase_inverse);
}

template <typename T>
const std::pair<T, T> PurePursuit<T>::ComputeGoalPoint(
    const T& s_lookahead, const LaneDirection& lane_direction,
    const PoseVector<T>& pose) {
  const Lane* const lane = lane_direction.lane;
  const bool with_s = lane_direction.with_s;
  const LanePositionWithAutoDiff<T> position =
      PoseSelector<T>::CalcLanePosition(lane, pose.get_isometry());
  const T s_new =
      cond(with_s, position.s() + T(s_lookahead),
           position.s() - T(s_lookahead));
  const T s_goal = math::saturate(s_new, T(0.), T(lane->length()));
  // TODO(jadecastro): Add support for locating goal points in ongoing lanes.
  return std::make_pair(s_goal, T(0.));
}

// These instantiations must match the API documentation in pure_pursuit.h.
template class PurePursuit<double>;
template class PurePursuit<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
