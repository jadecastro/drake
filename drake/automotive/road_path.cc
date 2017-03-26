#include "drake/automotive/road_path.h"

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/polynomial.h"
#include "drake/common/cond.h"
#include "drake/common/trajectories/qp_spline/spline_generation.h"

namespace drake {
namespace automotive {

using maliput::api::BranchPoint;
using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LaneEndSet;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;

template <typename T>
RoadPath<T>::RoadPath(const RoadGeometry& road,
                      const LaneDirection& initial_lane_dir,
                      const double step_size,
                      int num_segments)
    : path_(MakePiecewisePolynomial(
          initial_lane_dir, step_size, num_segments)) {}

template <typename T>
RoadPath<T>::~RoadPath() {}

template <typename T>
const std::vector<PiecewisePolynomial<T>>&
RoadPath<T>::get_path() const { return path_; }

/*
template <typename T>
const Eigen::Vector3d RoadPath::GetPathPosition(
    const Eigen::Vector3d& global_pos) const {
  // Use LaneDirection and monolane::{LineLane, ArcLane}::ToLanePosition()?
}
*/

template <typename T>
std::vector<PiecewisePolynomial<T>> RoadPath<T>::MakePiecewisePolynomial(
    const LaneDirection& initial_lane_dir, const double step_size,
    int num_segments) {
  lane_dirs_.push_back(initial_lane_dir);  // Need?
  const Lane* lane = initial_lane_dir.lane;
  bool with_s = initial_lane_dir.with_s;
  T lane_s{cond(with_s, T(0.), T(lane->length()))};
  T monotonic_s{0.};

  GeoPosition geo_pos =
      lane->ToGeoPosition(LanePosition(lane_s, 0. /* r */, 0. /* h */));
  std::vector<T> x0{geo_pos.x, geo_pos.y, geo_pos.z};
  std::vector<Eigen::VectorXd> x_interior;

  std::vector<T> s_breaks{};
  for (Eigen::Index i = 0; i < num_segments; ++i) {
    if (with_s) {
      lane_s += T(step_size);
    } else {
      lane_s += T(step_size);
    }
    monotonic_s += T(step_size);

    // Break if the end of the road has been reached.
    const LaneEndSet* lane_end_set{with_s
          ? lane->GetOngoingBranches(LaneEnd::kFinish)
          : lane->GetOngoingBranches(LaneEnd::kStart)};
    const T out_distance{cond(with_s, T(lane_s - lane->length()), T(-lane_s))};
    if (out_distance >= 0.) {
      if (lane_end_set->size() == 0) break;  // There are no more ongoing lanes.

      // Find a new lane; add it into the stack.  Here we choose only the first.
      lane = lane_end_set->get(0).lane;
      with_s = (lane_end_set->get(0).end == LaneEnd::kStart) ? true : false;
      lane_dirs_.emplace_back(LaneDirection(lane, with_s));  // Need?

      // Adjust the new lane position based on the distance overshoot and the
      // new lane's direction.
      lane_s = cond(with_s, out_distance, T(lane->length()) - out_distance);
    }
    geo_pos = lane->ToGeoPosition(LanePosition(lane_s, 0. /* r */, 0. /* h */));
    x_interior[0](i) = geo_pos.x;
    x_interior[1](i) = geo_pos.y;
    x_interior[2](i) = geo_pos.z;
  }
  // Take the lane end of the final lane as the end of the path.
  T path_end{};
  if (with_s) {
    path_end = (lane_s == lane->length()) ? lane_s + 1e-2 : lane->length();
  } else {
    path_end = (lane_s != 0.) ? -1e-2 : 0.;
  }
  geo_pos = lane->ToGeoPosition(LanePosition(path_end, 0. /* r */, 0. /* h */));
  std::vector<T> xf{geo_pos.x, geo_pos.y, geo_pos.z};

  // Create the resulting piecewise polynomial.
  std::vector<PiecewisePolynomial<T>> result{};
  for (int index = 0; index < 3; ++index) {
    result.emplace_back(PiecewisePolynomial<T>(nWaypointCubicSpline(
        s_breaks, x0[index], 0. /* x0_dot */, xf[index], 0. /* xf_dot */,
        x_interior[index])));
  }
  return result;
}

template class RoadPath<double>;

}  // namespace automotive
}  // namespace drake
