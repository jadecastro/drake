#include "drake/automotive/road_path.h"

#include <iostream>

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
                      int num_segments) {
  MakePiecewisePolynomial(
      initial_lane_dir, step_size, num_segments);
}

template <typename T>
RoadPath<T>::~RoadPath() {}

template <typename T>
const std::vector<PiecewisePolynomial<T>>&
RoadPath<T>::get_path() const { return path_; }

/*
template <typename T>
const Eigen::Vector3d RoadPath::GetPathPosition(
    const Eigen::Vector3d& global_pos) const {
  // TODO(jadecastro): Use linear search as a stop-gap, so that it screams for a
  // real implementation.
}
*/

template <typename T>
std::vector<PiecewisePolynomial<T>> RoadPath<T>::MakePiecewisePolynomial(
    const LaneDirection& initial_lane_dir, const double step_size,
    int num_segments) {
  const Lane* lane = initial_lane_dir.lane;
  bool with_s = initial_lane_dir.with_s;
  T lane_s{cond(with_s, T(0.), T(lane->length()))};
  T monotonic_s{0.};

  GeoPosition geo_pos =
      lane->ToGeoPosition(LanePosition(lane_s, 0. /* r */, 0. /* h */));
  std::cerr << "  " << geo_pos.x << "  " << geo_pos.y << "  " << geo_pos.z
            << std::endl;
  std::vector<T> x0({T(geo_pos.x), T(geo_pos.y), T(geo_pos.z)});

  std::vector<Eigen::VectorXd> x_interior(3);
  for (int index = 0; index < 3; ++index) {
    x_interior[index] = Eigen::VectorXd::Zero(num_segments);
  }

  s_knots_.push_back(T(0.));
  geo_knots_.emplace_back(x0);

  for (int i = 0; i < num_segments-1; ++i) {
    if (with_s) {
      lane_s += T(step_size);
    } else {
      lane_s -= T(step_size);
    }
    monotonic_s += T(step_size);
    s_knots_.emplace_back(monotonic_s);
    geo_pos = lane->ToGeoPosition(LanePosition(lane_s, 0. /* r */, 0. /* h */));
    std::cerr << "  " << geo_pos.x << "  " << geo_pos.y << "  " << geo_pos.z
              << std::endl;
    geo_knots_.push_back({T(geo_pos.x), T(geo_pos.y), T(geo_pos.z)});

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

      // Adjust the new lane position based on the distance overshoot and the
      // new lane's direction.
      lane_s = cond(with_s, out_distance, T(lane->length()) - out_distance);
    }
    geo_pos = lane->ToGeoPosition(LanePosition(lane_s, 0. /* r */, 0. /* h */));
    std::cerr << "  " << geo_pos.x << "  " << geo_pos.y << "  " << geo_pos.z
              << std::endl;
    x_interior[0](i) = geo_pos.x;  // TODO(jadecastro): Populate a std::vector
                                   // instead.
    x_interior[1](i) = geo_pos.y;
    x_interior[2](i) = geo_pos.z;
  }
  // Take the lane end of the final lane as the end of the path.
  T path_end{};
  if (with_s) {
    path_end = (lane_s == lane->length()) ? lane_s + step_size : lane->length();
  } else {
    path_end = (lane_s != 0.) ? -step_size : 0.;
  }
  geo_pos = lane->ToGeoPosition(LanePosition(path_end, 0. /* r */, 0. /* h */));
    std::cerr << "  " << geo_pos.x << "  " << geo_pos.y << "  " << geo_pos.z
              << std::endl;
  std::vector<T> xf{T(geo_pos.x), T(geo_pos.y), T(geo_pos.z)};

  // Add two more knots to get the count right?
  // TODO(jadecastro): Fake these for now, until we've understood what's really
  // happening here.
  s_knots_.emplace_back(monotonic_s + step_size);
  geo_knots_.push_back({geo_pos.x, geo_pos.y, geo_pos.z});
  s_knots_.emplace_back(monotonic_s + 2 * step_size);
  geo_knots_.push_back({geo_pos.x, geo_pos.y, geo_pos.z});

  // Create the resulting piecewise polynomial.
  std::vector<PiecewisePolynomial<T>> result{};
  for (int index = 0; index < 3; ++index) {
    std::cerr << " s_knots size: " << s_knots_.size() << std::endl;
    std::cerr << " x_interior[index] size: " << x_interior[index].size()
              << std::endl;
    PiecewisePolynomial<T> poly = nWaypointCubicSpline(
        s_knots_, x0[index], 0. /* x0_dot */, xf[index], 0. /* xf_dot */,
        x_interior[index]);
    result.emplace_back(poly);
  }
  path_ = result;
  return result;
}

template class RoadPath<double>;

}  // namespace automotive
}  // namespace drake
