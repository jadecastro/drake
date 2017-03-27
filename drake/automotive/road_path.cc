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
                      int num_segments) : num_segments_(num_segments) {
  MakePiecewisePolynomial(initial_lane_dir, step_size, num_segments);
}

template <typename T>
RoadPath<T>::~RoadPath() {}

template <typename T>
const PiecewisePolynomial<T>& RoadPath<T>::get_path() const { return path_; }

/*
template <typename T>
const Eigen::Vector3d RoadPath::GetPathPosition(
    const Eigen::Vector3d& global_pos) const {
  // TODO(jadecastro): Use linear search as a stop-gap, so that it screams for a
  // real implementation.
}
*/

template <typename T>
PiecewisePolynomial<T> RoadPath<T>::MakePiecewisePolynomial(
    const LaneDirection& initial_lane_direction, const double step_size,
    int num_segments) {
  LaneDirection ld = initial_lane_direction;
  T s_lane{cond(ld.with_s, T(0.), T(ld.lane->length()))};
  T s_break{0.};

  for (int i = 0; i < num_segments-1; ++i) {
    s_breaks_.emplace_back(s_break);
    s_break += T(step_size);

    GeoPosition geo_pos =
        ld.lane->ToGeoPosition(LanePosition(s_lane, 0. /* r */, 0. /* h */));
    std::cerr << "  " << geo_pos.x << "  " << geo_pos.y << "  " << geo_pos.z
              << std::endl;
    geo_knots_[i] << T(geo_pos.x), T(geo_pos.y), T(geo_pos.z);

    if (ld.with_s) {
      s_lane += T(step_size);
    } else {
      s_lane -= T(step_size);
    }

    // Break if the end of the road has been reached.
    const LaneEndSet* lane_end_set{ld.with_s
          ? ld.lane->GetOngoingBranches(LaneEnd::kFinish)
          : ld.lane->GetOngoingBranches(LaneEnd::kStart)};
    const T out_distance{
      cond(ld.with_s, T(s_lane - ld.lane->length()), T(-s_lane))};
    if (out_distance >= 0.) {
      if (lane_end_set->size() == 0) break;  // There are no more ongoing lanes.

      // Find a new lane; add it into the stack.  Here we choose only the first.
      ld.lane = lane_end_set->get(0).lane;
      ld.with_s = (lane_end_set->get(0).end == LaneEnd::kStart) ? true : false;

      // Adjust the new lane position based on the distance overshoot and the
      // new lane's direction.
      s_lane =
          cond(ld.with_s, out_distance, T(ld.lane->length()) - out_distance);
    }
  }
  // Make up the distance to the end, if necessary.
  T s_to_end{};
  if (s_lane != ld.lane->length()) {
    if (ld.with_s) {
      s_to_end = ld.lane->length() - s_lane;
      DRAKE_DEMAND(s_to_end > 0.);
      s_lane = ld.lane->length();
    } else {
      s_to_end = s_lane;
      DRAKE_DEMAND(s_to_end > 0.);
      s_lane = 0.;
    }
    s_breaks_.emplace_back(s_break + s_to_end);
    GeoPosition geo_pos =
        ld.lane->ToGeoPosition(LanePosition(s_lane, 0. /* r */, 0. /* h */));
    std::cerr << "  " << geo_pos.x << "  " << geo_pos.y << "  " << geo_pos.z
              << std::endl;
    geo_knots_[s_breaks_.size()] << T(geo_pos.x), T(geo_pos.y), T(geo_pos.z);
  }

  // Create the resulting piecewise polynomial.
  path_ = PiecewisePolynomial<double>::Cubic(s_breaks_, geo_knots_);
  return path_;
}

template class RoadPath<double>;

}  // namespace automotive
}  // namespace drake
