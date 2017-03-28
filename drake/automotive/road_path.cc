#include "drake/automotive/road_path.h"

#include <iostream>

#include <Eigen/Dense>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/polynomial.h"
#include "drake/common/cond.h"
#include "drake/common/trajectories/qp_spline/spline_generation.h"
#include "drake/math/saturate.h"

namespace drake {
namespace automotive {

using maliput::api::BranchPoint;
using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LaneEndSet;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;

namespace {

template <typename T>
static const T EvalP(const T& sk, const std::vector<T>& s,
                     const std::vector<T>& Ds) {
  const T P01 = (sk - s[0]) * (sk - s[1]) / ((s[2] - s[0]) * (s[2] - s[1]));
  const T P02 = (sk - s[0]) * (sk - s[2]) / ((s[1] - s[0]) * (s[1] - s[2]));
  const T P12 = (sk - s[1]) * (sk - s[2]) / ((s[0] - s[1]) * (s[0] - s[2]));
  return P01 * Ds[2]  + P02 * Ds[1] + P12 * Ds[0];
}

}  // namespace

template <typename T>
RoadPath<T>::RoadPath(const RoadGeometry& road,
                      const LaneDirection& initial_lane_dir,
                      const double step_size,
                      int num_breaks)
    : num_breaks_(num_breaks),
      geo_knots_(num_breaks, MatrixX<T>::Zero(3, 1)) {
  MakePiecewisePolynomial(initial_lane_dir, step_size, num_breaks);
  ComputeDerivatives();
}

template <typename T>
RoadPath<T>::~RoadPath() {}

template <typename T>
const PiecewisePolynomial<T>& RoadPath<T>::get_path() const { return path_; }

template <typename T>
const T RoadPath<T>::GetClosestPathPosition(
    const Eigen::Vector3d& geo_pos, const double s_guess) const {
  int index = path_.getSegmentIndex(s_guess);
  const T seg_start = path_.getStartTime(index);
  const T seg_end = path_.getEndTime(index);
  const T seg_midpoint = (seg_start + seg_end) / 2.;
  const T lower_bound = path_.getStartTime();
  const T upper_bound = path_.getEndTime();

  int max_iterations{20};
  const T width{T(1.)};
  const T termination_cond{width};

  std::vector<T> s{seg_start, seg_midpoint, seg_end};
  std::vector<T> Ds(3);
  std::cerr << " " << s[0] << " " << s[1] << " " << s[2] << std::endl;

  T sk_last{0.};
  T sk_result{0.};
  for (int i = 0; i < max_iterations; ++i) {
    std::cerr << " i: " << i << std::endl;
    for (int j = 0; j < 3; ++j) {
      Ds[j] = (path_.value(s[j]) - geo_pos).squaredNorm();
    }
    // Quadratic cost.
    const T den =
        ((s[1] - s[2]) * Ds[0] + (s[2] - s[0]) * Ds[1] + (s[0] - s[1]) * Ds[2]);
    DRAKE_DEMAND(den != 0.);
    const T sk = 0.5 * ((s[1] * s[1] - s[2] * s[2]) * Ds[0] +
                        (s[2] * s[2] - s[0] * s[0]) * Ds[1] +
                        (s[0] * s[0] - s[1] * s[1]) * Ds[2]) / den;
    T sk_sat = math::saturate(sk, lower_bound, upper_bound);

    // Newton solver.
    const T gradient = path_jacobian_.value(sk_sat)(0);
    const T curvature = path_hessian_.value(sk_sat)(0);
    DRAKE_DEMAND(curvature != 0.);
    sk_result = math::saturate(sk_sat - gradient / curvature,
                               lower_bound, upper_bound);

    // Termination criterion.
    if (std::abs(sk_result - sk_last) <= termination_cond) return sk_result;
    sk_last = sk_result;

    // Obtain new s's.
    const std::vector<T> Ps{EvalP(s[0], s, Ds), EvalP(s[1], s, Ds),
          EvalP(s[2], s, Ds), EvalP(sk, s, Ds)};
    const auto max_element = std::max_element(Ps.begin(), Ps.end());
    int new_index = std::distance(Ps.begin(), max_element);
    // N.B. The first three elements of Ps correspond to the the elements of s.
    if (new_index < 3) s[new_index] = sk;
    std::cerr << " " << s[0] << " " << s[1] << " " << s[2] << std::endl;
  }

  return sk_result;
}

template <typename T>
PiecewisePolynomial<T> RoadPath<T>::MakePiecewisePolynomial(
    const LaneDirection& initial_lane_direction, const double step_size,
    int num_breaks) {
  LaneDirection ld = initial_lane_direction;
  T s_lane{cond(ld.with_s, T(0.), T(ld.lane->length()))};
  T s_break{0.};

  for (int i = 0; i < num_breaks-1; ++i) {
    s_breaks_.emplace_back(s_break);
    s_break += T(step_size);

    GeoPosition geo_pos =
        ld.lane->ToGeoPosition(LanePosition(s_lane, 0. /* r */, 0. /* h */));
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

  // Resize if necessary.
  for (int i = 0; i < num_breaks_ - static_cast<int>(s_breaks_.size()); ++i) {
    geo_knots_.pop_back();
  }

  // Create the resulting piecewise polynomial.
  path_ = PiecewisePolynomial<T>::Cubic(s_breaks_, geo_knots_);
  return path_;
}

template <typename T>
void RoadPath<T>::ComputeDerivatives() {
  DRAKE_DEMAND(path_.getNumberOfSegments() > 0);
  path_jacobian_ = path_.derivative();
  path_hessian_ = path_.derivative(2);
}

template class RoadPath<double>;

}  // namespace automotive
}  // namespace drake
