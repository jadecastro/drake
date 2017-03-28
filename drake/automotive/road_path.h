#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace automotive {

/// RoadPath is a
///
/// This class is explicitly instantiated for the following scalar types. No
/// other scalar types are supported.
/// - double
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///           Only double is supported.
//
// TODO(jadecastro): Shadow ToLeft and ToRight from RoadGeometry, with possible
// position dependence.
template <typename T>
class RoadPath {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RoadPath)

  explicit RoadPath(const maliput::api::RoadGeometry& road,
                    const LaneDirection& initial_lane_direction,
                    const double step_size, int num_breaks);
  ~RoadPath();

  const PiecewisePolynomial<T>& get_path() const;

  // Computes the closest path position of an arbitrary point to a curve.
  // TODO(jadecastro): Implement this.
  const T GetClosestPathPosition(const Vector3<T>& geo_pos,
                                 const double s_guess) const;

 private:
  // Traverse the road, starting from an initial LaneDirection, and build a
  // PiecewisePolynomial until 1) a given a number of segments has been
  // traversed, or 2) the end of the road has been reached.
  //
  // If a BranchPoint is encountered in which there is more than one ongoing
  // lane, the zero-index lane is always selected.
  const PiecewisePolynomial<T> MakePiecewisePolynomial(
      const LaneDirection& initial_lane_direction, const double step_size,
      int num_breaks);

  PiecewisePolynomial<T> path_{};  // The path representing the mid-curve of the
                                   // road.
  PiecewisePolynomial<T> path_prime_{};         // First derivative of path_.
  PiecewisePolynomial<T> path_double_prime_{};  // Second derivative of path_.
};

}  // namespace automotive
}  // namespace drake
