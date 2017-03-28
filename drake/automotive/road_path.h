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

/// RoadPath is a ..
///
/// This class is explicitly instantiated for the following scalar types. No
/// other scalar types are supported.
/// - double
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///           Only double is supported.
//
// TODO(jadecastro): Extract ToLeft and ToRight from RoadGeometry and shadow
// those here.  Do we need to make these position-dependent?
template<typename T>
class RoadPath {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RoadPath)

  explicit RoadPath(const maliput::api::RoadGeometry& road,
                    const LaneDirection& initial_lane_dir,
                    const double step_size,
                    int num_breaks);
  ~RoadPath();

  const PiecewisePolynomial<T>& get_path() const;

  const T GetClosestPathPosition(
      const Eigen::Vector3d& geo_pos, const double s_guess) const;

 private:
  // Traverse the road, starting from an initial LaneDirection, and build a
  // PiecewisePolynomial until 1) given a number of segments has been traversed,
  // or 2) the end of the road has been reached.
  PiecewisePolynomial<T> MakePiecewisePolynomial(
      const LaneDirection& initial_lane_direction, const double step_size,
      int num_breaks);

  void ComputeDerivatives();

  int num_breaks_{};
  std::vector<T> s_breaks_{};  // Here?
  std::vector<MatrixX<T>> geo_knots_{};  // Here?
  PiecewisePolynomial<T> path_{};
  PiecewisePolynomial<T> path_jacobian_{};
  PiecewisePolynomial<T> path_hessian_{};
};

}  // namespace automotive
}  // namespace drake
