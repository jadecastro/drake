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
                    int num_segments);
  ~RoadPath();

  const std::vector<PiecewisePolynomial<T>>& get_path() const;

  // const Eigen::Vector3d GetPathPosition(const Eigen::Vector3d& global_pos)
  //    const;

 private:
  // Traverse the road, starting from an initial LaneDirection, and build a
  // PiecewisePolynomial until 1) given a number of segments has been traversed,
  // or 2) the end of the road has been reached.
  std::vector<PiecewisePolynomial<T>> MakePiecewisePolynomial(
      const LaneDirection& initial_lane_dir, const double step_size,
      int num_segments);

  std::vector<PiecewisePolynomial<T>> path_{};
  std::vector<T> s_knots_{};
  std::vector<std::vector<T>> geo_knots_{};
};

}  // namespace automotive
}  // namespace drake
