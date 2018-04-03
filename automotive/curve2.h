#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/autodiffxd_make_coherent.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace automotive {

template <typename T>
using Point = Eigen::Matrix<T, Eigen::Dynamic, 1, Eigen::DontAlign>;
using Pointd = Point<double>;
template <typename T>
using Point2 = Eigen::Matrix<T, 2, 1, Eigen::DontAlign>;
using Point2d = Point2<double>;

/// Contains a waypoint consisting of a 2-D x,y position and, optionally, other
/// unlabeled data (for instance velocities, orientation, etc).
template <typename T>
struct WaypointT {
  /// Default constructor.
  WaypointT() = default;
  /// Fully-parameterized constructor.
  WaypointT(const Point2<T>& pos, const Point<T>& data)
      : position(pos), other(data){};
  /// Position-only constructor, keeping the trailing/other member field empty.
  WaypointT(const Point2<T>& pos) : position(pos){};

  Point2<T> position{Point2<T>::Zero()};
  Point<T> other;
};

using Waypoint = WaypointT<double>;

/// A result type for the Curve2::CalcResult method.
template <typename T>
struct Result : WaypointT<T> {
  /// Default constructor.
  Result() = default;
  /// Fully-parameterized constructor.
  Result(const Point2<T>& pos, const Point<T>& other, const Point2d& pos_dot)
      : WaypointT<T>(pos, other), position_dot(pos_dot){};

  Point2d position_dot{Point2d::Zero()};
};

/// Curve2 represents a path through two-dimensional Cartesian space. Given a
/// list of waypoints, it linearly interpolates between them.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// TODO(jwnimmer-tri) We will soon trace the path using a spline, but
/// for now it's easiest to just interpolate straight segments, as a
/// starting point.  Callers should not yet rely on <em>how</em> we
/// are traversing between the waypoints.
/// TODO(jadecastro) Consider re-using (wrapping?) PiecewisePolynomial.
template <typename T>
class Curve2 {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Curve2)

  explicit Curve2(const std::vector<Waypoint>& waypoints)
      : waypoints_(waypoints), path_length_(GetLengthAndCheck(waypoints)) {
    // TODO(jwnimmer-tri) We should reject duplicate adjacent
    // waypoints (derivative problems); this will probably come for
    // free as part of the spline refactoring.
  }

  /// @return the waypoints associated with this curve.
  const std::vector<Waypoint>& waypoints() const { return waypoints_; }

  /// @return the length of this curve (the total distance traced).
  double path_length() const { return path_length_; }

  /// Returns Curve2's @p PositionResult::position at @p path_distance, as well
  /// as its first derivative @p PositionResult::position_dot with respect to @p
  /// path_distance.
  ///
  /// The @p path_distance is clipped to the ends of the curve:
  /// - A negative @p path_distance is interpreted as a @p path_distance
  ///   of zero.
  /// - A @p path_distance that exceeds the @p path_length() of the curve
  ///   is interpreted as a @p path_distance equal to the @p path_length().
  Result<T> CalcResult(const T& path_distance) const {
    using std::max;

    // TODO(jwnimmer-tri) This implementation is slow (linear search)
    // and incorrect (discontinuous; not a spline).  But it will do
    // for now, until we get a 2d spline code in C++.

    Result<T> result;

    // We need at least one segment.  If not, ignore the waypoints and return
    // the default Result.
    if (waypoints_.size() < 2) {
      return result;
    }

    // Iterate over the segments, up through the requested path_distance,
    // linearly interpolating the result (position and other data) between
    // waypoints.
    T remaining_distance = max(T{0.0}, path_distance);
    for (auto point0 = waypoints_.begin(), point1 = point0 + 1;
         point1 != waypoints_.end();  // BR
         point0 = point1++) {
      Point2d relative_position_step{point1->position - point0->position};
      Pointd relative_other_step{point1->other - point0->other};
      const T length = relative_position_step.norm();
      if (remaining_distance <= length) {
        auto fraction = remaining_distance / length;
        Point2<T> position =
            point0->position + fraction * Point2<T>{relative_position_step};
        Point<T> other =
            point0->other + fraction * Point<T>{relative_other_step};
        WaypointT<T> waypoint(position, other);
        MakeWaypointCoherent(path_distance, &waypoint);
        return CalcResult(waypoint, relative_position_step);
      }
      remaining_distance -= length;
    }

    // Oops, we ran out of waypoints; return the final one.
    // The position_dot is congruent with the final segment.
    {
      Waypoint ultimate(waypoints_.back());
      Waypoint penultimate(waypoints_.at(waypoints_.size() - 2));
      Point2d relative_position_step{ultimate.position - penultimate.position};
      WaypointT<T> waypoint{ultimate.position, ultimate.other};
      MakeWaypointCoherent(path_distance, &waypoint);
      result = CalcResult(waypoint, relative_position_step);
    }

    return result;
  }

 private:
  // Computes the path length with respect to x-y Waypoint::position, checking
  // for consistency in the dimensions of the Waypoint::other fields.
  //
  // TODO(jwnimmer-tri) Make sure this uses the spline length, not
  // sum-segment-length, once this class uses a spline.
  static double GetLengthAndCheck(const std::vector<Waypoint>& waypoints) {
    double result{0.0};
    if (waypoints.empty()) {
      return result;
    }
    if (waypoints.size() == 1) {
      throw std::invalid_argument{"single waypoint"};
    }
    for (const auto& point : waypoints) {
      DRAKE_THROW_UNLESS(point.other.size() == waypoints[0].other.size());
    }
    for (auto point0 = waypoints.begin(), point1 = point0 + 1;
         point1 != waypoints.end();  // BR
         point0 = point1++) {
      const Point2d relative_step{point1->position - point0->position};
      const double length = relative_step.norm();
      result += length;
    }
    return result;
  }

  /// Helper that returns Result given the current waypoint and relative
  /// position to go from the penultimate waypoint.
  ///
  /// The @p position_dot derivative, when evaluated exactly at a waypoint,
  /// will be congruent with the direction of one of the (max two) segments
  /// that neighbor the waypoint.  (At the first and last waypoints, there
  /// is only one neighboring segment.)  TODO(jwnimmer-tri) This will no
  /// longer be true once this class uses a spline.
  static Result<T> CalcResult(WaypointT<T>& waypoint,
                              const Point2d& relative_position_step) {
    const double length = relative_position_step.norm();
    Point2d position_dot = relative_position_step / length;
    return {waypoint.position, waypoint.other, position_dot};
  }

  static void MakeWaypointCoherent(const T& donor, WaypointT<T>* point) {
    autodiffxd_make_coherent(donor, &point->position(0));
    autodiffxd_make_coherent(donor, &point->position(1));
    for (int i{0}; i < static_cast<int>(point->other.size()); i++) {
      autodiffxd_make_coherent(donor, &point->other(i));
    }
  }

  std::vector<Waypoint> waypoints_;
  double path_length_{};
};

}  // namespace automotive
}  // namespace drake
