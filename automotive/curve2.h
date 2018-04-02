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

/// 
template <typename T, typename Result, typename Point>
class Curve {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Curve)

  explicit Curve(double path_length) : path_length_(path_length) {}

  virtual ~Curve() = default;

  Result GetWaypoint(const T& path_distance) {
    Result result;
    result = DoGetWaypoint(path_distance, waypoint_index_);
    return DoGetWaypoint(path_distance);
  }

  /// @return the length of this curve (the total distance traced).
  double path_length() const { return path_length_; }

  /// @return the waypoints associated with this curve.
  const std::vector<Point>& waypoints() const { return waypoints_; }

  int get_waypoint_index() const { return waypoint_index_; }

 private:
  virtual Result DoGetWaypoint(
      const T& path_distance, int waypoint_index) const = 0;

  std::vector<Point> waypoints_;
  double path_length_{};
  int waypoint_index_{};
};

typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Point2;

/// A result type for the GetWaypoint method.
template <typename T>
struct PositionResultT {
  // *** Make this read Point2T.
  Eigen::Matrix<T, 2, 1, Eigen::DontAlign> position{
    Eigen::Matrix<T, 2, 1, Eigen::DontAlign>::Zero()};
  Eigen::Matrix<T, 2, 1, Eigen::DontAlign> position_dot{
    Eigen::Matrix<T, 2, 1, Eigen::DontAlign>::Zero()};
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
///
template <typename T>
class Curve2 : public Curve<T, PositionResultT<T>, Point2> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Curve2)

  typedef Eigen::Matrix<T, 2, 1, Eigen::DontAlign> Point2T;
  /// A two-dimensional Cartesian point that is alignment-safe.
  typedef PositionResultT<T> PositionResult;

  /// Constructor that traces through the given @p waypoints in order.
  /// Throws an error if @p waypoints.size() == 1.
  explicit Curve2(const std::vector<Point2>& waypoints)
      : Curve<T, PositionResult, Point2>(GetLength(waypoints)),
        waypoints_(waypoints) {
    // TODO(jwnimmer-tri) We should reject duplicate adjacent
    // waypoints (derivative problems); this will probably come for
    // free as part of the spline refactoring.
  }

  /// Returns the Curve's @p PositionResult::position at @p path_distance,
  /// as well as its first derivative @p PositionResult::position_dot with
  /// respect to @p path_distance.
  ///
  /// The @p path_distance is clipped to the ends of the curve:
  /// - A negative @p path_distance is interpreted as a @p path_distance
  ///   of zero.
  /// - A @p path_distance that exceeds the @p path_length() of the curve
  ///   is interpreted as a @p path_distance equal to the @p path_length().
  ///
  /// The @p position_dot derivative, when evaluated exactly at a waypoint,
  /// will be congruent with the direction of one of the (max two) segments
  /// that neighbor the waypoint.  (At the first and last waypoints, there
  /// is only one neighboring segment.)  TODO(jwnimmer-tri) This will no
  /// longer be true once this class uses a spline.
  PositionResult DoGetWaypoint(const T& path_distance, int waypoint_index)
      const override {
    using std::max;

    // TODO(jwnimmer-tri) This implementation is slow (linear search)
    // and incorrect (discontinuous; not a spline).  But it will do
    // for now, until we get a 2d spline code in C++.

    PositionResult result;
    current_waypoint_ = waypoints_.begin();

    // We need at least one segment.  If not, we're just zero.
    if (waypoints_.size() < 2) {
      return result;
    }

    // Iterate over the segments, up through the requested path_distance.
    T remaining_distance = max(T{0.0}, path_distance);
    for (auto point0 = waypoints_.begin(), point1 = point0 + 1;
         point1 != waypoints_.end();  // BR
         point0 = point1++) {
      Point2 relative_step{*point1 - *point0};
      const double length = relative_step.norm();
      if (remaining_distance <= length) {
        auto fraction = remaining_distance / length;
        result.position = *point0 + fraction * Point2T{relative_step};
        Point2T position_dot = relative_step / length;
        MakePointCoherent(path_distance, &position_dot);
        result.position_dot.head(2) = position_dot;
        current_waypoint_index_ = point0;
        return result;
      }
      remaining_distance -= length;
    }

    // Oops, we ran out of waypoints; return the final one.
    // The position_dot is congruent with the final segment.
    {
      Point2T final_waypoint = waypoints_.back();
      MakePointCoherent(path_distance, &final_waypoint);
      result.position = final_waypoint;
      Point2 ultimate = waypoints_.back();
      Point2 penultimate = waypoints_.at(waypoints_.size() - 2);
      const Point2 relative_step{ultimate - penultimate};
      const double length = relative_step.norm();
      Point2T position_dot = relative_step / length;
      MakePointCoherent(path_distance, &position_dot);
      result.position_dot.head(2) = position_dot;
    }

    current_waypoint_ = waypoints_.end();
    return result;
  }

 private:
  // TODO(jwnimmer-tri) Make sure this uses the spline length, not
  // sum-segment-length, once this class uses a spline.
  static double GetLength(const std::vector<Point2>& waypoints) {
    double result{0.0};
    if (waypoints.empty()) {
      return result;
    }
    if (waypoints.size() == 1) {
      throw std::invalid_argument{"single waypoint"};
    }
    for (auto point0 = waypoints.begin(), point1 = point0 + 1;
         point1 != waypoints.end();  // BR
         point0 = point1++) {
      const Point2 relative_step{*point1 - *point0};
      const double length = relative_step.norm();
      result += length;
    }
    return result;
  }

  static void MakePointCoherent(const T& donor, Point2T* point2) {
    autodiffxd_make_coherent(donor, &(*point2)(0));
    autodiffxd_make_coherent(donor, &(*point2)(1));
  }
};

typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Point3;

/// A result type for the GetWaypoint method.
template <typename T>
struct PositionVelocityResultT : PositionResultT {
  T velocity{0.};
};

/// Curve3 represents a path through two-dimensional Cartesian space annotated
/// with velocities. Given a list of waypoints, it linearly interpolates between
/// them.
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
///
template <typename T>
class Curve3 : public Curve<T, PositionVelocityResultT<T>, Point3> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Curve3)

  typedef Eigen::Matrix<T, 3, 1, Eigen::DontAlign> Point3T;
  /// A two-dimensional Cartesian point that is alignment-safe.
  typedef PositionVelocityResultT<T> PositionVelocityResult;

  /// Constructor that traces through the given @p waypoints in order.
  /// Throws an error if @p waypoints.size() == 1.
  explicit Curve3(const std::vector<Point3>& waypoints)
      : Curve<T, PositionVelocityResult, Point3>(GetLength(waypoints)),
        waypoints_(waypoints),
        curve2_(waypoints) {}

  /// Returns the Curve's @p PositionResult::position at @p path_distance,
  /// as well as its first derivative @p PositionResult::position_dot with
  /// respect to @p path_distance.
  ///
  /// The @p path_distance is clipped to the ends of the curve:
  /// - A negative @p path_distance is interpreted as a @p path_distance
  ///   of zero.
  /// - A @p path_distance that exceeds the @p path_length() of the curve
  ///   is interpreted as a @p path_distance equal to the @p path_length().
  ///
  /// The @p position_dot derivative, when evaluated exactly at a waypoint,
  /// will be congruent with the direction of one of the (max two) segments
  /// that neighbor the waypoint.  (At the first and last waypoints, there
  /// is only one neighboring segment.)  TODO(jwnimmer-tri) This will no
  /// longer be true once this class uses a spline.
  PositionVelocityResult DoGetWaypoint(
      const T& path_distance, int waypoint_index) const override {
    
  }

 private:
  std::vector<Point3> waypoints_;
  Curve2 curve2_;
};

}  // namespace automotive
}  // namespace drake
