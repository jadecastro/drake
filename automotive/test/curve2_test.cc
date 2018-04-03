#include "drake/automotive/curve2.h"

#include <stdexcept>
#include <vector>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

#include <gtest/gtest.h>

namespace drake {
namespace automotive {
namespace {

typedef Vector1d V1;

// Checks the Result::position, Result::other against the expected
// Waypoint.  Result::position_dot is always expected to be √½.
template <typename T = double>
void CheckResult(const Result<T>& actual, const Waypoint& expected) {
  EXPECT_TRUE(CompareMatrices(actual.position, expected.position));
  EXPECT_TRUE(CompareMatrices(actual.other, expected.other));
  EXPECT_TRUE(CompareMatrices(actual.position_dot, Point2d{M_SQRT1_2, M_SQRT1_2},
                              1e-12));
}

// Checks the value() and derivatives() of the actual result against the
// expected Waypoint.  Derivatives for each entry should all be equivalent.
void CheckResult(const Result<AutoDiffXd>& actual, const Waypoint& expected,
                 const Eigen::VectorXd& exp_derivs) {
  CheckResult(actual, expected);
  EXPECT_TRUE(
      CompareMatrices(actual.position[0].derivatives(), exp_derivs, 1e-12));
  EXPECT_TRUE(
      CompareMatrices(actual.position[1].derivatives(), exp_derivs, 1e-12));
  EXPECT_TRUE(
      CompareMatrices(actual.other[0].derivatives(), exp_derivs, 1e-12));
  EXPECT_TRUE(
      CompareMatrices(actual.other[1].derivatives(), exp_derivs, 1e-12));
}

// An empty curve.
GTEST_TEST(Curve2Test, EmptyTest) {
  const std::vector<Waypoint> empty_waypoints{};
  const Curve2<double> empty_curve{empty_waypoints};
  EXPECT_DOUBLE_EQ(empty_curve.path_length(), 0.0);
}

// A single waypoint.
GTEST_TEST(Curve2Test, SingleWaypointTest) {
  const Waypoint point(Point2d{1.0, 2.0});
  EXPECT_TRUE(CompareMatrices(point.position, Point2d{1.0, 2.0}));
  EXPECT_EQ(point.other.size(), 0);
  const std::vector<Waypoint> single_waypoint{
      point,
  };
  EXPECT_THROW(Curve2<double>{single_waypoint}, std::exception);
}

// Incompatible dimensions.
GTEST_TEST(Curve2Test, DimensionsTest) {
  const Waypoint start_point(Point2d{1.0, 2.0}, {});
  const Waypoint end_point(Point2d{2.0, 3.0}, Point2d{4.0, 5.0});
  EXPECT_EQ(start_point.other.size(), 0);
  EXPECT_EQ(end_point.other.size(), 2);
  const std::vector<Waypoint> segment_waypoints{start_point, end_point};
  EXPECT_THROW(Curve2<double>{segment_waypoints}, std::exception);
}

// A single segment.
GTEST_TEST(Curve2Test, BasicTest) {
  const Waypoint start_point(Point2d{1.0, 2.0}, Point2d{7.0, 42.0});
  const Waypoint end_point(Point2d{2.0, 3.0}, Point2d{4.0, 5.0});
  const std::vector<Waypoint> segment_waypoints{start_point, end_point};
  const Curve2<double> segment{segment_waypoints};
  EXPECT_DOUBLE_EQ(segment.path_length(), M_SQRT2);
  auto waypoints = segment.waypoints();
  EXPECT_GE(waypoints.size(), 2);
  EXPECT_EQ(waypoints[0].position, start_point.position);
  EXPECT_EQ(waypoints[1].position, end_point.position);

  const Result<double> before_start = segment.CalcResult(-1.0);
  const Result<double> at_start = segment.CalcResult(0.0);
  const Result<double> at_midpoint =
      segment.CalcResult(0.5 * segment.path_length());
  const Result<double> at_end = segment.CalcResult(segment.path_length());
  const Result<double> after_end = segment.CalcResult(10.0);

  CheckResult(before_start, start_point);
  CheckResult(at_start, start_point);
  const Point2d mid_position = 0.5 * (start_point.position + end_point.position);
  const Point2d mid_other = 0.5 * (start_point.other + end_point.other);
  CheckResult(at_midpoint, {mid_position, mid_other});
  CheckResult(at_end, end_point);
  CheckResult(after_end, end_point);
}

// AutoDiffXd coherency.
GTEST_TEST(Curve2Test, AutoDiffTest) {
  // Configure `position` and `other` vectors such that they are spaced by a
  // distance √½.
  const Waypoint start_point(Point2d{1.0, 2.0}, Point2d{4.0, 5.0});
  const Waypoint end_point(Point2d{2.0, 3.0}, Point2d{5.0, 6.0});
  const std::vector<Waypoint> segment_waypoints{start_point, end_point};
  const Curve2<AutoDiffXd> segment{segment_waypoints};
  auto waypoints = segment.waypoints();

  const V1 derivative{1.};
  const Result<AutoDiffXd> before_start =
      segment.CalcResult(AutoDiffXd(-1, derivative));
  const Result<AutoDiffXd> at_start =
      segment.CalcResult(AutoDiffXd(0.0, derivative));
  const Result<AutoDiffXd> at_midpoint =
      segment.CalcResult(AutoDiffXd(0.5 * segment.path_length(), derivative));
  const Result<AutoDiffXd> at_end =
      segment.CalcResult(AutoDiffXd(segment.path_length(), derivative));
  const Result<AutoDiffXd> after_end =
      segment.CalcResult(AutoDiffXd(10.0, derivative));

  CheckResult(before_start, start_point, V1{0.});
  CheckResult(at_start, start_point, V1{0.});
  const Point2d mid_position = 0.5 * (start_point.position + end_point.position);
  const Point2d mid_other = 0.5 * (start_point.other + end_point.other);
  CheckResult(at_midpoint, {mid_position, mid_other}, V1{M_SQRT1_2});
  CheckResult(at_end, end_point, V1{M_SQRT1_2});
  CheckResult(after_end, end_point, V1{0.});
}

}  // namespace
}  // namespace automotive
}  // namespace drake
