#include "drake/automotive/road_path.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace automotive {
namespace {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::monolane::ArcOffset;
using maliput::monolane::Builder;
using maliput::monolane::Connection;
using maliput::monolane::Endpoint;
using maliput::monolane::EndpointXy;
using maliput::monolane::EndpointZ;

// The length of the straight lane segment.
const double kStraightRoadLength{10};

// The arc radius and angular displacement of the curved road segment.
const double kCurvedRoadRadius{10};
const double kCurvedRoadTheta{M_PI_2};

// Mis
const double kTotalRoadLength{kStraightRoadLength +
                              kCurvedRoadRadius * M_PI / 2.};
const EndpointZ kEndZ{EndpointZ(0, 0, 0, 0)};

// Build a road with two lanes in series.
std::unique_ptr<const RoadGeometry> MakeTwoLaneRoad(bool is_opposing) {
  Builder builder(maliput::api::RBounds(-2, 2), /* lane_bounds       */
                  maliput::api::RBounds(-4, 4), /* driveable_bounds  */
                  0.01,                         /* linear tolerance  */
                  0.5 * M_PI / 180.0);          /* angular_tolerance */
  const Connection* straight_lane_connection =
      builder.Connect("0_fwd",                              /* id     */
                      Endpoint(EndpointXy(0, 0, 0), kEndZ), /* start  */
                      kStraightRoadLength,                  /* length */
                      kEndZ);                               /* z_end  */

  if (is_opposing) {
    //
    const Connection* curved_lane_connection = builder.Connect(
        "1_rev", /* id     */
        Endpoint(EndpointXy(kStraightRoadLength + kCurvedRoadRadius,
                            kCurvedRoadRadius, 1.5 * M_PI),
                 kEndZ),                                 /* start  */
        ArcOffset(kCurvedRoadRadius, -kCurvedRoadTheta), /* arc    */
        kEndZ);                                          /* z_end  */
    builder.SetDefaultBranch(
        straight_lane_connection, LaneEnd::kFinish, /* in_end */
        curved_lane_connection, LaneEnd::kFinish);  /* out_end */
    builder.SetDefaultBranch(
        curved_lane_connection, LaneEnd::kFinish,    /* in_end */
        straight_lane_connection, LaneEnd::kFinish); /* out_end */
  } else {
    //
    const Connection* curved_lane_connection = builder.Connect(
        "1_fwd",                                                /* id     */
        Endpoint(EndpointXy(kStraightRoadLength, 0, 0), kEndZ), /* start  */
        ArcOffset(kCurvedRoadRadius, kCurvedRoadTheta),         /* arc    */
        kEndZ);                                                 /* z_end  */
    builder.SetDefaultBranch(
        straight_lane_connection, LaneEnd::kFinish, /* in_end */
        curved_lane_connection, LaneEnd::kStart);   /* out_end */
    builder.SetDefaultBranch(
        curved_lane_connection, LaneEnd::kStart,     /* in_end */
        straight_lane_connection, LaneEnd::kFinish); /* out_end */
  }

  return builder.Build(maliput::api::RoadGeometryId({"TwoLaneStretchOfRoad"}));
}

const Lane* GetLaneById(const RoadGeometry& road, const std::string& lane_id) {
  if (road.junction(0)->id().id == lane_id) {
    return road.junction(0)->segment(0)->lane(0);
  } else {
    return road.junction(1)->segment(0)->lane(0);
  }
}

// Tests the constructor given a sufficient number of points.
GTEST_TEST(IdmControllerTest, ConstructorOpposingSegments) {
  // Instantiate a road with opposing segments.
  auto road_opposing = MakeTwoLaneRoad(true);
  // Start in the straight segment and progress in the positive-s-direction.
  const LaneDirection initial_lane_dir =
      LaneDirection(GetLaneById(*road_opposing, "j:0_fwd"), /* lane */
                    true);                                  /* with_s */
  // Create the path.
  const auto path = RoadPath<double>(
      *road_opposing, initial_lane_dir, /* initial_lane_direction */
      0.1,                              /* step_size */
      1000);                            /* num_breaks */

  // Expect the lane boundary values to match.
  Vector3<double> expected_value{};
  expected_value << 0., 0., 0.;
  EXPECT_TRUE(CompareMatrices(expected_value, path.get_path().value(0), 1e-5));
  expected_value << 10., 0., 0.;
  EXPECT_TRUE(CompareMatrices(
      expected_value, path.get_path().value(kStraightRoadLength), 1e-3));
  expected_value << 20., 10., 0.;
  EXPECT_TRUE(CompareMatrices(expected_value,
                              path.get_path().value(kTotalRoadLength), 1e-3));
}

GTEST_TEST(IdmControllerTest, ConstructorConfluentSegments) {
  // Instantiate a road with confluent segments.
  auto road_confluent = MakeTwoLaneRoad(true);
  // Start in the curved segment, and progress in the negative-s-direction.
  const LaneDirection initial_lane_dir =
      LaneDirection(GetLaneById(*road_confluent, "j:1_fwd"), /* lane */
                    false);                                  /* with_s */
  // Create the path
  const auto path = RoadPath<double>(
      *road_confluent, initial_lane_dir, /* initial_lane_direction */
      0.1,                               /* step_size */
      1000);                             /* num_breaks */

  // Expect the lane boundary values to match.
  Vector3<double> expected_value{};
  expected_value << 20., 10., 0.;
  EXPECT_TRUE(CompareMatrices(expected_value, path.get_path().value(0), 1e-5));
  expected_value << 10., 0., 0.;
  EXPECT_TRUE(CompareMatrices(
      expected_value, path.get_path().value(kStraightRoadLength), 1e-3));
  expected_value << 0., 0., 0.;
  EXPECT_TRUE(CompareMatrices(
      expected_value, path.get_path().value(kStraightRoadLength +
                                            kCurvedRoadRadius * M_PI / 2.),
      1e-3));
}

// Tests that no failure occurs when the number of points generated is less than
// num_segments.
GTEST_TEST(IdmControllerTest, ConstructorWithTooFewBreakPoints) {
  const double step_size = 2.;
  const int num_breaks = 8;
  // Instantiate a road with opposing segments.
  auto road_opposing = MakeTwoLaneRoad(true);
  // Start in the straight segment and progress in the positive-s-direction.
  const LaneDirection initial_lane_dir =
      LaneDirection(GetLaneById(*road_opposing, "j:0_fwd"), /* lane */
                    true);                                  /* with_s */
  // Create the path.
  const auto path =
      RoadPath<double>(*road_opposing, initial_lane_dir, step_size, num_breaks);

  // Expect there to be a mismatch in the geo position values at the end of the
  // road.
  const Lane* curved_lane = GetLaneById(*road_opposing, "j:1_rev");
  const double s_final = num_breaks * step_size - kStraightRoadLength;
  const GeoPosition geo_pos = curved_lane->ToGeoPosition(
      LanePosition(s_final /* s */, 0. /* r */, 0. /* h */));
  Vector3<double> expected_value{};
  expected_value << geo_pos.x, geo_pos.y, geo_pos.z;
  EXPECT_TRUE(CompareMatrices(expected_value,
                              path.get_path().value(kTotalRoadLength), 1e-3));
}

// Tests that no failure occurs when the number of points generated is less than
// num_segments.
GTEST_TEST(IdmControllerTest, GetClosestPathPosition) {
  const double step_size = 2.;
  const int num_breaks = 8;
  // Instantiate a road with opposing segments.
  auto road_opposing = MakeTwoLaneRoad(true);
  // Start in the straight segment and progress in the positive-s-direction.
  const LaneDirection initial_lane_dir =
      LaneDirection(GetLaneById(*road_opposing, "j:0_fwd"), /* lane */
                    true);                                  /* with_s */
  // Create the path.
  const auto path =
      RoadPath<double>(*road_opposing, initial_lane_dir, step_size, num_breaks);

  // Expect there to be a mismatch in the geo position values at the end of the
  // road.
  Vector3<double> geo_pos{0.1, 0.1, 0.1};
  const double s_guess{0.1};
  EXPECT_NEAR(0., path.GetClosestPathPosition(geo_pos, s_guess), 1e-6);
}


}  // namespace
}  // namespace automotive
}  // namespace drake
