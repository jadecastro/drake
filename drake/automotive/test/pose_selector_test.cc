#include "drake/automotive/pose_selector.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"

namespace drake {
namespace automotive {
namespace pose_selector {
namespace {

// TODO(jadecastro): Also RoadGeometry, etc.
using systems::rendering::PoseVector;
using systems::rendering::PoseBundle;
using maliput::monolane::ArcOffset;
using maliput::monolane::Connection;
using maliput::monolane::Endpoint;
using maliput::monolane::EndpointXy;
using maliput::monolane::EndpointZ;

constexpr double kLaneLength{100.};
constexpr double kLaneWidth{4.};

constexpr double kEgoSPosition{10.};
constexpr double kEgoRPosition{-0.5 * kLaneWidth};
constexpr double kLeadingSPosition{31.};
constexpr double kTrailingSPosition{7.};
constexpr double kSOffset{4.};
constexpr int kFarAheadIndex{0};
constexpr int kJustAheadIndex{1};
constexpr int kJustBehindIndex{2};
constexpr int kFarBehindIndex{3};

constexpr double kRadius{kLaneLength / 2 / M_PI};  // Length of circle is 100.

// Implements the equation `r * cos(s / r)`.
static const double r_cos(const double s, const double r) {
  return r * std::cos(s / r);
}

// Implements the equation `r * sin(s / r)`.
static const double r_sin(const double s, const double r) {
  return r * std::sin(s / r);
}

// Sets the default `ego_pose` and `traffic_poses` in global coordinates. If
// `is_circle` is true, then the road is assumed circular, beginning at `x =
// kRadius`, `y = 0` and centered at the origin (`is_circle` is false, by
// default).
static void SetDefaultPoses(PoseVector<double>* ego_pose,
                            PoseBundle<double>* traffic_poses,
                            const double is_circle = false) {
  DRAKE_DEMAND(traffic_poses->get_num_poses() == 4);
  DRAKE_DEMAND(kEgoSPosition > 0. && kLaneLength > kEgoSPosition);
  DRAKE_DEMAND(kLeadingSPosition > kEgoSPosition &&
               kLaneLength > kLeadingSPosition);
  DRAKE_DEMAND(kEgoSPosition > kTrailingSPosition && kTrailingSPosition > 0.);

  // Create poses for four traffic cars and one ego positioned in the rightmost
  // lane (smallest r-value). For a road of length 100, the cars are
  // interspersed as follows:
  //
  //     Far Behind   Just Behind     Ego     Just Ahead   Far Ahead
  //   |------o------------o-----------o----------o------------o-------------|
  //  s=0     3            7           10         31           35           100
  //
  // Circular roads are identical but are scaled proportionally to radius.
  std::cerr << " test 1: " << r_cos(0., kRadius) << std::endl;
  ego_pose->set_translation(Eigen::Translation3d(
      is_circle ? r_cos(kEgoSPosition, kRadius + kEgoRPosition)
                : kEgoSPosition, /* x */
      is_circle ? r_sin(kEgoSPosition, kRadius + kEgoRPosition)
                : kEgoRPosition, /* y */
      0.));                      /* z */
  const double s_far_ahead = kLeadingSPosition + kSOffset;
  const Eigen::Translation3d translation_far_ahead(
      is_circle ? r_cos(s_far_ahead, kRadius + kEgoRPosition)
                : s_far_ahead, /* x */
      is_circle ? r_sin(s_far_ahead, kRadius + kEgoRPosition)
                : kEgoRPosition, /* y */
      0.);                       /* z */
  const Eigen::Translation3d translation_just_ahead(
      is_circle ? r_cos(kLeadingSPosition, kRadius + kEgoRPosition)
                : kLeadingSPosition, /* x */
      is_circle ? r_sin(kLeadingSPosition, kRadius + kEgoRPosition)
                : kEgoRPosition, /* y */
      0.);                       /* z */
  const Eigen::Translation3d translation_just_behind(
      is_circle ? r_cos(kTrailingSPosition, kRadius + kEgoRPosition)
                : kTrailingSPosition, /* x */
      is_circle ? r_sin(kTrailingSPosition, kRadius + kEgoRPosition)
                : kEgoRPosition, /* y */
      0.);                       /* z */
  const double s_far_behind = kTrailingSPosition - kSOffset;
  const Eigen::Translation3d translation_far_behind(
      is_circle ? r_cos(s_far_behind, kRadius + kEgoRPosition)
                : s_far_behind, /* x */
      is_circle ? r_sin(s_far_behind, kRadius + kEgoRPosition)
                : kEgoRPosition, /* y */
      0.);                       /* z */
  traffic_poses->set_pose(kFarAheadIndex,
                          Eigen::Isometry3d(translation_far_ahead));
  traffic_poses->set_pose(kJustAheadIndex,
                          Eigen::Isometry3d(translation_just_ahead));
  traffic_poses->set_pose(kJustBehindIndex,
                          Eigen::Isometry3d(translation_just_behind));
  traffic_poses->set_pose(kFarBehindIndex,
                          Eigen::Isometry3d(translation_far_behind));
}

// Builds a pair of concentric circular lanes to test the s-coordinate mapping.
static std::unique_ptr<const maliput::api::RoadGeometry>
ConstructTwoLaneCircularRoad() {
  maliput::monolane::Builder builder(
      maliput::api::RBounds(-kLaneWidth / 2., kLaneWidth / 2.), /* lane */
      maliput::api::RBounds(-kLaneWidth / 2., kLaneWidth / 2.), /* driveable */
      1e-2,  /* linear tolerance */
      1e-2); /* angular_tolerance */

  builder.Connect(
      "inner", /* id */
      Endpoint(EndpointXy(kRadius - 0.5 * kLaneWidth, 0., M_PI / 2.),
               EndpointZ(0., 0., 0., 0.)),               /* start */
      ArcOffset(kRadius - 0.5 * kLaneWidth, 2. * M_PI), /* arc */
      EndpointZ(0., 0., 0., 0.));                        /* z_end */

  builder.Connect(
      "outer", /* id */
      Endpoint(EndpointXy(kRadius + 0.5 * kLaneWidth, 0., M_PI / 2.),
               EndpointZ(0., 0., 0., 0.)),               /* start */
      ArcOffset(kRadius + 0.5 * kLaneWidth, 2. * M_PI), /* arc */
      EndpointZ(0., 0., 0., 0.));                        /* z_end */

  return builder.Build(maliput::api::RoadGeometryId({"Concentric Circles"}));
}

GTEST_TEST(PoseSelectorTest, DragwayTest) {
  // Create a straight road, two lanes wide, in which the two s-r
  // Lane-coordinate frames are aligned with the x-y world coordinates, with s_i
  // = 0 for the i-th lane, i ∈ {0, 1}, corresponds to x = 0, and r_i = 0
  // corresponds to y = (i - 0.5) * kLaneWidth.  See sketch below.
  //
  // +y ^
  //    | -  -  -  -  -  -  -  -   <-- lane 1 (y_1)
  //  0 |-----------------------> +x
  //    | -  -  -  -  -  -  -  -   <-- lane 0 (y_0)
  const int kNumLanes{2};
  const maliput::dragway::RoadGeometry road(
      maliput::api::RoadGeometryId({"Test Dragway"}), kNumLanes, kLaneLength,
      kLaneWidth, 0. /* shoulder width */);
  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(4);

  // Define the default poses.
  SetDefaultPoses(&ego_pose, &traffic_poses);

  // Calculate the current road position and use it to determine the ego car's
  // lane.
  const maliput::api::RoadPosition& ego_position =
      CalcRoadPosition(road, ego_pose.get_isometry());

  maliput::api::RoadPosition leading_position;
  maliput::api::RoadPosition trailing_position;
  std::tie(leading_position, trailing_position) =
      FindClosestPair(road, ego_pose, traffic_poses);

  // Verifies that we are on the road and that the correct car was identified.
  EXPECT_EQ(kLeadingSPosition, leading_position.pos.s);
  EXPECT_EQ(kTrailingSPosition, trailing_position.pos.s);

  // Test that we get the same result when just the leading car is returned.
  const maliput::api::RoadPosition& traffic_position =
      FindClosestLeading(road, ego_pose, traffic_poses);
  EXPECT_EQ(kLeadingSPosition, traffic_position.pos.s);

  // Peer into the adjacent lane to the left.
  std::tie(leading_position, trailing_position) = FindClosestPair(
      road, ego_pose, traffic_poses, ego_position.lane->to_left());

  // Expect to see no cars in the left lane.
  EXPECT_EQ(std::numeric_limits<double>::infinity(), leading_position.pos.s);
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), trailing_position.pos.s);

  // Bump the "just ahead" car into the lane to the left.
  Isometry3<double> isometry_just_ahead =
      traffic_poses.get_pose(kJustAheadIndex);
  isometry_just_ahead.translation().y() += kLaneWidth;
  traffic_poses.set_pose(kJustAheadIndex, isometry_just_ahead);
  std::tie(leading_position, std::ignore) =
      FindClosestPair(road, ego_pose, traffic_poses);

  // Expect the "far ahead" car to be identified.
  EXPECT_EQ(kLeadingSPosition + kSOffset, leading_position.pos.s);

  // Bump the "far ahead" car into the lane to the left.
  Isometry3<double> isometry_far_ahead = traffic_poses.get_pose(kFarAheadIndex);
  isometry_far_ahead.translation().y() += kLaneWidth;
  traffic_poses.set_pose(kFarAheadIndex, isometry_far_ahead);
  std::tie(leading_position, std::ignore) =
      FindClosestPair(road, ego_pose, traffic_poses);

  // Looking forward, we expect there to be no car in sight.
  EXPECT_EQ(std::numeric_limits<double>::infinity(), leading_position.pos.s);

  // Peer into the adjacent lane to the left.
  std::tie(leading_position, trailing_position) = FindClosestPair(
      road, ego_pose, traffic_poses, ego_position.lane->to_left());

  // Expect there to be no car behind on the immediate left and the "just ahead"
  // car to be leading.
  EXPECT_EQ(kLeadingSPosition, leading_position.pos.s);
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), trailing_position.pos.s);
}

GTEST_TEST(PoseSelectorTest, CircularLaneTest) {
  // Create a trivial circular road with two lanes.
  auto road = ConstructTwoLaneCircularRoad();

  PoseVector<double> ego_pose;
  PoseBundle<double> traffic_poses(4);

  // Define the default poses.
  SetDefaultPoses(&ego_pose, &traffic_poses, true);
  std::cerr << " ego x: " << ego_pose.get_isometry().translation().x()
            << std::endl;
  std::cerr << " ego y: " << ego_pose.get_isometry().translation().y()
            << std::endl;

  const auto lane = road->junction(0)->segment(0)->lane(0);
  const double length = lane->length();
  std::cerr << " length: " << length << std::endl;
  for (int i = 0; i < 5; ++i) {
    const double s = (double)i / 4 * length;
    const auto geo_pos =
        lane->ToGeoPosition(maliput::api::LanePosition(s, 0, 0));
    std::cerr << " s: " << s << "  x: " << geo_pos.x << "  y: " << geo_pos.y
              << std::endl;
  }

  // Calculate the current road position and use it to determine the ego car's
  // lane.
  // const maliput::api::RoadPosition& ego_position =
  //    CalcRoadPosition(*road, ego_pose.get_isometry());

  maliput::api::RoadPosition leading_position;
  maliput::api::RoadPosition trailing_position;
  std::tie(leading_position, trailing_position) =
      FindClosestPair(*road, ego_pose, traffic_poses);

  // Verifies that we are on the road and that the correct car was identified.
  EXPECT_EQ(kLeadingSPosition, leading_position.pos.s);
  EXPECT_EQ(kTrailingSPosition, trailing_position.pos.s);
}

}  // namespace
}  // namespace pose_selector
}  // namespace automotive
}  // namespace drake
