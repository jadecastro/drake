#include "drake/automotive/road_path.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/monolane_onramp_merge.h"
#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace automotive {
namespace {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::monolane::Builder;
using maliput::monolane::Connection;
using maliput::monolane::Endpoint;
using maliput::monolane::EndpointZ;

// The length of the straight lane segment.
const double kStraightRoadLength{10};

// The arc radius, angular displacement, and length of the curved road segment.
const double kCurvedRoadRadius{10};
const double kCurvedRoadTheta{M_PI_2};
const double kCurvedRoadLength{kCurvedRoadRadius * M_PI / 2.};

const double kTotalRoadLength{kStraightRoadLength + kCurvedRoadLength};
const EndpointZ kEndZ{0, 0, 0, 0};  // Specifies zero elevation/super-elevation.

// Build a road with two lanes in series.
std::unique_ptr<const RoadGeometry> MakeTwoLaneRoad(bool is_opposing) {
  Builder builder(maliput::api::RBounds(-2, 2), /* lane_bounds */
                  maliput::api::RBounds(-4, 4), /* driveable_bounds */
                  0.01,                         /* linear tolerance */
                  0.5 * M_PI / 180.0);          /* angular_tolerance */
  builder.Connect("0_fwd",                      /* id */
                  Endpoint({0, 0, 0}, kEndZ),   /* start */
                  kStraightRoadLength,          /* length */
                  kEndZ);                       /* z_end */

  if (is_opposing) {
    // Construct a curved segment that is directionally opposite the straight
    // lane.
    builder.Connect("1_rev", /* id */
                    Endpoint({kStraightRoadLength + kCurvedRoadRadius,
                              kCurvedRoadRadius, 1.5 * M_PI},
                             kEndZ),                        /* start */
                    {kCurvedRoadRadius, -kCurvedRoadTheta}, /* arc */
                    kEndZ);                                 /* z_end */
  } else {
    // Construct a curved segment that is directionally confluent with the
    // straight lane.
    builder.Connect("1_fwd",                                      /* id */
                    Endpoint({kStraightRoadLength, 0, 0}, kEndZ), /* start */
                    {kCurvedRoadRadius, kCurvedRoadTheta},        /* arc */
                    kEndZ);                                       /* z_end */
  }

  return builder.Build(maliput::api::RoadGeometryId({"TwoLaneStretchOfRoad"}));
}

const Lane* GetLaneById(const RoadGeometry& road, const std::string& lane_id) {
  for (int i = 0; i < road.num_junctions(); ++i) {
    if (road.junction(i)->id().id == lane_id) {
      return road.junction(i)->segment(0)->lane(0);
    }
  }
  throw std::runtime_error("No matching junction name in the road network");
}

static const double path_radius(const Vector3<double> value) {
  Vector3<double> result;
  result << value(0) - kStraightRoadLength, value(1) - kCurvedRoadRadius,
      value(2);
  return result.norm();
}

// Tests the constructor given a sufficient number of points.
GTEST_TEST(IdmControllerTest, ConstructOpposingSegments) {
  // Instantiate a road with opposing segments.
  auto road_opposing = MakeTwoLaneRoad(true);
  // Start in the straight segment and progress in the positive-s-direction.
  const LaneDirection initial_lane_dir =
      LaneDirection(GetLaneById(*road_opposing, "j:0_fwd"), /* lane */
                    true);                                  /* with_s */
  // Create a finely-discretized path with a sufficent number of segments to
  // cover the full length.
  const auto path = RoadPath<double>(
      *road_opposing, initial_lane_dir, /* initial_lane_direction */
      0.1,                              /* step_size */
      1000);                            /* num_breaks */
  ASSERT_LE(kTotalRoadLength,
            path.get_path().getEndTime() - path.get_path().getStartTime());

  // Expect the lane boundary values to match.
  Vector3<double> expected_value{};
  Vector3<double> actual_value{};
  expected_value << 0., 0., 0.;
  actual_value = path.get_path().value(0.);
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-2));

  expected_value << 10., 0., 0.;
  actual_value = path.get_path().value(kStraightRoadLength);
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-2));

  expected_value << 20., 10., 0.;
  actual_value = path.get_path().value(kTotalRoadLength);
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-2));

  // Pick a few arbitrary points on the curved section, expect them to trace the
  // arc, hence demonstrating the interpolation is working.
  actual_value = path.get_path().value(4. / 7. * kTotalRoadLength);
  EXPECT_NEAR(kCurvedRoadRadius, path_radius(actual_value), 1e-2);

  actual_value = path.get_path().value(5. / 7. * kTotalRoadLength);
  EXPECT_NEAR(kCurvedRoadRadius, path_radius(actual_value), 1e-2);

  actual_value = path.get_path().value(6. / 7. * kTotalRoadLength);
  EXPECT_NEAR(kCurvedRoadRadius, path_radius(actual_value), 1e-2);

  // Check that the number of segments created is well below the max
  // number specified.
  EXPECT_GT(1000, path.get_path().getNumberOfSegments());
}

GTEST_TEST(IdmControllerTest, ConstructConfluentSegments) {
  // Instantiate a road with confluent segments.
  auto road_confluent = MakeTwoLaneRoad(false);
  // Start in the curved segment, and progress in the negative-s-direction.
  const LaneDirection initial_lane_dir =
      LaneDirection(GetLaneById(*road_confluent, "j:1_fwd"), /* lane */
                    false);                                  /* with_s */
  // Create a finely-discretized path with a sufficent number of segments to
  // cover the full length.
  const auto path = RoadPath<double>(
      *road_confluent, initial_lane_dir, /* initial_lane_direction */
      0.1,                               /* step_size */
      1000);                             /* num_breaks */
  ASSERT_LE(kTotalRoadLength,
            path.get_path().getEndTime() - path.get_path().getStartTime());

  // Expect the lane boundary values to match.
  Vector3<double> expected_value{};
  Vector3<double> actual_value{};
  expected_value << 20., 10., 0.;
  actual_value = path.get_path().value(0.);
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-2));

  expected_value << 10., 0., 0.;
  actual_value = path.get_path().value(kCurvedRoadLength);
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-2));

  expected_value << 0., 0., 0.;
  actual_value = path.get_path().value(kStraightRoadLength + kCurvedRoadLength);
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-2));
}

// Tests with a more complex example of a road with two separate lanes that
// merge together.  See #5169 for a visual representation.
GTEST_TEST(IdmControllerTest, MonolaneMerge) {
  // Instantiate the road.
  std::unique_ptr<MonolaneOnrampMerge> merge_example(new MonolaneOnrampMerge);
  auto road_merge = merge_example->BuildOnramp();

  // Create a path starting at the beginning of the lane that will merge from
  // the left (the main highway).
  const LaneDirection initial_lane_highway =
      LaneDirection(GetLaneById(*road_merge, "j:pre0"), /* lane */
                    true);                              /* with_s */
  const auto path_highway = RoadPath<double>(
      *road_merge, initial_lane_highway, /* initial_lane_direction */
      3.,                                /* step_size */
      10000);                            /* num_breaks */

  // Create a path starting at the beginning of the lane that will merge from
  // the right (the onramp).
  const LaneDirection initial_lane_onramp =
      LaneDirection(GetLaneById(*road_merge, "j:onramp0"), /* lane */
                    false);                                /* with_s */
  const auto path_onramp = RoadPath<double>(
      *road_merge, initial_lane_onramp, /* initial_lane_direction */
      3.,                               /* step_size */
      10000);                           /* num_breaks */

  // Expect the two paths to begin at different locations and end at the same
  // location.
  Vector3<double> expected_value{};
  Vector3<double> actual_value{};
  expected_value << 0., 0., 0.;
  actual_value = path_highway.get_path().value(0.);  // Highway start
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-2));
  std::cerr << " path_highway.get_path().value(0.) "
            << path_highway.get_path().value(0.) << std::endl;

  expected_value << 0., 0., 0.;
  actual_value = path_onramp.get_path().value(0.);  // Onramp start
  EXPECT_TRUE(CompareMatrices(expected_value, actual_value, 1e-2));
  std::cerr << " path_onramp.get_path().value(0.) "
            << path_onramp.get_path().value(0.) << std::endl;

  const double highway_path_length = path_highway.get_path().getEndTime() -
                                     path_highway.get_path().getStartTime();
  const double onramp_path_length = path_onramp.get_path().getEndTime() -
                                    path_onramp.get_path().getStartTime();
  Vector3<double> highway_end_value{};
  Vector3<double> onramp_end_value{};
  std::cerr << " highway_path_length " << highway_path_length << std::endl;
  std::cerr << " onramp_path_length " << onramp_path_length << std::endl;
  highway_end_value =
      path_highway.get_path().value(highway_path_length);  // Highway end
  onramp_end_value =
      path_onramp.get_path().value(onramp_path_length);  // Highway end
  std::cerr << " highway_end_value " << highway_end_value << std::endl;
  std::cerr << " onramp_end_value " << onramp_end_value << std::endl;
  EXPECT_TRUE(CompareMatrices(highway_end_value, onramp_end_value, 1e-2));

  /*
    for (int i = 0; i <= 100; ++i) {
    const double val = i / 100. * onramp_path_length;
    std::cerr << path_onramp.get_path().value(val)(0) << std::endl;
  }
  */
}

}  // namespace
}  // namespace automotive
}  // namespace drake
