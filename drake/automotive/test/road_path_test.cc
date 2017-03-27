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

using maliput::api::LaneEnd;
using maliput::api::RoadGeometry;
using maliput::monolane::ArcOffset;
using maliput::monolane::Builder;
using maliput::monolane::Connection;
using maliput::monolane::Endpoint;
using maliput::monolane::EndpointXy;
using maliput::monolane::EndpointZ;

// The length of the straight lane segment of the road when it is created
// using InitializeTwoLaneStretchOfRoad().
const double kStraightRoadLength{10};

// The arc radius and theta of the road when it is created using
// InitializeCurvedMonoLane().
const double kCurvedRoadRadius{10};
const double kCurvedRoadTheta{M_PI_2};

std::unique_ptr<const RoadGeometry>  InitializeTwoLaneStretchOfRoad(
    bool flip_curve_lane) {
  Builder builder(
      maliput::api::RBounds(-2, 2),   /* lane_bounds       */
      maliput::api::RBounds(-4, 4),   /* driveable_bounds  */
      0.01,                           /* linear tolerance  */
      0.5 * M_PI / 180.0);            /* angular_tolerance */
  const Connection* straight_lane_connection = builder.Connect(
      "point.0",                                                /* id     */
      Endpoint(EndpointXy(0, 0, 0), EndpointZ(0, 0, 0, 0)),     /* start  */
      kStraightRoadLength,                                      /* length */
      EndpointZ(0, 0, 0, 0));                                   /* z_end  */

  if (flip_curve_lane) {
    //
    const Connection* curved_lane_connection = builder.Connect(
        "point.1",                                              /* id     */
        Endpoint(                                               /* start  */
            EndpointXy(kStraightRoadLength + kCurvedRoadRadius,
                       kCurvedRoadRadius, 1.5 * M_PI),
            EndpointZ(0, 0, 0, 0)),
        ArcOffset(kCurvedRoadRadius, -kCurvedRoadTheta),        /* arc    */
        EndpointZ(0, 0, 0, 0));                                 /* z_end  */
    builder.SetDefaultBranch(
        straight_lane_connection, LaneEnd::kFinish,             /* in_end */
        curved_lane_connection, LaneEnd::kFinish);              /* out_end */
    builder.SetDefaultBranch(
        curved_lane_connection, LaneEnd::kFinish,               /* in_end */
        straight_lane_connection, LaneEnd::kFinish);            /* out_end */
  } else {
    //
    const Connection* curved_lane_connection = builder.Connect(
        "point.1",                                              /* id     */
        Endpoint(EndpointXy(kStraightRoadLength, 0, 0),
                 EndpointZ(0, 0, 0, 0)),  /* start  */
        ArcOffset(kCurvedRoadRadius, kCurvedRoadTheta),         /* arc    */
        EndpointZ(0, 0, 0, 0));                                 /* z_end  */
    builder.SetDefaultBranch(
        straight_lane_connection, LaneEnd::kFinish,             /* in_end */
        curved_lane_connection, LaneEnd::kStart);               /* out_end */
    builder.SetDefaultBranch(
        curved_lane_connection, LaneEnd::kStart,                /* in_end */
        straight_lane_connection, LaneEnd::kFinish);            /* out_end */
  }

  return builder.Build(maliput::api::RoadGeometryId({"TwoLaneStretchOfRoad"}));
}

GTEST_TEST(IdmControllerTest, Constructor) {
  auto road = InitializeTwoLaneStretchOfRoad(true);
  const LaneDirection initial_lane_dir = LaneDirection(
      road->junction(0)->segment(0)->lane(0), /* lane */
      true); /* with_s */
  const auto path = RoadPath<double>(*road,
                   initial_lane_dir, /* initial_lane_direction */
                   1., /* step_size */
                   20); /* num_segments */
  EXPECT_EQ(3, path.get_path().size());
  Vector1d value;
  value << 0.;
  EXPECT_TRUE(CompareMatrices(value,
                              path.get_path()[0].value(0.),
                              1e-12));
  value << 10.;
  EXPECT_TRUE(CompareMatrices(value,
                              path.get_path()[0].value(kStraightRoadLength),
                              1e-12));
  value << 20.;
  EXPECT_TRUE(CompareMatrices(
      value, path.get_path()[0].value(kStraightRoadLength +
                                      kCurvedRoadRadius * M_PI / 2.),
      1e-12));
}

}  // namespace
}  // namespace automotive
}  // namespace drake
