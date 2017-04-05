#include "drake/automotive/pure_pursuit.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/monolane_onramp_merge.h"

namespace drake {
namespace automotive {
namespace {

using maliput::api::GeoPosition;

class PurePursuitTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a straight road with one lane.
    road_.reset(new maliput::dragway::RoadGeometry(
        maliput::api::RoadGeometryId({"Single-Lane Dragway"}),
        1 /* num_lanes */, 100 /* length */, 4. /* lane_width */,
        0. /* shoulder_width */));
  }

  const PurePursuitParams<double> pp_params_{};
  const SimpleCarParams<double> car_params_{};
  std::unique_ptr<maliput::api::RoadGeometry> road_;
};

TEST_F(PurePursuitTest, Evaluate) {
  systems::rendering::PoseVector<double> ego_pose;

  // Set the ego car's pose to be all zeros, and facing parallel to the track.
  ego_pose.set_translation(Eigen::Translation3d(
      0. /* s */, 0. /* r */, 0. /* h */));
  ego_pose.set_rotation(Eigen::Quaternion<double>(
      0. /* w */, 0. /* x */, 0. /* y */, 0. /* z */));

  double result = PurePursuit<double>::Evaluate(
      pp_params_, car_params_, true /* with_s */, *road_, ego_pose);

  // Expect the steering angle to be zero.
  EXPECT_EQ(0., result);

  // Introduce a positive r-offset  (car is to the left of the track).
  ego_pose.set_translation(Eigen::Translation3d(
      0. /* s */, 20. /* r */, 0. /* h */));

  result = PurePursuit<double>::Evaluate(
      pp_params_, car_params_, true /* with_s */, *road_, ego_pose);

  // Expect the steering angle to be between zero and -90-degrees.
  EXPECT_GT(0., result);
  EXPECT_LT(-M_PI_2, result);

  // Introduce a negative r-offset (car is to the right of the track).
  ego_pose.set_translation(Eigen::Translation3d(
      0. /* s */, -20. /* r */, 0. /* h */));

  result = PurePursuit<double>::Evaluate(
      pp_params_, car_params_, true /* with_s */, *road_, ego_pose);

  // Expect the steering angle to be between zero and 90-degrees.
  EXPECT_LT(0., result);
  EXPECT_GT(M_PI_2, result);

  // Introduce an r-offset much greater than the lookahead distance (typically
  // on the order of 10).
  ego_pose.set_translation(Eigen::Translation3d(
      0. /* s */, -1e5 /* r */, 0. /* h */));

  result = PurePursuit<double>::Evaluate(
      pp_params_, car_params_, true /* with_s */, *road_, ego_pose);

  // Expect the steering angle to be close to but not exceeding 90-degrees.
  EXPECT_NEAR(M_PI_2, result, 1e-2);
  EXPECT_GT(M_PI_2, result);

  // Arrange the pose to be to the right of the track and facing it at a
  // 90-degree angle.
  ego_pose.set_translation(Eigen::Translation3d(
      0. /* s */, -1. /* r */, 0. /* h */));
  const double yaw = M_PI_2;
  ego_pose.set_rotation(Eigen::Quaternion<double>(
      std::cos(yaw * 0.5) /* w */, 0. /* x */, 0. /* y */,
      std::sin(yaw * 0.5) /* z */));

  result = PurePursuit<double>::Evaluate(
      pp_params_, car_params_, true /* with_s */, *road_, ego_pose);

  // Expect the steering angle to be between zero and 90-degrees.
  EXPECT_GT(0., result);
  EXPECT_LT(-M_PI_2, result);
}

TEST_F(PurePursuitTest, ComputeGoalPoint) {
  // Tests with a more complex example of a road with two separate lanes that
  // merge together.  See #5169 for a visual representation.
  std::unique_ptr<MonolaneOnrampMerge> merge_example(new MonolaneOnrampMerge);
  auto road = merge_example->BuildOnramp();

  // Set the ego car's pose to be all zeros, and facing parallel to the track.
  systems::rendering::PoseVector<double> pose{};
  pose.set_translation(Eigen::Translation3d(
      0. /* s */, 0. /* r */, 0. /* h */));
  pose.set_rotation(Eigen::Quaternion<double>(
      0. /* w */, 0. /* x */, 0. /* y */, 0. /* z */));

  const GeoPosition goal_position = PurePursuit<double>::ComputeGoalPoint(
      100. /* s_lookahead */, true /* with_s */, *road, pose);

  EXPECT_EQ(0., goal_position.x);
  EXPECT_EQ(0., goal_position.y);
  EXPECT_EQ(0., goal_position.z);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
