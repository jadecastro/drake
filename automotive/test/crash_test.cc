// Adapted from GTEST sample code by Vlad S.
// https://github.com/google/googletest/blob/master/googletest/samples/sample8_unittest.cc

#include "drake/automotive/automotive_simulator.h"

#include "gtest/gtest.h"

#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/prius_vis.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace automotive {
namespace {
#if GTEST_HAS_COMBINE

using ::testing::TestWithParam;
using ::testing::Bool;
using ::testing::Values;
using ::testing::Combine;

class CrashTest2 : public TestWithParam< ::testing::tuple<double, double, double, double> > {
 protected:
  virtual void SetUp() {
    v1 = ::testing::get<0>(GetParam());
    v2 = ::testing::get<1>(GetParam());
    accel = ::testing::get<2>(GetParam());
    distance = ::testing::get<3>(GetParam()) + 5.;
  }
  virtual void TearDown() {}
  double v1;
  double v2;
  double accel;
  double distance;
};

// TEST_P(CrashTest2, CorrectlyReadsParameters) {
//     EXPECT_GT(v1, 0);
//     EXPECT_GT(v2, 0);
//     EXPECT_GT(accel, 0);
//     EXPECT_GT(distance, 0);
// }

TEST_P(CrashTest2, ReadsSimpleCarState) {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(
      std::make_unique<lcm::DrakeMockLcm>());
  const double kX_car1{distance};
  const double kY_car1{0};
  const double kVelocity_car1{v1};
  const double kVelocity_car2{v2};
  const double kCar1Acceleration{accel};
  const int kCar1Index{0};
  const int kCar2Index{1};

  SimpleCarState<double> initial_state_car1;
  initial_state_car1.set_x(kX_car1);
  initial_state_car1.set_y(kY_car1);
  initial_state_car1.set_velocity(kVelocity_car1);
  initial_state_car1.set_acceleration(kCar1Acceleration);
  const int car1_id = simulator->AddPriusSimpleCar(
      "Car1", "Channel", initial_state_car1);

  const MaliputRailcarParams<double> params;
  const maliput::api::RoadGeometry* road =
      simulator->SetRoadGeometry(
          std::make_unique<const maliput::dragway::RoadGeometry>(
              maliput::api::RoadGeometryId("TestDragway"), 1 /* num lanes */,
              10000 /* length */, 4 /* lane width */, 1 /* shoulder width */,
              5 /* maximum_height */,
              std::numeric_limits<double>::epsilon() /* linear_tolerance */,
              std::numeric_limits<double>::epsilon() /* angular_tolerance */));
  MaliputRailcarState<double> initial_state_car2;
  initial_state_car2.set_speed(kVelocity_car2);
  const int car2_id = simulator->AddPriusMaliputRailcar("Car2",
      LaneDirection(road->junction(0)->segment(0)->lane(0)), params,
      initial_state_car2);

  simulator->Start(100.0); // target_realtime_rate

  // simulator->StepBy(1.0);
  //simulator->StepBy(1.0);
  //simulator->StepBy(1.0);
  //simulator->StepBy(1.0);
  // simulator->StepBy(1.0);

  const systems::rendering::PoseBundle<double> poses =
      simulator->GetCurrentPoses();
  ASSERT_EQ(poses.get_num_poses(), 2);
  ASSERT_EQ(poses.get_model_instance_id(kCar1Index), car1_id);
  ASSERT_EQ(poses.get_model_instance_id(kCar2Index), car2_id);
  
  const Isometry3<double>& pose_1 = poses.get_pose(kCar1Index);
  const Isometry3<double>& pose_2 = poses.get_pose(kCar2Index);

  EXPECT_EQ(pose_2.translation().x() + 5.0, pose_1.translation().x());
  // 5.0 is the length of the car
}

// In order to run value-parameterized tests, you need to instantiate them,
// or bind them to a list of values which will be used as test parameters.
// You can instantiate them in a different translation module, or even
// instantiate them several times.
//
// Here, we instantiate our tests with a list of parameters. We must combine
// all variations of the boolean flag suppressing PrecalcPrimeTable and some
// meaningful values for tests. We choose a small value (1), and a value that
// will put some of the tested numbers beyond the capability of the
// PrecalcPrimeTable instance and some inside it (10). Combine will produce all
// possible combinations.
INSTANTIATE_TEST_CASE_P(MeaningfulTestParameters,
                        CrashTest2,
                        Combine(Values(40),     // velocities for car 1
                                Values(40),     // velocities for car 2
                                Values(-20),   // acceleration of car 1
                                Values(2)));     // distance between the cars

#else

// Google Test may not support Combine() with some compilers. If we
// use conditional compilation to compile out all code referring to
// the gtest_main library, MSVC linker will not link that library at
// all and consequently complain about missing entry point defined in
// that library (fatal error LNK1561: entry point must be
// defined). This dummy test keeps gtest_main linked in.
TEST(DummyTest, CombineIsNotSupportedOnThisPlatform) {}

#endif  // GTEST_HAS_COMBINE
}  // namespace
}  // namespace automotive
}  // namespace drake
