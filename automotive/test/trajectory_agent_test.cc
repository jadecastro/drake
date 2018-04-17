#include "drake/automotive/trajectory_agent.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/extract_double.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace automotive {
namespace {

static constexpr double kSpeed = 10.;

using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;

GTEST_TEST(TrajectoryAgentTest, Topology) {
  const std::vector<double> times{0., 1.};
  const std::vector<Eigen::Isometry3d> poses{
    Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};
  const TrajectoryAgent<double> car(
      CarAgent(), AgentTrajectory::Make(times, poses));

  ASSERT_EQ(0, car.get_num_input_ports());
  ASSERT_EQ(3, car.get_num_output_ports());

  const auto& state_output = car.raw_pose_output();
  EXPECT_EQ(systems::kVectorValued, state_output.get_data_type());
  EXPECT_EQ(SimpleCarStateIndices::kNumCoordinates, state_output.size());

  const auto& pose_output = car.pose_output();
  EXPECT_EQ(systems::kVectorValued, pose_output.get_data_type());
  EXPECT_EQ(PoseVector<double>::kSize, pose_output.size());

  const auto& velocity_output = car.velocity_output();
  EXPECT_EQ(systems::kVectorValued, velocity_output.get_data_type());
  EXPECT_EQ(FrameVelocity<double>::kSize, velocity_output.size());

  ASSERT_FALSE(car.HasAnyDirectFeedthrough());
}

// Check that the outputs are formed as expected.
GTEST_TEST(TrajectoryAgentTest, ConstantSpeedTest) {
  using std::cos;
  using std::sin;

  struct Case {
    double heading;
    double distance;
  };

  const std::vector<Case> cases{
    {0., 1.},
    {M_PI_2, 1.},
    {-M_PI_2, 1.},
    {0.125 * M_PI, 10.},
    {0.125 * M_PI, 1.},
    {0.125 * M_PI, 1.}
  };

  for (const auto& it : cases) {
    Eigen::Isometry3d start_pose({5., 10., 0.});
    start_pose.rotate(
        math::RollPitchYaw<double>(0., 0., it.heading).ToQuaternion());
    Eigen::Isometry3d end_pose = start_pose;
    end_pose.translation() = end_pose.translation() +
        math::RotationMatrix<double>(start_pose.rotation()).matrix() *
        Eigen::Vector3d{it.distance, 0., 0.};

    const std::vector<Eigen::Isometry3d> poses{start_pose, end_pose};
    const std::vector<double> times{0., it.distance / kSpeed};
    const AgentTrajectory trajectory = AgentTrajectory::Make(times, poses);
    std::cout << trajectory.value(0.).speed() << std::endl;

    const TrajectoryAgent<double> car(CarAgent(), trajectory);

    // Check that the systems' outputs are correct over the entire duration of
    // the trajectory.
    const double distance = it.distance;
    const double start_time = 0.;
    const double end_time = distance / kSpeed;

    systems::Simulator<double> simulator(car);
    systems::Context<double>& context = simulator.get_mutable_context();
    std::unique_ptr<systems::SystemOutput<double>> all_output =
        car.AllocateOutput(context);
    simulator.Initialize();

    for (double time = start_time; time <= end_time; time += 0.1) {
      simulator.StepTo(time);

      const double fractional_progress =
          std::min(std::max(0.0, (time * kSpeed) / distance), 1.);

      const double position = it.distance * fractional_progress;
      Eigen::Translation<double, 3> expected_position(
          start_pose.translation() +
          Eigen::Vector3d{cos(it.heading) * position, sin(it.heading) * position, 0.});

      car.CalcOutput(context, all_output.get());

      ASSERT_EQ(3, all_output->get_num_ports());

      // Tests the raw pose output.
      const SimpleCarState<double>* raw_pose =
          dynamic_cast<const SimpleCarState<double>*>(
              all_output->get_vector_data(
                  car.raw_pose_output().get_index()));
      EXPECT_EQ(SimpleCarStateIndices::kNumCoordinates, raw_pose->size());

      const double kMaxErrorPos = 1e-6;
      const double kMaxErrorRad = 1e-6;

      // N.B. We tolerate some small integration errors.
      EXPECT_NEAR(expected_position.x(), raw_pose->x(),
                  kMaxErrorPos);
      EXPECT_NEAR(expected_position.y(), raw_pose->y(),
                  kMaxErrorPos);
      EXPECT_NEAR(it.heading, raw_pose->heading(),
                  kMaxErrorRad);
      EXPECT_DOUBLE_EQ(kSpeed, raw_pose->velocity());

      // Tests the PoseVector output.
      const PoseVector<double>* pose = dynamic_cast<const PoseVector<double>*>(
          all_output->get_vector_data(car.pose_output().get_index()));
      ASSERT_NE(nullptr, pose);
      EXPECT_EQ(PoseVector<double>::kSize, pose->size());

      // N.B. We tolerate some small integration errors.
      EXPECT_NEAR(expected_position.x(),
                      pose->get_translation().translation().x(),
                  kMaxErrorPos);
      EXPECT_NEAR(expected_position.y(),
                      pose->get_translation().translation().y(),
                  kMaxErrorPos);
      EXPECT_NEAR(cos(it.heading / 2),
                  pose->get_rotation().w(),
                  kMaxErrorRad);
      EXPECT_NEAR(sin(it.heading / 2),
                  pose->get_rotation().z(),
                  kMaxErrorRad);

      // Tests the FrameVelocity output.
      const FrameVelocity<double>* velocity =
          dynamic_cast<const FrameVelocity<double>*>(
              all_output->get_vector_data(
                  car.velocity_output().get_index()));

      ASSERT_NE(nullptr, velocity);
      EXPECT_EQ(FrameVelocity<double>::kSize, velocity->size());

      EXPECT_NEAR(kSpeed * cos(it.heading),
                      velocity->get_velocity().translational().x(),
                  kMaxErrorRad);
      EXPECT_NEAR(kSpeed * sin(it.heading),
                      velocity->get_velocity().translational().y(),
                  kMaxErrorRad);
    }
  }
}

GTEST_TEST(TrajectoryAgentTest, ToAutoDiff) {
  const std::vector<double> times{0., 1.};
  const std::vector<Eigen::Isometry3d> poses{
    Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};
  const TrajectoryAgent<double> car(
      CarAgent(), AgentTrajectory::Make(times, poses));

  EXPECT_TRUE(is_autodiffxd_convertible(car, [&](const auto& autodiff_dut) {
    auto context = autodiff_dut.CreateDefaultContext();
    auto output = autodiff_dut.AllocateOutput(*context);

    // Check that the public methods can be called without exceptions.
    autodiff_dut.CalcOutput(*context, output.get());
  }));
}

}  // namespace
}  // namespace automotive
}  // namespace drake
