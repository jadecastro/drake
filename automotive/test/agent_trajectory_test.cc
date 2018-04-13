#include "drake/automotive/agent_trajectory.h"

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace automotive {
namespace {

using Eigen::Isometry3d;
using Eigen::Quaternion;
using Eigen::Translation;
using Eigen::Vector3d;

// ** TODO **
// 2) Check that poses and velocities from AgentTrajValues ctor are consistent?
// 3) Check that poses and velocities from Isometry3 ctor are consistent?  Only
//    need to do that at knot points.
// 4) MakeFromWaypoints().
//     - Check that interpolation of speeds is handled correctly.
//     - Check that pose interpolation is handled correctly. (check between waypoints)
//     - Compute breaks by hand from speeds and verify they are actually the breaks.
// 5) MakeFromWaypoints(),
//     - Check that speed cannot be < 0.
//     - When speed vector has zero, then it should throw if the next speed is not > 0.
//     - Check that speed() gives the same thing as input speed!
// 6) Check the constant-speed version of MakeFromWaypoints().
// 7) Make sure at least one check ensures that quat_dots correspond to the correct
//    entries of the reconstructed AgentTrajectoryValues.
//    (check: w input == AgentTraj.value(t) for different w's)

// Checks the defaults.
GTEST_TEST(AgentTrajectoryValuesTest, Defaults) {
  const AgentTrajectoryValues actual;

  EXPECT_TRUE(CompareMatrices(actual.isometry3().matrix(),
                              Isometry3d::Identity().matrix()));
  EXPECT_TRUE(
      CompareMatrices(actual.velocity().rotational(), Vector3d::Zero()));
  EXPECT_TRUE(
      CompareMatrices(actual.velocity().translational(), Vector3d::Zero()));
}

// Checks the accessors.
GTEST_TEST(AgentTrajectoryValuesTest, AgentTrajectoryValues) {
  using std::pow;
  using std::sqrt;

  const Translation<double, 3> translation{1., 2., 3.};
  const Vector3d rpy{0.4, 0.5, 0.6};
  const Quaternion<double> rotation = math::RollPitchYawToQuaternion(rpy);
  const Vector3d w{8., 9., 10.};
  const Vector3d v{11., 12., 13.};
  const SpatialVelocity<double> velocity{w, v};
  const AgentTrajectoryValues actual(rotation, translation, velocity);

  Isometry3d expected_isometry(translation);
  expected_isometry.rotate(rotation);
  EXPECT_TRUE(
      CompareMatrices(actual.isometry3().matrix(), expected_isometry.matrix()));
  EXPECT_TRUE(CompareMatrices(actual.velocity().rotational(), w));
  EXPECT_TRUE(CompareMatrices(actual.velocity().translational(), v));
  const Vector3d expected_pose3{translation.x(), translation.y(), rpy.z()};
  EXPECT_TRUE(CompareMatrices(actual.pose3(), expected_pose3));
  EXPECT_EQ(actual.speed(), sqrt(pow(v(0), 2) + pow(v(1), 2) + pow(v(2), 2)));
}

// Unnormalized quaternions are normalized when retrieved.
GTEST_TEST(AgentTrajectoryValuesTest, UnnormalizedQuaternions) {
  const Quaternion<double> rotation(4., 5., 6., 7.);
  const Translation<double, 3> translation{
    Eigen::Translation<double, 3>::Identity()};
  const SpatialVelocity<double> velocity{Vector6<double>::Zero()};
  const AgentTrajectoryValues actual(rotation, translation, velocity);

  Quaternion<double> norm_rotation = rotation;
  norm_rotation.normalize();

  Isometry3d expected_isometry(translation);
  expected_isometry.rotate(norm_rotation);
  EXPECT_TRUE(
      CompareMatrices(actual.isometry3().matrix(), expected_isometry.matrix()));
}


// Mismatched-sized vectors are rejected.
GTEST_TEST(AgentTrajectoryTest, MismatchedSizes) {
  std::vector<double> times{0., 1., 2.};  // A 3D vector.
  const AgentTrajectoryValues dummy_value;
  std::vector<AgentTrajectoryValues> values{dummy_value};  // A 1D vector.
  EXPECT_THROW(
      AgentTrajectory::Make(InterpolationType::kFirstOrderHold, times, values),
      std::exception);

  const Eigen::Isometry3d dummy_poses;
  std::vector<Eigen::Isometry3d> poses{dummy_poses};  // A 1D vector.
  EXPECT_THROW(
      AgentTrajectory::Make(InterpolationType::kFirstOrderHold, times, poses),
      std::exception);

  std::vector<double> speeds{0., 1., 2.};  // A 3D vector.
  EXPECT_THROW(
      AgentTrajectory::MakeFromWaypoints(poses, speeds), std::exception);
}

// Accepts all interpolation types.
GTEST_TEST(AgentTrajectoryTest, InterpolationType) {
  using Type = InterpolationType;

  std::vector<double> times{0., 1., 2.};
  const AgentTrajectoryValues dummy_value;
  std::vector<AgentTrajectoryValues> values{dummy_value, dummy_value,
                                            dummy_value};
  for (const auto& type : {Type::kZeroOrderHold, Type::kFirstOrderHold,
                           Type::kCubic, Type::kPchip}) {
    EXPECT_NO_THROW(AgentTrajectory::Make(type, times, values));
  }
}

// Checks that the provided time-indexed TrajectoryAgentValues agree with the
// values at the knot points.
GTEST_TEST(AgentTrajectoryTest, MakeFromValues) {
  const std::vector<double> times{0., 1., 2.};
  std::vector<AgentTrajectoryValues> values{};
  std::vector<Translation<double, 3>> translations{};
  std::vector<Quaternion<double>> rotations{};
  std::vector<Vector3d> w_vector{};
  std::vector<Vector3d> v_vector{};

  for (const double time : times) {
    translations.push_back({1. + time, 2., 3.});
    rotations.push_back({4. + time, 5., 6., 7.});
    rotations.back().normalize();
    w_vector.push_back({8. + time, 9., 10.});
    v_vector.push_back({11. + time, 12., 13.});
    const SpatialVelocity<double> velocity{w_vector.back(), v_vector.back()};
    values.push_back({rotations.back(), translations.back(), velocity});
  }
  AgentTrajectory trajectory =
      AgentTrajectory::Make(InterpolationType::kFirstOrderHold, times, values);

  for (int i{0}; i < static_cast<int>(times.size()); i++) {
    const AgentTrajectoryValues actual = trajectory.value(times[i]);
    EXPECT_TRUE(CompareMatrices(actual.isometry3().translation(),
                                translations[i].vector(), 1e-12));
    EXPECT_TRUE(CompareMatrices(actual.isometry3().rotation().matrix(),
                                rotations[i].matrix(), 1e-12));
    EXPECT_TRUE(
        CompareMatrices(actual.velocity().rotational(), w_vector[i], 1e-12));
    EXPECT_TRUE(
        CompareMatrices(actual.velocity().translational(), v_vector[i], 1e-12));
  }
}

// Checks that the provided time-indexed poses agree with the values at the knot
// points and that the velocities are correctly inferred.
GTEST_TEST(AgentTrajectoryTest, MakeFromIsometry) {
  const std::vector<double> times{0., 1., 2.};
  std::vector<Eigen::Isometry3d> poses{};
  std::vector<Translation<double, 3>> translations{};
  std::vector<Quaternion<double>> rotations{};

  for (const double time : times) {
    translations.push_back({1. + time, 2., 3.});
    rotations.push_back({4. + time, 5., 6., 7.});
    rotations.back().normalize();
    poses.push_back(Eigen::Isometry3d{translations.back()});
    poses.back().rotate(rotations.back());
  }
  AgentTrajectory trajectory =
      AgentTrajectory::Make(InterpolationType::kFirstOrderHold, times, poses);

  for (int i{0}; i < static_cast<int>(times.size()); i++) {
    const AgentTrajectoryValues actual = trajectory.value(times[i]);
    EXPECT_TRUE(CompareMatrices(actual.isometry3().translation(),
                                translations[i].vector(), 1e-12));
    EXPECT_TRUE(CompareMatrices(actual.isometry3().rotation().matrix(),
                                rotations[i].matrix(), 1e-12));
    // Ensures that the velocities agree with hand-computed values.
    EXPECT_TRUE(
        CompareMatrices(actual.velocity().rotational(), Vector3d{0., 0., 0.}, 1e-12));
    EXPECT_TRUE(
        CompareMatrices(actual.velocity().translational(), Vector3d{0., 0., 0.}, 1e-12));
  }
}

// Checks that the provided speeds and waypoints yield correctly-formed time
// vectors.
GTEST_TEST(AgentTrajectoryTest, MakeFromWaypoint) {
  const std::vector<double> speeds{0., 1., 2.};
  std::vector<Eigen::Isometry3d> poses{};
  std::vector<Translation<double, 3>> translations{};
  std::vector<Quaternion<double>> rotations{};

  for (const double speed : speeds) {
    translations.push_back({1. + speed, 2., 3.});
    rotations.push_back({4. + speed, 5., 6., 7.});
    rotations.back().normalize();
    poses.push_back(Eigen::Isometry3d{translations.back()});
    poses.back().rotate(rotations.back());
  }
  AgentTrajectory trajectory = AgentTrajectory::MakeFromWaypoints(poses, speeds);

  for (int i{0}; i < static_cast<int>(speeds.size()); i++) {
    const AgentTrajectoryValues actual = trajectory.value(speeds[i]);
    EXPECT_TRUE(CompareMatrices(actual.isometry3().translation(),
                                translations[i].vector(), 1e-12));
    EXPECT_TRUE(CompareMatrices(actual.isometry3().rotation().matrix(),
                                rotations[i].matrix(), 1e-12));
    // Ensures that the velocities agree with hand-computed values.
    EXPECT_TRUE(
        CompareMatrices(actual.velocity().rotational(), Vector3d{0., 0., 0.}, 1e-12));
    EXPECT_TRUE(
        CompareMatrices(actual.velocity().translational(), Vector3d{0., 0., 0.}, 1e-12));
  }
}

// Negative speeds are rejected.
GTEST_TEST(AgentTrajectoryTest, NegativeSpeeds) {
  const std::vector<double> speeds{-1, 5.};
  std::vector<Eigen::Isometry3d> poses{};
  std::vector<Translation<double, 3>> translations{};
  std::vector<Quaternion<double>> rotations{};

  for (const double speed : speeds) {
    translations.push_back({1. + speed, 2., 3.});
    rotations.push_back({4. + speed, 5., 6., 7.});
    rotations.back().normalize();
    poses.push_back(Eigen::Isometry3d{translations.back()});
    poses.back().rotate(rotations.back());
  }
  EXPECT_THROW(AgentTrajectory::MakeFromWaypoints(poses, speeds), std::exception);
}

// Deadlock detection rejects unreachable waypoints.
GTEST_TEST(AgentTrajectoryTest, UnreachableWaypoint) {
  const std::vector<double> speeds{0., 0.};
  std::vector<Eigen::Isometry3d> poses{};
  std::vector<Translation<double, 3>> translations{};
  std::vector<Quaternion<double>> rotations{};

  for (int i{0}; i < 2; i++) {
    const double d = i * 10.;
    translations.push_back({1. + d, 2., 3.});
    rotations.push_back({4. + d, 5., 6., 7.});
    rotations.back().normalize();
    poses.push_back(Eigen::Isometry3d{translations.back()});
    poses.back().rotate(rotations.back());
  }
  EXPECT_THROW(AgentTrajectory::MakeFromWaypoints(poses, speeds), std::exception);
}

// Checks that the provided waypoints yield correctly-formed time vectors with
// the constant-speed constructor.
GTEST_TEST(AgentTrajectoryTest, MakeFromWaypointConstantSpeed) {
  const double speed = 5.;
  std::vector<Eigen::Isometry3d> poses{};
  std::vector<Translation<double, 3>> translations{};
  std::vector<Quaternion<double>> rotations{};
  const std::vector<double> expected_times{0., 2.};

  for (int i{0}; i < static_cast<int>(expected_times.size()); i++) {
    const double d = i * 10.;
    translations.push_back({1. + d, 2., 3.});
    rotations.push_back({4. + d, 5., 6., 7.});
    rotations.back().normalize();
    poses.push_back(Eigen::Isometry3d{translations.back()});
    poses.back().rotate(rotations.back());
  }
  AgentTrajectory trajectory = AgentTrajectory::MakeFromWaypoints(poses, speed);

  for (int i{0}; i < static_cast<int>(expected_times.size()); i++) {
    const AgentTrajectoryValues actual = trajectory.value(expected_times[i]);
    EXPECT_TRUE(CompareMatrices(actual.isometry3().translation(),
                                translations[i].vector(), 1e-12));
    EXPECT_TRUE(CompareMatrices(actual.isometry3().rotation().matrix(),
                                rotations[i].matrix(), 1e-12));
    // Ensures that the velocities agree with hand-computed values.
    EXPECT_TRUE(
        CompareMatrices(actual.velocity().rotational(), Vector3d{0., 0., 0.}, 1e-12));
    EXPECT_TRUE(
        CompareMatrices(actual.velocity().translational(), Vector3d{0., 0., 0.}, 1e-12));
  }
}

}  // namespace
}  // namespace automotive
}  // namespace drake
