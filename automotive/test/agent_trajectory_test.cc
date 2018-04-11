#include "drake/automotive/agent_trajectory.h"

#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace automotive {
namespace {

using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;
using test::CheckDerivatives;

void SetAcceleration(const TrajectoryCar<T>& car_dut,
                     const T& acceleration_input,
                     systems::Context<T>* context) {
  context->FixInputPort(car_dut.command_input().get_index(),
                        systems::BasicVector<T>::Make(acceleration_input));
}

// Empty curves are rejected.
GTEST_TEST(TrajectoryCarTest, StationaryTest) {
  const std::vector<Point2d> empty_waypoints{};
  const Curve2d empty_curve{empty_waypoints};
  EXPECT_THROW((TrajectoryCar<double>{empty_curve}), std::exception);
  CheckDerivatives(raw_pose->x(), Vector1d{0.});
}

}  // namespace
}  // namespace automotive
}  // namespace drake
