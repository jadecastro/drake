#include "drake/automotive/idm_planner.h"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/extract_double.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace automotive {
namespace {

using std::pow;
using std::sqrt;

// Helper to set the derivatives to the desired 3-dimensional values.
static void SetDerivatives(AutoDiffXd* variable,
                           const Eigen::Vector3d& desired) {
  variable->derivatives() = desired;
}

static void SetDerivatives(double*, const Eigen::Vector3d&) {}

// Helper to check that the derivatives match the provided expected values.
static void CheckDerivatives(const AutoDiffXd& variable,
                             const Eigen::Vector3d& expected) {
  const double tol = 1e-9;
  EXPECT_TRUE(CompareMatrices(expected, variable.derivatives(), tol));
}

static void CheckDerivatives(const double&, const Eigen::Vector3d&) {}

// Helper to check only that the sign of the derivatives are as expected.
static void CheckDerivativeSigns(const AutoDiffXd& variable) {
  EXPECT_GT(0., variable.derivatives()(0));
  EXPECT_LT(0., variable.derivatives()(1));
  EXPECT_GT(0., variable.derivatives()(2));
}

static void CheckDerivativeSigns(const double&) {}

template <typename T>
class IdmPlannerTest : public ::testing::Test {
 protected:
  // Returns ∂/∂v(result) for the special case where accel_interaction is zero
  // and v = v_ref, where `v` is ego_velocity and `result` is the longitudinal
  // acceleration returned by IDM.
  double get_free_dadv() { return -p_.delta() * p_.a() / p_.v_ref(); }

  // Returns ∂/∂v(result) for the special case where v = d_dot = 0 and d = 1,
  // where `v` is ego_velocity, `d` is target_distance, `d_dot` is
  // target_distance_dot, and `result` is the longitudinal acceleration returned
  // by IDM.
  double get_dadv() { return -2 * p_.a() * p_.s_0() * p_.time_headway(); }

  // Returns ∂/∂d(result) for the special case where v = d_dot = 0 and d = 1,
  // where `v` is ego_velocity, `d` is target_distance, `d_dot` is
  // target_distance_dot, and `result` is the longitudinal acceleration returned
  // by IDM.
  double get_dadd() { return 2 * p_.a() * p_.s_0(); }

  const IdmPlannerParameters<T> params_;

 private:
  const IdmPlannerParameters<double> p_;
};

typedef ::testing::Types<double, AutoDiffXd> Implementations;
TYPED_TEST_CASE(IdmPlannerTest, Implementations);

// Set the initial states such that the agent and ego start at the headway
// distance, with the ego car closing in on the lead car.
TYPED_TEST(IdmPlannerTest, SameSpeedAtHeadwayDistance) {
  using T = TypeParam;

  T ego_velocity = this->params_.v_ref();
  T target_distance = this->params_.v_ref() * this->params_.time_headway();
  T target_distance_dot =
      -4 * sqrt(this->params_.a() * this->params_.b()) / this->params_.v_ref();

  // Set the derivatives as follows: {ego_velocity, target_distance,
  // target_distance_dot}.
  SetDerivatives(&ego_velocity, {1., 0., 0.});
  SetDerivatives(&target_distance, {0., 1., 0.});
  SetDerivatives(&target_distance_dot, {0., 0., 1.});

  const T result = IdmPlanner<T>::Evaluate(
      this->params_, ego_velocity, target_distance, target_distance_dot);

  // We expect acceleration to be close to zero.
  EXPECT_NEAR(0., ExtractDoubleOrThrow(result), 1e-2);

  // Since (closing_term + too_close_term) = 0 in the IDM equation with these
  // values, it then follows that:
  //
  // ∂/∂v(result) ∝ ∂/∂v(accel_free_road) and
  // ∂/∂d(result) = ∂/∂d_dot(result) = 0.
  //
  // where v is ego_velocity, d = target_distance, and d_dot is
  // target_distance_dot.
  CheckDerivatives(result, {this->get_free_dadv(), 0., 0.});
}

// Set the initial states such that the agent and ego start within the headway
// distance, both at the desired speed.
TYPED_TEST(IdmPlannerTest, SameSpeedBelowHeadwayDistance) {
  using T = TypeParam;

  T ego_velocity = this->params_.v_ref();
  T target_distance = 6.;
  T target_distance_dot = 0.;

  // Set the derivatives as follows: {ego_velocity, target_distance,
  // target_distance_dot}.
  SetDerivatives(&ego_velocity, {1., 0., 0.});
  SetDerivatives(&target_distance, {0., 1., 0.});
  SetDerivatives(&target_distance_dot, {0., 0., 1.});

  const T result = IdmPlanner<T>::Evaluate(
      this->params_, ego_velocity, target_distance, target_distance_dot);

  // We expect the car to decelerate.
  EXPECT_GE(0., ExtractDoubleOrThrow(result));

  // Expect ∂/∂v(result) < 0, ∂/∂d(result) > 0, and ∂/∂d_dot(result) < 0.
  CheckDerivativeSigns(result);
}

// Set the initial states such that the agent and ego start close together at
// different speeds.
TYPED_TEST(IdmPlannerTest, DifferentSpeedsBelowHeadwayDistance) {
  using T = TypeParam;

  T ego_velocity = 7.;
  T target_distance = 6.;
  T target_distance_dot = 3.;

  // Set the derivatives as follows: {ego_velocity, target_distance,
  // target_distance_dot}.
  SetDerivatives(&ego_velocity, {1., 0., 0.});
  SetDerivatives(&target_distance, {0., 1., 0.});
  SetDerivatives(&target_distance_dot, {0., 0., 1.});

  const T result = IdmPlanner<T>::Evaluate(
      this->params_, ego_velocity, target_distance, target_distance_dot);

  // We expect the car to decelerate.
  EXPECT_GE(0., ExtractDoubleOrThrow(result));

  // Expect ∂/∂v(result) < 0, ∂/∂d(result) > 0, and ∂/∂d_dot(result) < 0.
  CheckDerivativeSigns(result);
}

// Set the agent and ego sufficiently far apart from one another, with the ego
// car initially at the desired speed.
TYPED_TEST(IdmPlannerTest, EgoAtDesiredSpeed) {
  using T = TypeParam;

  T ego_velocity = this->params_.v_ref();
  T target_distance = 1e6;
  T target_distance_dot = this->params_.v_ref();

  // Set the derivatives as follows: {ego_velocity, target_distance,
  // target_distance_dot}.
  SetDerivatives(&ego_velocity, {1., 0., 0.});
  SetDerivatives(&target_distance, {0., 1., 0.});
  SetDerivatives(&target_distance_dot, {0., 0., 1.});

  const T result = IdmPlanner<T>::Evaluate(
      this->params_, ego_velocity, target_distance, target_distance_dot);

  // We expect acceleration to be close to zero.
  EXPECT_NEAR(0., ExtractDoubleOrThrow(result), 1e-2);

  // As above, since (closing_term + too_close_term) = 0 (to within tolerance),
  // the following result is immediate.
  CheckDerivatives(result, {this->get_free_dadv(), 0., 0.});
}

// Set the agent and ego sufficiently far apart from one another, with the ego
// car speed initially zero.
TYPED_TEST(IdmPlannerTest, EgoStartFromRest) {
  using T = TypeParam;

  T ego_velocity = 0.;
  T target_distance = 1;
  T target_distance_dot = 0.;

  // Set the derivatives as follows: {ego_velocity, target_distance,
  // target_distance_dot}.
  SetDerivatives(&ego_velocity, {1., 0., 0.});
  SetDerivatives(&target_distance, {0., 1., 0.});
  SetDerivatives(&target_distance_dot, {0., 0., 1.});

  const T result = IdmPlanner<T>::Evaluate(
      this->params_, ego_velocity, target_distance, target_distance_dot);

  // We expect the car to accelerate.
  EXPECT_LE(0., ExtractDoubleOrThrow(result));

  // Since v = d_dot = 0, then ∂/∂d_dot(result) = 0 and ∂/∂v(result) and
  // ∂/∂d(result) reduce to the simple relations provided above.
  CheckDerivatives(result, {this->get_dadv(), this->get_dadd(), 0.});
}

}  // namespace
}  // namespace automotive
}  // namespace drake
