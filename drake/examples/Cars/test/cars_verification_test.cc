#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/Cars/MultiCarSystem.h"

using drake::solvers::SolutionResult;

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

namespace drake {
namespace examples {
namespace cars {
namespace {

GTEST_TEST(CarsTrajectoryOptimization,
           CarsTrajectoryOptimizationTest) {
  auto p = make_shared<MultiCarSystem>();

  const int kTrajectoryTimeUpperBound = 6;

  const Eigen::Vector2d x0(0, 0);
  const Eigen::Vector2d xG(M_PI, 0);

  solvers::DreachFeasibility dreach(
      getNumInputs(*p), getNumStates(*p),
      kTrajectoryTimeUpperBound);

  dreach->AddStateConstraint(
      std::make_shared<LinearEqualityConstraint>(
          Eigen::Matrix2d::Identity(), x0), {0});
  dreach->AddStateConstraint(
      std::make_shared<LinearEqualityConstraint>(
          Eigen::Matrix2d::Identity(), xG), {num_time_samples - 1});

  dreach->AddDynamicConstraint(
      std::make_shared<
      drake::systems::SystemDirectCollocationConstraint<Pendulum>>(pendulum));

  SolutionResult result =
      dreach.SolveTraj(PiecewisePolynomialType());
  ASSERT_EQ(result, SolutionResult::kSolutionFound) << "Result is an Error";

  Eigen::MatrixXd inputs;
  Eigen::MatrixXd states;
  std::vector<double> times_out;

  drake.GetResultSamples(&inputs, &states, &times_out);
  EXPECT_TRUE(CompareMatrices(x0, states.col(0),
                              1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(xG, states.col(kNumTimeSamples - 1),
                              1e-10, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake
