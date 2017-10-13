#include "drake/systems/primitives/time_varying_data.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/primitives/test/piecewise_linear_affine_test.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(TimeVaryingData, LinearConstructorWithEigenVectors) {
  TimeVaryingData ppt_data;
  MatrixData mat_data;
  std::tie(ppt_data, mat_data) = ExampleAffineTimeVaryingData();

  // Construct a linear TimeVaryingData struct from PiecewisePolynomials.
  const TimeVaryingData dut_from_vec =
      LinearTimeVaryingData(mat_data.Avec, mat_data.Bvec, mat_data.Cvec,
                            mat_data.Dvec, kDiscreteTimeStep);

  PiecewisePolynomialTrajectory zero_f0_trajectory(
      internal::MakeZeroedPiecewisePolynomial(
          ppt_data.A.get_piecewise_polynomial()));
  PiecewisePolynomialTrajectory zero_y0_trajectory(
      internal::MakeZeroedPiecewisePolynomial(
          ppt_data.C.get_piecewise_polynomial()));
  for (const double t :
       ppt_data.A.get_piecewise_polynomial().getSegmentTimes()) {
    EXPECT_TRUE(CompareMatrices(dut_from_vec.A.value(t), ppt_data.A.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_vec.B.value(t), ppt_data.B.value(t)));
    EXPECT_TRUE(
        CompareMatrices(dut_from_vec.f0.value(t), zero_f0_trajectory.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_vec.C.value(t), ppt_data.C.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_vec.D.value(t), ppt_data.D.value(t)));
    EXPECT_TRUE(
        CompareMatrices(dut_from_vec.y0.value(t), zero_y0_trajectory.value(t)));
  }
}

GTEST_TEST(TimeVaryingData, LinearConstructorWithPiecewisePolynomials) {
  TimeVaryingData ppt_data;
  MatrixData mat_data;
  std::tie(ppt_data, mat_data) = ExampleAffineTimeVaryingData();

  // Construct a linear TimeVaryingData struct from PiecewisePolynomials.
  const TimeVaryingData dut_from_pp =
      LinearTimeVaryingData(PiecewisePolynomial<double>::FirstOrderHold(
                                mat_data.times, mat_data.Avec),
                            PiecewisePolynomial<double>::FirstOrderHold(
                                mat_data.times, mat_data.Bvec),
                            PiecewisePolynomial<double>::FirstOrderHold(
                                mat_data.times, mat_data.Cvec),
                            PiecewisePolynomial<double>::FirstOrderHold(
                                mat_data.times, mat_data.Dvec));

  PiecewisePolynomialTrajectory zero_f0_trajectory(
      internal::MakeZeroedPiecewisePolynomial(
          ppt_data.A.get_piecewise_polynomial()));
  PiecewisePolynomialTrajectory zero_y0_trajectory(
      internal::MakeZeroedPiecewisePolynomial(
          ppt_data.C.get_piecewise_polynomial()));
  for (const double t :
       ppt_data.A.get_piecewise_polynomial().getSegmentTimes()) {
    EXPECT_TRUE(CompareMatrices(dut_from_pp.A.value(t), ppt_data.A.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_pp.B.value(t), ppt_data.B.value(t)));
    EXPECT_TRUE(
        CompareMatrices(dut_from_pp.f0.value(t), zero_f0_trajectory.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_pp.C.value(t), ppt_data.C.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_pp.D.value(t), ppt_data.D.value(t)));
    EXPECT_TRUE(
        CompareMatrices(dut_from_pp.y0.value(t), zero_y0_trajectory.value(t)));
  }
}

GTEST_TEST(TimeVaryingData, LinearConstructorWithTrajectories) {
  TimeVaryingData data;
  std::tie(data, std::ignore) = ExampleAffineTimeVaryingData();

  // Construct a linear TimeVaryingData struct from
  // PiecewisePolynomialTrajectories.
  const TimeVaryingData dut_from_ppt =
      LinearTimeVaryingData(data.A, data.B, data.C, data.D);

  PiecewisePolynomialTrajectory zero_f0_trajectory(
      internal::MakeZeroedPiecewisePolynomial(
          data.A.get_piecewise_polynomial()));
  PiecewisePolynomialTrajectory zero_y0_trajectory(
      internal::MakeZeroedPiecewisePolynomial(
          data.C.get_piecewise_polynomial()));
  for (const double t : data.A.get_piecewise_polynomial().getSegmentTimes()) {
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.A.value(t), data.A.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.B.value(t), data.B.value(t)));
    EXPECT_TRUE(
        CompareMatrices(dut_from_ppt.f0.value(t), zero_f0_trajectory.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.C.value(t), data.C.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.D.value(t), data.D.value(t)));
    EXPECT_TRUE(
        CompareMatrices(dut_from_ppt.y0.value(t), zero_y0_trajectory.value(t)));
  }
}

GTEST_TEST(TimeVaryingData, AffineConstructorWithTrajectories) {
  TimeVaryingData data;
  std::tie(data, std::ignore) = ExampleAffineTimeVaryingData();

  // Construct an affine TimeVaryingData struct from
  // PiecewisePolynomialTrajectories.
  const TimeVaryingData dut_from_ppt =
      TimeVaryingData(data.A, data.B, data.f0, data.C, data.D, data.y0);

  for (const double t : data.A.get_piecewise_polynomial().getSegmentTimes()) {
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.A.value(t), data.A.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.B.value(t), data.B.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.C.value(t), data.C.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.D.value(t), data.D.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.f0.value(t), data.f0.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.y0.value(t), data.y0.value(t)));
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
