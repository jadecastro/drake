#include "drake/automotive/linear_car.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace automotive {
namespace {

class LinearCarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new LinearCar<double>);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  LinearCarState<double>* continuous_state() {
    auto result = dynamic_cast<LinearCarState<double>*>(
        context_->get_mutable_continuous_state_vector());
    if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }

  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(LinearCarTest, Topology) {
  ASSERT_EQ(0, dut_->get_num_input_ports());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_ports().at(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(systems::kOutputPort, output_descriptor.get_face());
  EXPECT_EQ(LinearCarStateIndices::kNumCoordinates,
            output_descriptor.get_size());
  EXPECT_EQ(systems::kContinuousSampling, output_descriptor.get_sampling());
}

TEST_F(LinearCarTest, Output) {
  // Grab a pointer to where the EvalOutput results end up.
  const LinearCarState<double>* const result =
      dynamic_cast<
    const LinearCarState<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Starting state and output is all zeros.
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->x());
  EXPECT_EQ(0.0, result->v());

  // New state just propagates through.
  continuous_state()->set_x(1.0);
  continuous_state()->set_v(2.0);
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->x());
  EXPECT_EQ(2.0, result->v());
}

TEST_F(LinearCarTest, Derivatives) {
  // Grab a pointer to where the EvalTimeDerivatives results end up.
  const LinearCarState<double>* const result =
      dynamic_cast<const LinearCarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Starting derivatives are almost all zeros, except for ego car velocity.
  dut_->EvalTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->x());
  EXPECT_EQ(0.0, result->v());

  // Test at a nontrivial initial condition.
  continuous_state()->set_x(4.2);
  continuous_state()->set_v(5.3);
  dut_->EvalTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(5.3, result->x());
  EXPECT_EQ(0.0, result->v());

  // TODO(rick.poyner@tri.global): exercise the dynamics at all, ever.
}

}  // namespace
}  // namespace automotive
}  // namespace drake
