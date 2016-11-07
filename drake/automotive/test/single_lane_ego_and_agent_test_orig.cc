#include "drake/automotive/single_lane_ego_and_agent.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace automotive {
namespace {

class SingleLaneEgoAndAgentTest : public ::testing::Test {
 protected:
  void SetUp() override {
    //dut_ = std::make_unique<SingleLaneEgoAndAgent<double>>(10.0, 0.0);
    //dut_.reset(new SingleLaneEgoAndAgent<double>(10.0, 0.0));
    context_ = dut_.CreateDefaultContext();
    output_ = dut_.AllocateOutput(*context_);
    derivatives_ = dut_.AllocateTimeDerivatives();
  }
  /*
  SingleLaneEgoAndAgentState<double>* continuous_state() {
    const auto result = context_->get_mutable_continuous_state_vector();
    //auto result = dynamic_cast<SingleLaneEgoAndAgentState<double>*>(
    //    context_->get_mutable_continuous_state_vector());
    //if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }
  */
  //std::unique_ptr<SingleLaneEgoAndAgent<double>> dut_;
  SingleLaneEgoAndAgent<double> dut_{10.0, 0.0};
                     //< The device under test.
  //std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(SingleLaneEgoAndAgentTest, Topology) {
  ASSERT_EQ(0, dut_.get_num_input_ports());
  ASSERT_EQ(2, dut_.get_num_output_ports());
  const auto& output_ego = dut_.get_output_ports().at(0);
  const auto& output_agent = dut_.get_output_ports().at(1);
  EXPECT_EQ(systems::kVectorValued, output_ego.get_data_type());
  EXPECT_EQ(systems::kVectorValued, output_agent.get_data_type());
  EXPECT_EQ(systems::kOutputPort, output_ego.get_face());
  EXPECT_EQ(systems::kOutputPort, output_agent.get_face());
  EXPECT_EQ(SingleLaneEgoAndAgentStateIndices::kNumCoordinates,
            output_ego.get_size() + output_agent.get_size());
  EXPECT_EQ(systems::kContinuousSampling, output_ego.get_sampling());
  EXPECT_EQ(systems::kContinuousSampling, output_agent.get_sampling());
}
  /*
TEST_F(SingleLaneEgoAndAgentTest, Output) {
  // Grab a pointer to where the EvalOutput results end up.
  const auto result = output_->get_vector_data(0);
  //const SingleLaneEgoAndAgentState<double>* const result =
  //    dynamic_cast<
  //  const SingleLaneEgoAndAgentState<double>*>(output_->get_vector_data(0));
  //ASSERT_NE(nullptr, result);

  // Starting state and output is all zeros.
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->x_e());

  EXPECT_EQ(0.0, result->v_e());
  EXPECT_EQ(0.0, result->x_a());
  EXPECT_EQ(0.0, result->v_a());

  // New state just propagates through.
  continuous_state()->set_x_e(1.0);
  continuous_state()->set_v_e(2.0);
  continuous_state()->set_x_a(3.0);
  continuous_state()->set_v_a(4.0);
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->x_e());
  EXPECT_EQ(2.0, result->v_e());
  EXPECT_EQ(3.0, result->x_a());
  EXPECT_EQ(4.0, result->v_a());

}
*/
TEST_F(SingleLaneEgoAndAgentTest, Derivatives) {
  // Declare a pointer to where the EvalTimeDerivatives results end up.
  //const auto result = derivatives_->get_mutable_vector();
  //const SingleLaneEgoAndAgentState<double>* const result =
  //    dynamic_cast<const SingleLaneEgoAndAgentState<double>*>(
  //        derivatives_->get_mutable_vector());
  //ASSERT_NE(nullptr, result);

  // Starting derivatives are almost all zeros, except for ego car velocity.
  dut_.SetDefaultState(context_.get());
  dut_.EvalTimeDerivatives(*context_, derivatives_.get());
  //EXPECT_EQ(10.0, derivatives_->get_vector().GetAtIndex(0));

  dut_.EvalOutput(*context_, output_.get());
  const systems::BasicVector<double>*
    output_vector = output_->get_vector_data(0);
  EXPECT_EQ(0.0, output_vector->GetAtIndex(0));

  //EXPECT_EQ(0.0, result->x_e());
  //EXPECT_NEAR(0.950617, result->v_e(), 1e-6);
  //EXPECT_EQ(0.0, result->x_a());
  //EXPECT_EQ(0.0, result->v_a());

  // TODO(rick.poyner@tri.global): exercise the dynamics at all, ever.
}

}  // namespace
}  // namespace automotive
}  // namespace drake
