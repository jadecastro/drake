#include "drake/automotive/driving_command_demux.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/automotive/gen/driving_command.h"
#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace automotive {
namespace {

class DrivingCommandDemuxTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<DrivingCommandDemux<double>>(2);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();
  }

  std::unique_ptr<DrivingCommandDemux<double>> dut_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
};

TEST_F(DrivingCommandDemuxTest, Basic) {
  // Confirm the shape.
  ASSERT_EQ(1, dut_->get_num_input_ports());
  ASSERT_EQ(4, dut_->get_input_port(0).size());
  ASSERT_EQ(2, dut_->get_num_output_ports());
  ASSERT_EQ(2, dut_->get_output_port(0).size());
  ASSERT_EQ(2, dut_->get_output_port(1).size());

  // Confirm that both outputs are DrivingCommands.
  const DrivingCommand<double>* output0 =
      dynamic_cast<const DrivingCommand<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, output0);
  const DrivingCommand<double>* output1 =
      dynamic_cast<const DrivingCommand<double>*>(output_->get_vector_data(1));
  ASSERT_NE(nullptr, output1);

  // Provide input data.
  auto input_vector =
      systems::BasicVector<double>::Make({43., 71., -37., -17.});
  context_->FixInputPort(0, std::move(input_vector));

  // Confirm output data.
  dut_->CalcOutput(*context_, output_.get());
  ASSERT_EQ(43., output0->steering_angle());
  ASSERT_EQ(71., output0->acceleration());
  ASSERT_EQ(-37., output1->steering_angle());
  ASSERT_EQ(-17., output1->acceleration());
}

TEST_F(DrivingCommandDemuxTest, IsStateless) {
  EXPECT_EQ(0, context_->get_continuous_state().size());
}

// Tests conversion to AutoDiffXd.
TEST_F(DrivingCommandDemuxTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*dut_, [&](const auto& converted) {
    EXPECT_EQ(1, converted.get_num_input_ports());
    EXPECT_EQ(2, converted.get_num_output_ports());

    EXPECT_EQ(4, converted.get_input_port(0).size());
    EXPECT_EQ(2, converted.get_output_port(0).size());
    EXPECT_EQ(2, converted.get_output_port(1).size());

    const auto context = converted.CreateDefaultContext();
    const auto output = converted.AllocateOutput();
    const DrivingCommand<AutoDiffXd>* driving_command_output =
        dynamic_cast<const DrivingCommand<AutoDiffXd>*>(
            output->get_vector_data(0));
    EXPECT_NE(nullptr, driving_command_output);
  }));
}

// Tests conversion to symbolic::Expression.
TEST_F(DrivingCommandDemuxTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*dut_, [&](const auto& converted) {
    EXPECT_EQ(1, converted.get_num_input_ports());
    EXPECT_EQ(2, converted.get_num_output_ports());

    EXPECT_EQ(4, converted.get_input_port(0).size());
    EXPECT_EQ(2, converted.get_output_port(0).size());
    EXPECT_EQ(2, converted.get_output_port(1).size());

    const auto context = converted.CreateDefaultContext();
    const auto output = converted.AllocateOutput();
    const DrivingCommand<symbolic::Expression>* driving_command_output =
        dynamic_cast<const DrivingCommand<symbolic::Expression>*>(
            output->get_vector_data(0));
    EXPECT_NE(nullptr, driving_command_output);
  }));
}

}  // namespace
}  // namespace automotive
}  // namespace drake
