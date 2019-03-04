#include "drake/automotive/driving_command_adder.h"

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

class DrivingCommandAdderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<DrivingCommandAdder<double>>(2);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();
    input0_.reset(new DrivingCommand<double>());
    input1_.reset(new DrivingCommand<double>());
  }

  std::unique_ptr<DrivingCommandAdder<double>> dut_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<DrivingCommand<double>> input0_;
  std::unique_ptr<DrivingCommand<double>> input1_;
};

TEST_F(DrivingCommandAdderTest, Topology) {
  ASSERT_EQ(2, dut_->get_num_input_ports());
  for (int i = 0; i < 2; ++i) {
    const auto& descriptor = dut_->get_input_port(i);
    EXPECT_EQ(systems::kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(2, descriptor.size());
  }

  // TODO Check that the input port is a DrivingCommand.

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const systems::OutputPort<double>& output_port =
      static_cast<systems::LeafSystem<double>*>(dut_.get())->get_output_port(0);
  EXPECT_EQ(&output_port, &dut_->get_output_port());
  // Confirm that the output is a DrivingCommand.
  const DrivingCommand<double>* output =
      dynamic_cast<const DrivingCommand<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, output);
}

/*
// Tests conversion to AutoDiffXd.
TEST_F(DrivingCommandAdderTest, ToAutoDiff) {
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
TEST_F(DrivingCommandAdderTest, ToSymbolic) {
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
*/

}  // namespace
}  // namespace automotive
}  // namespace drake
