#include "drake/systems/framework/hybrid_automaton_context.h"

#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {
namespace {

constexpr int kSize = 1;
constexpr int kNumSystems = 2;
constexpr double kTime = 12.0;

class HybridAutomatonContextTest : public ::testing::Test {
 protected:
  void SetUp() override {
    integrator0_.reset(new Integrator<double>(kSize));
    integrator1_.reset(new Integrator<double>(kSize));

    context_.reset(new HybridAutomatonContext<double>());
    context_->set_time(kTime);

    // Instantiate a new modal subsystem.
    const int mode_id = 27;
    //std::unique_ptr<ModalSubsystem<double>> mss0(new ModalSubsystem<double>(
    //    mode_id, integrator0_.get()));
    const ModalSubsystem<double> mss0 =
        ModalSubsystem<double>(mode_id, integrator0_.get());
    //const ModalSubsystem<double> mss0 =
    //    ModalSubsystem(mode_id, integrator0_.get());
    auto subcontext0 = integrator0_->CreateDefaultContext();
    auto suboutput0 = integrator0_->AllocateOutput(*subcontext0);
    context_->RegisterSubsystem(std::make_unique<ModalSubsystem<double>>(mss0),
                                std::move(subcontext0),
                                std::move(suboutput0));
    context_->MakeState();
    context_->SetModalState();

    // Explicitly export the ports.
    context_->ExportInput({0 /* export input port 0 */});
    context_->ExportOutput({0 /* export output port 0 */});

    // Set the continuous state.
    ContinuousState<double>* xc = context_->get_mutable_continuous_state();
    xc->get_mutable_vector()->SetAtIndex(0, 42.0);

    // Set the abstract state with the ID of the current modal subsystem.
    context_->SetModalState();
  }

  // Mocks up a descriptor that's sufficient to read a FreestandingInputPort
  // connected to @p context at @p index.
  static const BasicVector<double>* ReadVectorInputPort(
      const Context<double>& context, int index) {
    SystemPortDescriptor<double> descriptor(nullptr, kInputPort, index,
                                            kVectorValued, 0);
    return context.EvalVectorInput(nullptr, descriptor);
  }

  std::unique_ptr<HybridAutomatonContext<double>> context_;
  std::unique_ptr<Integrator<double>> integrator0_;
  std::unique_ptr<Integrator<double>> integrator1_;
};

// Tests that subsystems have outputs and contexts in the
// HybridAutomatonContext.
TEST_F(HybridAutomatonContextTest, RetrieveConstituents) {
  // The current active ModalSubsystem should be a leaf System.
  auto subcontext = context_->GetSubsystemContext();
  auto context = dynamic_cast<const LeafContext<double>*>(
      subcontext);
  EXPECT_TRUE(context != nullptr);

  auto output = dynamic_cast<const LeafSystemOutput<double>*>(
      context_->GetSubsystemOutput());
  EXPECT_TRUE(output != nullptr);

  auto modal_subsystem = dynamic_cast<const ModalSubsystem<double>*>(
      context_->GetModalSubsystem());
  EXPECT_TRUE(modal_subsystem != nullptr);

}

// Tests that the time writes through to the context of the active
// ModalSubsystem.
TEST_F(HybridAutomatonContextTest, Time) {
  context_->set_time(42.0);
  EXPECT_EQ(42.0, context_->get_time());
  for (int i = 0; i < kNumSystems; ++i) {
    EXPECT_EQ(42.0, context_->GetSubsystemContext()->get_time());
  }
}

// Tests that state variables appear in the diagram context, and write
// transparently through to the constituent system contexts.
TEST_F(HybridAutomatonContextTest, State) {
  // The integrator has a single continuous state variable.
  ContinuousState<double>* xc = context_->get_mutable_continuous_state();
  EXPECT_EQ(1, xc->size());
  EXPECT_EQ(0, xc->get_generalized_position().size());
  EXPECT_EQ(0, xc->get_generalized_velocity().size());
  EXPECT_EQ(1, xc->get_misc_continuous_state().size());

  DiscreteState<double>* xd = context_->get_mutable_discrete_state();
  EXPECT_EQ(0, xd->size());

  AbstractState* xa = context_->get_mutable_abstract_state();
  EXPECT_EQ(1, xa->size());

  // Changes to the continuous state write through to constituent system state.
  ContinuousState<double>* integrator0_xc =
      context_->GetMutableSubsystemContext()->get_mutable_continuous_state();
  EXPECT_EQ(42.0, integrator0_xc->get_vector().GetAtIndex(0));

  // Changes to constituent system state appears in the HA state.
  integrator0_xc->get_mutable_vector()->SetAtIndex(0, 1000.0);
  EXPECT_EQ(1000.0, xc->get_vector().GetAtIndex(0));
}

// Tests that the pointers to substates in the HybridAutomatonState are equal to
// the substates in the subsystem context.
TEST_F(HybridAutomatonContextTest, HybridAutomatonState) {
  auto hybrid_state = dynamic_cast<State<double>*>(
      context_->get_mutable_state());
  ASSERT_NE(nullptr, hybrid_state);
  EXPECT_EQ(context_->GetMutableSubsystemContext()->get_mutable_state(),
            hybrid_state);
}

// Tests that a change in the ModalSubsystem is reflected in the AbstractState.
TEST_F(HybridAutomatonContextTest, HybridAutomatonMode) {
  const int mode_id = 555;
  std::unique_ptr<ModalSubsystem<double>> mss(new ModalSubsystem<double>(
      mode_id, integrator1_.get()));
  //ModalSubsystem<double> mss1 = ModalSubsystem(mode_id, integrator1_.get());
  auto subcontext1 = integrator1_->CreateDefaultContext();
  auto suboutput1 = integrator1_->AllocateOutput(*subcontext1);
  context_->DeRegisterSubsystem();
  context_->RegisterSubsystem(std::move(mss), std::move(subcontext1),
                              std::move(suboutput1));
  context_->MakeState();
  context_->SetModalState();

  EXPECT_EQ(555, context_->template get_abstract_state<int>(0));
}

// Tests that input ports can be assigned to the HybridAutomatonContext and then
// retrieved.
TEST_F(HybridAutomatonContextTest, SetAndGetInputPorts) {
  ASSERT_EQ(1, context_->get_num_input_ports());
  context_->FixInputPort(0, BasicVector<double>::Make({128}));
  EXPECT_EQ(128, ReadVectorInputPort(*context_, 0)->get_value()[0]);
}

TEST_F(HybridAutomatonContextTest, Clone) {
  context_->FixInputPort(0, BasicVector<double>::Make({128}));

  std::unique_ptr<HybridAutomatonContext<double>> clone(
      dynamic_cast<HybridAutomatonContext<double>*>(
          context_->Clone().release()));
  ASSERT_TRUE(clone != nullptr);

  // Verify that the time was copied.
  EXPECT_EQ(kTime, clone->get_time());

  // Verify that the state was copied.
  const ContinuousState<double>* xc = clone->get_continuous_state();
  EXPECT_EQ(42.0, xc->get_vector().GetAtIndex(0));

  // Verify that the cloned input port contains the same data,
  // but with a different pointer.
  EXPECT_EQ(1, clone->get_num_input_ports());
  const BasicVector<double>* orig_port = ReadVectorInputPort(*context_, 0);
  const BasicVector<double>* clone_port = ReadVectorInputPort(*clone, 0);
  EXPECT_NE(orig_port, clone_port);
  EXPECT_TRUE(CompareMatrices(orig_port->get_value(), clone_port->get_value(),
                              1e-8, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace systems
}  // namespace drake
