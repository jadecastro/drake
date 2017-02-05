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
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {
namespace {

constexpr int kSize = 1;
constexpr double kTime = 12.0;

class HybridAutomatonContextTest : public ::testing::Test {
 protected:
  void SetUp() override {

    integrator0_.reset(new Integrator<double>(kSize));
    integrator1_.reset(new Integrator<double>(kSize));

    context_.reset(new HybridAutomatonContext<double>());
    context_->set_time(kTime);

    // Instantiate a new modal subsystem. Implicitly, one input and one output
    // are exported as freestanding.
    const int mode_id = 42;
    const ModalSubsystem<double> mss0 = ModalSubsystem<double>(mode_id,
                                                               integrator0_);

    // Allocate the context and outputs.
    auto subcontext0 = mss0.get_system()->CreateDefaultContext();
    auto suboutput0 = mss0.get_system()->AllocateOutput(*subcontext0);

    // Register the system in the HA Context for the first time.
    context_->RegisterSubsystem(std::make_unique<ModalSubsystem<double>>(mss0),
                                std::move(subcontext0),
                                std::move(suboutput0));
    context_->MakeState();
    context_->SetModalState();

    // Set the continuous state.
    ContinuousState<double>* xc =
        context_->get_mutable_continuous_state();
    xc->get_mutable_vector()->SetAtIndex(0, 42.0);

    // Set the abstract state with the ID of the current modal subsystem.
    context_->SetModalState();
  }

  // Create a descriptor that reads a FreestandingInputPort connected to the
  // context.
  static const BasicVector<double>* ReadVectorInputPort(
      const Context<double>& context, int index) {
    InputPortDescriptor<double> descriptor(nullptr, index, kVectorValued, 0);
    return context.EvalVectorInput(nullptr, descriptor);
  }

  std::unique_ptr<HybridAutomatonContext<double>> context_;
  std::shared_ptr<Integrator<double>> integrator0_;
  std::shared_ptr<Integrator<double>> integrator1_;
};

// Tests that the SystemOutput, Context, and ModalSubsystem structures had been
// created within the HybridAutomatonContext.
TEST_F(HybridAutomatonContextTest, RetrieveSubsystemAttributes) {
  // The current active ModalSubsystem should be a leaf System.
  auto subcontext = context_->GetSubsystemContext();
  auto context = dynamic_cast<const LeafContext<double>*>(subcontext);
  EXPECT_TRUE(context != nullptr);

  auto output = dynamic_cast<const LeafSystemOutput<double>*>(
      context_->GetSubsystemOutput());
  EXPECT_TRUE(output != nullptr);

  auto modal_subsystem = dynamic_cast<const ModalSubsystem<double>*>(
      context_->GetModalSubsystem());
  EXPECT_TRUE(modal_subsystem != nullptr);
}

// Tests that the time writes through to the context of the active underlying
// System.
TEST_F(HybridAutomatonContextTest, Time) {
  context_->set_time(42.0);
  EXPECT_EQ(42.0, context_->get_time());
  EXPECT_EQ(42.0, context_->GetSubsystemContext()->get_time());
}

// Tests that state variables appear in the diagram context, and write
// transparently through to the constituent system contexts.
TEST_F(HybridAutomatonContextTest, State) {
  // The integrator has a single continuous state variable.
  ASSERT_NE(nullptr, context_->get_mutable_continuous_state());
  ContinuousState<double>* xc = context_->get_mutable_continuous_state();
  EXPECT_EQ(1, xc->size());
  EXPECT_EQ(0, xc->get_generalized_position().size());
  EXPECT_EQ(0, xc->get_generalized_velocity().size());
  EXPECT_EQ(1, xc->get_misc_continuous_state().size());

  DiscreteState<double>* xd = context_->get_mutable_discrete_state();
  EXPECT_EQ(0, xd->size());

  AbstractState* xa = context_->get_mutable_abstract_state();
  EXPECT_EQ(1, xa->size());

  // Changes to the continuous state appear in the leaf system state.
  ContinuousState<double>* integrator0_xc =
      context_->GetMutableSubsystemContext()->get_mutable_continuous_state();
  EXPECT_EQ(42.0, integrator0_xc->get_vector().GetAtIndex(0));

  // Changes to leaf system state appear in the HA state.
  integrator0_xc->get_mutable_vector()->SetAtIndex(0, 1000.);
  EXPECT_EQ(1000., xc->get_vector().GetAtIndex(0));
}

// Tests that the pointers to substates in the HybridAutomatonState are equal to
// the substates in the subsystem context.
TEST_F(HybridAutomatonContextTest, HybridAutomatonSubsystemState) {
  ASSERT_NE(nullptr, context_->get_mutable_state());
  auto hybrid_state = dynamic_cast<HybridAutomatonState<double>*>(
      context_->get_mutable_state());
  ASSERT_NE(nullptr, hybrid_state);
  EXPECT_EQ(context_->GetMutableSubsystemContext()->get_mutable_state(),
            hybrid_state->get_mutable_substate());
}

// Tests that a change in the ModalSubsystem is reflected in the AbstractState.
TEST_F(HybridAutomatonContextTest, DeRegisterAndRegisterSubsystem) {
  const int mode_id = 555;

  // Register a new system as its own ModalSubsystem.
  std::unique_ptr<ModalSubsystem<double>>
      mss(new ModalSubsystem<double>(mode_id, integrator1_));

  auto subcontext1 = mss->get_system()->CreateDefaultContext();
  auto suboutput1 = mss->get_system()->AllocateOutput(*subcontext1);

  //context_->DeRegisterSubsystem();

  // Check that the output and context are reset.
  //ASSERT_NE(nullptr, context_->GetSubsystemContext());
  //ASSERT_NE(nullptr, context_->GetSubsystemOutput());

  context_->RegisterSubsystem(std::move(mss), std::move(subcontext1),
                              std::move(suboutput1));
  context_->MakeState();
  context_->SetModalState();

  // Verify the id of the new ModalSubsystem.
  EXPECT_EQ(555, context_->template get_abstract_state<int>(0));
}

// Tests that input ports can be assigned to the HybridAutomatonContext and then
// retrieved.
TEST_F(HybridAutomatonContextTest, SetAndGetInputPorts) {
  ASSERT_EQ(1, context_->get_num_input_ports());
  context_->FixInputPort(0, BasicVector<double>::Make({128}));
  EXPECT_EQ(128, ReadVectorInputPort(*context_, 0)->get_value()[0]);
}

// Tests that a clone of the HybridAutomatonContext contains precisely the same
// data as the original.
TEST_F(HybridAutomatonContextTest, Clone) {
  context_->FixInputPort(0, BasicVector<double>::Make({128}));

  std::unique_ptr<HybridAutomatonContext<double>> clone(
      dynamic_cast<HybridAutomatonContext<double>*>(
          context_->Clone().release()));
  ASSERT_TRUE(clone != nullptr);

  // Verify that the time has been copied.
  EXPECT_EQ(kTime, clone->get_time());

  // Verify that the state has been copied.
  const ContinuousState<double>* xc = clone->get_continuous_state();
  EXPECT_EQ(42.0, xc->get_vector().GetAtIndex(0));

  // Verify that the cloned input port contains the same data, but with a
  // different pointer.
  EXPECT_EQ(1, clone->get_num_input_ports());
  const BasicVector<double>* orig_port = ReadVectorInputPort(*context_, 0);
  const BasicVector<double>* clone_port = ReadVectorInputPort(*clone, 0);
  EXPECT_NE(orig_port, clone_port);
  EXPECT_TRUE(CompareMatrices(orig_port->get_value(), clone_port->get_value(),
                              1e-8, MatrixCompareType::absolute));
}

// Tests that a cloned version of the state contains precisely the same data as
// the original.
TEST_F(HybridAutomatonContextTest, CloneState) {
  std::unique_ptr<State<double>> state = context_->CloneState();
  // Verify that the state was copied.
  const ContinuousState<double>* xc = (*state).get_continuous_state();
  EXPECT_EQ(42.0, xc->get_vector().GetAtIndex(0));
  //const DiscreteState<double>* xd = (*state).get_discrete_state();
  //EXPECT_EQ(44.0, xd->get_discrete_state(0)->GetAtIndex(0));
  //const AbstractState* xa = (*state).get_abstract_state();
  //EXPECT_EQ(42, xa->get_abstract_state(0).GetValue<int>());

  // Verify that the underlying type was preserved.
  EXPECT_NE(nullptr, dynamic_cast<HybridAutomatonState<double>*>(state.get()));
  // Verify that changes to the state do not write through to the original
  // context.
  (*state->get_mutable_continuous_state())[0] = 1024.;
  EXPECT_EQ(1024., (*state->get_continuous_state())[0]);
  EXPECT_EQ(42., (*context_->get_continuous_state())[0]);
}

// Tests the ability to specify symbolic expressions and evaluate them.
TEST_F(HybridAutomatonContextTest, Symbolic) {
  
}

}  // namespace
}  // namespace systems
}  // namespace drake
