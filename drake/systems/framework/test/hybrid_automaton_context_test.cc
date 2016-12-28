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

    context_.reset(new HybridAutomatonContext<double>(kNumSystems));
    context_->set_time(kTime);

    const int mode_0_ = 0;
    std::unique_ptr<ModalSubsystem<double>> mss0_(new ModalSubsystem<double>(
        mode_0_, integrator0_.get()));

    std::unique_ptr<AbstractValue> abstract_state =
        std::unique_ptr<AbstractValue>(
            new Value<ModalSubsystem<double>>(*mss0_.get()));
    std::vector<AbstractValue*> xm;
    xm.push_back(abstract_state.get());
    context_->
        set_abstract_state(std::make_unique<AbstractState>(std::move(xm)));
    std::cerr << " Abstract state: "
              << context_->get_abstract_state<
                 ModalSubsystem<double>>(0).get_mode_id() << std::endl;

    //AddModalSubsystem(*integrator0_, std::move(mss0_));
    auto subcontext0 = integrator1_->CreateDefaultContext();
    auto suboutput0 = integrator1_->AllocateOutput(*subcontext0);
    context_->AddModalSubsystem(std::move(mss0_),
                                std::move(subcontext0), std::move(suboutput0));

    const int mode_1_ = 1;
    std::unique_ptr<ModalSubsystem<double>> mss1_(new ModalSubsystem<double>(
        mode_1_, integrator1_.get()));
    //AddModalSubsystem(*integrator1_, std::move(mss1_));
    auto subcontext1 = integrator1_->CreateDefaultContext();
    auto suboutput1 = integrator1_->AllocateOutput(*subcontext1);
    context_->AddModalSubsystem(std::move(mss1_),
                                std::move(subcontext1), std::move(suboutput1));

    context_->ExportInput(0 /* either integrator's input */, 0 /* port 0 */);

    // Default the initial active ModalSubsystem to be integrator0_.
    context_->MakeState(0);
    ContinuousState<double>* xc = context_->get_mutable_continuous_state();
    xc->get_mutable_vector()->SetAtIndex(0, 42.0);
    std::cerr << " Continuous state: "
              << context_->
        get_continuous_state()->get_vector().GetAtIndex(0) << std::endl;
  }

  void
  AddModalSubsystem(const System<double>& sys,  // TODO: Nix `sys` argument or
                                                // put ModalSubsystem ctor
                                                // within the function.
                    std::unique_ptr<ModalSubsystem<double>> modal_subsystem) {
    //const System<double> sys = modal_subsystem->get_system();
    auto subcontext = sys.CreateDefaultContext();
    auto suboutput = sys.AllocateOutput(*subcontext);
    context_->AddModalSubsystem(std::move(modal_subsystem),
                                std::move(subcontext), std::move(suboutput));
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

// TODO(jadecastro): Repurpose as
TEST_F(HybridAutomatonContextTest, RetrieveConstituents) {
  // The current active ModalSubsystem should be a leaf System.
  auto subcontext = context_->GetSubsystemContext();
  auto context = dynamic_cast<const LeafContext<double>*>(
      subcontext);
  EXPECT_TRUE(context != nullptr);

  auto output = dynamic_cast<const LeafSystemOutput<double>*>(
      context_->GetSubsystemOutput());
  EXPECT_TRUE(output != nullptr);
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

/*
// Tests that state variables appear in the diagram context, and write
// transparently through to the constituent system contexts.
TEST_F(HybridAutomatonContextTest, State) {
  // Each integrator has a single continuous state variable.
  ContinuousState<double>* xc = context_->get_mutable_continuous_state();
  EXPECT_EQ(2, xc->size());
  EXPECT_EQ(0, xc->get_generalized_position().size());
  EXPECT_EQ(0, xc->get_generalized_velocity().size());
  EXPECT_EQ(2, xc->get_misc_continuous_state().size());

  // The zero-order hold has a difference state vector of length 1.
  DiscreteState<double>* xd = context_->get_mutable_discrete_state();
  EXPECT_EQ(1, xd->size());
  EXPECT_EQ(1, xd->get_discrete_state(0)->size());

  // Changes to the diagram state write through to constituent system states.
  // - Continuous
  ContinuousState<double>* integrator0_xc =
      context_->GetMutableSubsystemContext(2)->get_mutable_continuous_state();
  ContinuousState<double>* integrator1_xc =
      context_->GetMutableSubsystemContext(3)->get_mutable_continuous_state();
  EXPECT_EQ(42.0, integrator0_xc->get_vector().GetAtIndex(0));
  EXPECT_EQ(43.0, integrator1_xc->get_vector().GetAtIndex(0));
  // - Discrete
  DiscreteState<double>* hold_xd =
      context_->GetMutableSubsystemContext(4)->get_mutable_discrete_state();
  EXPECT_EQ(44.0, hold_xd->get_discrete_state(0)->GetAtIndex(0));

  // Changes to constituent system states appear in the diagram state.
  // - Continuous
  integrator1_xc->get_mutable_vector()->SetAtIndex(0, 1000.0);
  EXPECT_EQ(1000.0, xc->get_vector().GetAtIndex(1));
  // - Discrete
  hold_xd->get_mutable_discrete_state(0)->SetAtIndex(0, 1001.0);
  EXPECT_EQ(1001.0, xd->get_discrete_state(0)->GetAtIndex(0));
}

// Tests that the pointers to substates in the HybridAutomatonState are equal to
// the substates in the subsystem contexts.
TEST_F(HybridAutomatonContextTest, HybridAutomatonState) {
  auto diagram_state = dynamic_cast<HybridAutomatonState<double>*>(
      context_->get_mutable_state());
  ASSERT_NE(nullptr, diagram_state);
  for (int i = 0; i < kNumSystems; ++i) {
    EXPECT_EQ(context_->GetMutableSubsystemContext(i)->get_mutable_state(),
              diagram_state->get_mutable_substate(i));
  }
}

// Tests that input ports can be assigned to the HybridAutomatonContext and then
// retrieved.
TEST_F(HybridAutomatonContextTest, SetAndGetInputPorts) {
  ASSERT_EQ(2, context_->get_num_input_ports());
  context_->FixInputPort(0, BasicVector<double>::Make({128}));
  EXPECT_EQ(128, ReadVectorInputPort(*context_, 0)->get_value()[0]);
  EXPECT_EQ(256, ReadVectorInputPort(*context_, 1)->get_value()[0]);
}

TEST_F(HybridAutomatonContextTest, Clone) {
*/
  //context_->Connect({0 /* adder0_ */, 0 /* port 0 */},
  //                  {1 /* adder1_ */, 1 /* port 1 */});
/*
  AttachInputPorts();

  std::unique_ptr<HybridAutomatonContext<double>> clone(
      dynamic_cast<HybridAutomatonContext<double>*>(
          context_->Clone().release()));
  ASSERT_TRUE(clone != nullptr);

  // Verify that the time was copied.
  EXPECT_EQ(kTime, clone->get_time());

  // Verify that the state was copied.
  // - Continuous
  const ContinuousState<double>* xc = clone->get_continuous_state();
  EXPECT_EQ(42.0, xc->get_vector().GetAtIndex(0));
  EXPECT_EQ(43.0, xc->get_vector().GetAtIndex(1));
  // - Discrete
  const BasicVector<double>* xd_vec = clone->get_discrete_state(0);
  EXPECT_EQ(44.0, xd_vec->GetAtIndex(0));

  // Verify that the cloned input ports contain the same data,
  // but are different pointers.
  EXPECT_EQ(2, clone->get_num_input_ports());
  for (int i = 0; i < 2; ++i) {
    const BasicVector<double>* orig_port = ReadVectorInputPort(*context_, i);
    const BasicVector<double>* clone_port = ReadVectorInputPort(*clone, i);
    EXPECT_NE(orig_port, clone_port);
    EXPECT_TRUE(CompareMatrices(orig_port->get_value(), clone_port->get_value(),
                                1e-8, MatrixCompareType::absolute));
  }
}
*/

}  // namespace
}  // namespace systems
}  // namespace drake
