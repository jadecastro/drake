#include "drake/systems/framework/hybrid_automaton_context.h"

#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;
using std::move;
using std::shared_ptr;
using std::unique_ptr;

constexpr int kSize{1};
constexpr double kTime{12.};
constexpr double kZohSamplingPeriod{0.1};

class AbstractTestSource : public LeafSystem<double> {
 public:
  AbstractTestSource() {
    this->DeclareOutputPort(kVectorValued, 1);
  }

  unique_ptr<AbstractState> AllocateAbstractState() const override {
    std::vector<unique_ptr<AbstractValue>> values;
    values.push_back({PackValue<std::string>("I'm an abstract state.")});
    return make_unique<AbstractState>(move(values));
  }

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {
    DRAKE_ASSERT_VOID(System<double>::CheckValidOutput(output));
    DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));

    System<double>::GetMutableOutputVector(output, 0) =
        System<double>::CopyContinuousStateVector(context);
  }
};

// A system that is an amalgamation of continuous, discrete, and abstract
// states. Specifically, it is a diagram of an Integrator in series with a
// ZeroOrderHold element, in parallel with an AbstractTestSource system.
class ContinuousDiscreteAbstractSystem : public Diagram<double> {
 public:
  explicit ContinuousDiscreteAbstractSystem() {
    DiagramBuilder<double> builder;

    integrator_ = builder.template AddSystem<Integrator<double>>(kSize);
    zoh_ = builder.template AddSystem<ZeroOrderHold<double>>(
        kZohSamplingPeriod, kSize);
    abstract_ = builder.template AddSystem<AbstractTestSource>();

    builder.Connect(integrator_->get_output_port(0),
                    zoh_->get_input_port(0));

    builder.ExportInput(integrator_->get_input_port(0));
    builder.ExportOutput(zoh_->get_output_port(0));
    builder.ExportOutput(abstract_->get_output_port(0));

    builder.BuildInto(this);
  }

 private:
  Integrator<double>* integrator_ = nullptr;
  ZeroOrderHold<double>* zoh_ = nullptr;
  AbstractTestSource* abstract_ = nullptr;
};

class HybridAutomatonContextTest : public ::testing::Test {
 protected:
  void SetUp() override {

    integrator_.reset(new Integrator<double>(kSize));

    context_.reset(new HybridAutomatonContext<double>());
    context_->set_time(kTime);

    // Instantiate a new modal subsystem. Implicitly, one input and one output
    // are exported as freestanding.
    const int mode_id = 42;
    const ModalSubsystem<double> mss = ModalSubsystem<double>(mode_id,
                                                              integrator_);

    // Allocate the context and outputs.
    auto subcontext = mss.get_system()->CreateDefaultContext();
    auto suboutput = mss.get_system()->AllocateOutput(*subcontext);

    // Register the system in the HA Context for the first time.
    context_->RegisterSubsystem(make_unique<ModalSubsystem<double>>(mss),
                                move(subcontext), move(suboutput));
    context_->MakeState();

    // Set the abstract state with the ID of the current modal subsystem.
    context_->SetModalState();

    // Set the continuous state.
    ContinuousState<double>* xc = context_->get_mutable_continuous_state();
    xc->get_mutable_vector()->SetAtIndex(0, 42.0);
  }

  // Create a descriptor that reads a FreestandingInputPort connected to the
  // context.
  static const BasicVector<double>* ReadVectorInputPort(
      const Context<double>& context, int index) {
    InputPortDescriptor<double> descriptor(nullptr, index, kVectorValued, 0);
    return context.EvalVectorInput(nullptr, descriptor);
  }

  unique_ptr<HybridAutomatonContext<double>> context_;

  shared_ptr<Integrator<double>> integrator_;
};

class HybridAutomatonContextStateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    example_system_.reset(new ContinuousDiscreteAbstractSystem());
    example_system_->set_name("Bob");

    context_.reset(new HybridAutomatonContext<double>());
    context_->set_time(kTime);

    // Instantiate a new modal subsystem. Implicitly, one input and one output
    // are exported as freestanding.
    const int mode_id = 6;
    const ModalSubsystem<double> mss = ModalSubsystem<double>(mode_id,
                                                              example_system_);

    // Allocate the context and outputs.
    auto subcontext = mss.get_system()->CreateDefaultContext();
    auto suboutput = mss.get_system()->AllocateOutput(*subcontext);

    // Register the system in the HA Context for the first time.
    context_->RegisterSubsystem(make_unique<ModalSubsystem<double>>(mss),
                                move(subcontext), move(suboutput));
    context_->MakeState();

    // Set the abstract state with the ID of the current modal subsystem.
    context_->SetModalState();

    // Set the continuous and discrete states.
    ContinuousState<double>* xc = context_->get_mutable_continuous_state();
    xc->get_mutable_vector()->SetAtIndex(0, 27.0);
    DiscreteState<double>* xd = context_->get_mutable_discrete_state();
    xd->get_mutable_discrete_state(0)->SetAtIndex(0, 9.9);
  }

  // Create a descriptor that reads a FreestandingInputPort connected to the
  // context.
  static const BasicVector<double>* ReadVectorInputPort(
      const Context<double>& context, int index) {
    InputPortDescriptor<double> descriptor(nullptr, index, kVectorValued, 0);
    return context.EvalVectorInput(nullptr, descriptor);
  }

  unique_ptr<HybridAutomatonContext<double>> context_;

  shared_ptr<ContinuousDiscreteAbstractSystem> example_system_;
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
  context_->set_time(67.8);
  EXPECT_EQ(67.8, context_->get_time());
  EXPECT_EQ(67.8, context_->GetSubsystemContext()->get_time());
}

// Tests that we can correctly select among exported input and output ports
// those that are inputs and outputs of the HA.
TEST_F(HybridAutomatonContextTest, ModalSubsystemPortIds) {
  EXPECT_EQ(1, context_->GetModalSubsystem()->get_num_input_ports());
  EXPECT_EQ(1, context_->GetModalSubsystem()->get_num_output_ports());

  EXPECT_EQ(0, context_->GetModalSubsystem()->get_input_port_ids()[0]);
  EXPECT_EQ(0, context_->GetModalSubsystem()->get_output_port_ids()[0]);

  // Explicitly specify the ports for a non-trivial example via the constructor.
  shared_ptr<System<double>> example_system(
      new ContinuousDiscreteAbstractSystem());
  const ModalSubsystem<double> mss = ModalSubsystem<double>(0, example_system,
                                                            {0}, {1, 0});

  // Verify the number and identities of the ports have passed into the object.
  EXPECT_EQ(1, mss.get_num_input_ports());
  EXPECT_EQ(2, mss.get_num_output_ports());

  EXPECT_EQ(0, mss.get_input_port_ids()[0]);
  EXPECT_EQ(1, mss.get_output_port_ids()[0]);
  EXPECT_EQ(0, mss.get_output_port_ids()[1]);
}

// Tests the ability to specify symbolic expressions and evaluate them.
TEST_F(HybridAutomatonContextStateTest, ModalSubsystemSymbolicExpressions) {
  // Fetch the symbolic variables and mutable expressions for this context.
  symbolic::Variable xc =
      context_->GetModalSubsystem()->get_symbolic_continuous_states()[0];
  symbolic::Variable xd =
      context_->GetModalSubsystem()->get_symbolic_discrete_states_at(0)[0];
  std::vector<symbolic::Expression>* example_invariant =
      context_->GetModalSubsystem()->get_mutable_invariant();
  std::vector<symbolic::Expression>* example_ic =
      context_->GetModalSubsystem()->get_mutable_initial_conditions();

  // Define the symbols @p p (continuous-state) and @p q (discrete-state).
  const symbolic::Expression p{xc};
  const symbolic::Expression q{xd};

  // Check that the keys are indeed labeled "xc0" and "xd0".
  EXPECT_EQ("xc0", p.to_string());
  EXPECT_EQ("xd0", q.to_string());

  // Create invariants and initial conditions in terms of the symbols @p p and
  // @p q.
  (*example_invariant).push_back({pow(p, 2.) - cos(p) + tanh(q)});
  (*example_ic).push_back({pow(p, 3.)});
  (*example_ic).push_back({-p + 3.});
  (*example_ic).push_back({q});

  // Evaluate the expressions for a given assignment.
  symbolic::Environment x_env{{xc, 3.}, {xd, -2.}};
  EXPECT_EQ(pow(3., 2.) - cos(3.) + tanh(-2.),
            (*example_invariant)[0].Evaluate(x_env));
  EXPECT_EQ(pow(3., 3.), (*example_ic)[0].Evaluate(x_env));
  EXPECT_EQ(-3. + 3., (*example_ic)[1].Evaluate(x_env));
  EXPECT_EQ(-2., (*example_ic)[2].Evaluate(x_env));
}

// Tests the ability to create a clone of a ModalSubsystem.
TEST_F(HybridAutomatonContextStateTest, CloneModalSubsystem) {
  auto mss = context_->GetModalSubsystem();

  // Set fictitious invariants and initial conditions.
  const symbolic::Expression y{symbolic::Variable{"x"}};
  symbolic::Expression expression{y};
  (*mss->get_mutable_invariant()).push_back(y);
  (*mss->get_mutable_initial_conditions()).push_back(y);

  // Retrieve a clone.
  unique_ptr<ModalSubsystem<double>> mss_new = mss->Clone();

  // Verify that the data survives the clone.
  EXPECT_EQ(6, mss_new->get_mode_id());
  EXPECT_EQ("Bob", mss_new->get_system()->get_name());
  EXPECT_EQ(1, mss->get_invariant().size());
  EXPECT_EQ(1, mss->get_initial_conditions().size());
  EXPECT_EQ(1, mss_new->get_num_input_ports());
  EXPECT_EQ(2, mss_new->get_num_output_ports());
}

// Tests that state variables appear in the diagram context, and write
// transparently through to the constituent system contexts.
TEST_F(HybridAutomatonContextStateTest, State) {
  // The integrator has a single continuous state variable.
  ASSERT_NE(nullptr, context_->get_mutable_continuous_state());
  ContinuousState<double>* xc = context_->get_mutable_continuous_state();
  EXPECT_EQ(1, xc->size());
  EXPECT_EQ(0, xc->get_generalized_position().size());
  EXPECT_EQ(0, xc->get_generalized_velocity().size());
  EXPECT_EQ(1, xc->get_misc_continuous_state().size());

  DiscreteState<double>* xd = context_->get_mutable_discrete_state();
  EXPECT_EQ(1, xd->size());  /* expect one discrete state group */
  EXPECT_EQ(1, xd->get_discrete_state(0)->size());

  AbstractState* xa = context_->get_mutable_abstract_state();
  EXPECT_EQ(2, xa->size());  /* expect the abstract substate and mode id */

  // Check that the expected continuous state appears in the leaf system state.
  ContinuousState<double>* sub_xc =
      context_->GetMutableSubsystemContext()->get_mutable_continuous_state();
  EXPECT_EQ(27.0, (*sub_xc)[0]);

  // Check that the expected discrete state appears in the leaf system state.
  const BasicVector<double>* sub_xd =
      context_->GetMutableSubsystemContext()->get_discrete_state(0);
  EXPECT_EQ(9.9, (*sub_xd)[0]);

  // Check that the expected abstract state appears in the leaf system state.
  const AbstractState* sub_xa =
      context_->GetMutableSubsystemContext()->get_abstract_state();
  EXPECT_EQ("I'm an abstract state.",
            sub_xa->get_abstract_state(0).GetValue<std::string>());

  // Verify that the mode id is an element appended to the abstract substate.
  EXPECT_EQ(6, xa->get_abstract_state(1).GetValue<int>());

  // Check that changes to leaf system state appear in the HA state.
  sub_xc->get_mutable_vector()->SetAtIndex(0, 1000.);
  EXPECT_EQ(1000., (*xc)[0]);
}

// Tests that the pointers to substates in the HybridAutomatonState are equal to
// the substates in the subsystem context.
TEST_F(HybridAutomatonContextTest, SubsystemState) {
  ASSERT_NE(nullptr, context_->get_mutable_state());
  auto hybrid_state = dynamic_cast<HybridAutomatonState<double>*>(
      context_->get_mutable_state());
  ASSERT_NE(nullptr, hybrid_state);
  EXPECT_EQ(context_->GetMutableSubsystemContext()->get_mutable_state(),
            hybrid_state->get_mutable_substate());
}

// Tests that a change in the ModalSubsystem is reflected in the AbstractState.
TEST_F(HybridAutomatonContextTest, RegisterNewSubsystem) {
  const int mode_id = 555;

  shared_ptr<Integrator<double>>
      integrator1(new Integrator<double>(kSize));

  // Register a new system as its own ModalSubsystem.
  unique_ptr<ModalSubsystem<double>>
      mss(new ModalSubsystem<double>(mode_id, integrator1));

  auto subcontext1 = mss->get_system()->CreateDefaultContext();
  auto suboutput1 = mss->get_system()->AllocateOutput(*subcontext1);

  context_->RegisterSubsystem(move(mss), move(subcontext1),
                              move(suboutput1));
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

  unique_ptr<HybridAutomatonContext<double>> clone(
      dynamic_cast<HybridAutomatonContext<double>*>(
          context_->Clone().release()));
  ASSERT_TRUE(clone != nullptr);

  // Verify that the time has been copied.
  EXPECT_EQ(kTime, clone->get_time());

  // Verify that the state has been copied.
  const ContinuousState<double>* xc = clone->get_continuous_state();
  EXPECT_EQ(42.0, (*xc)[0]);

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
TEST_F(HybridAutomatonContextStateTest, CloneState) {
  unique_ptr<State<double>> state = context_->CloneState();

  // Verify that the state was copied and that all the elements survive intact.
  const ContinuousState<double>* xc = state->get_continuous_state();
  EXPECT_EQ(27.0, (*xc)[0]);
  const DiscreteState<double>* xd = state->get_discrete_state();
  EXPECT_EQ(9.9, (*xd->get_discrete_state(0))[0]);
  const AbstractState* xa = state->get_abstract_state();
  EXPECT_EQ("I'm an abstract state.",
            xa->get_abstract_state(0).GetValue<std::string>());

  // Check that the modal state also survives.
  EXPECT_EQ(6, xa->get_abstract_state(1).GetValue<int>());

  // Verify that the underlying type was preserved.
  EXPECT_NE(nullptr, dynamic_cast<HybridAutomatonState<double>*>(state.get()));

  // Verify that changes to the state do not transfer to the original context.
  (*state->get_mutable_continuous_state())[0] = 10.;
  EXPECT_EQ(10., (*state->get_continuous_state())[0]);
  EXPECT_EQ(27., (*context_->get_continuous_state())[0]);
}

}  // namespace
}  // namespace systems
}  // namespace drake
