#include "drake/systems/framework/hybrid_automaton.h"

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/hybrid_automaton_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/examples/bouncing_ball/ball.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {
namespace {

  /*
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<double>> data) {
  return std::make_unique<FreestandingInputPort>(std::move(data));
}
  */
/// Set up an example HybridAutomaton, consisting of a ball plant
/// and two integrators.

constexpr int kModeIdBall = 0;

class ExampleHybridAutomaton : public HybridAutomaton<double> {
 public:
  explicit ExampleHybridAutomaton(int size) {
    HybridAutomatonBuilder<double> builder;

    ball_subsystem_
      = builder.AddModalSubsystem(
          std::unique_ptr<bouncing_ball::Ball<double>>(), kModeIdBall);
    ball_ = this->get_subsystem(*ball_subsystem_);
    //symbolic::Expression x = get_symbolic_state_vector(ball_subsystem_);
    symbolic::Formula invariant_ball = symbolic::Formula::True();
    builder.AddInvariant(ball_subsystem_, invariant_ball);

    //ModeTransition* ball_to_ball_ =
    //    builder.AddModeTransition(*ball_subsystem_);
    //symbolic::Formula guard_formula_bounce = symbolic::Formula::True();

    //builder.BuildInto(this);
  }

  // Accessors.
  const System<double>* ball() { return ball_; }
  const ModalSubsystem* ball_subsystem() { return ball_subsystem_; }

 private:
  ModalSubsystem* ball_subsystem_ = nullptr;
  const System<double>* ball_ = nullptr;
};

/*
class HybridAutomatonTest : public ::testing::Test {
 protected:
  void SetUp() override {
    hybrid_aut_ = std::make_unique<ExampleHybridAutomaton>(kSize);

    context_ = hybrid_aut_->CreateDefaultContext();
    output_ = hybrid_aut_->AllocateOutput(*context_);

    //input0_ = BasicVector<double>::Make({1, 2, 4});
    //input1_ = BasicVector<double>::Make({8, 16, 32});
    //input2_ = BasicVector<double>::Make({64, 128, 256});

    // Initialize the integrator states.
    auto ball_xc = GetMutableContinuousState(ball_subsystem());
    ASSERT_TRUE(ball_xc != nullptr);
    ball_xc->get_mutable_vector()->SetAtIndex(0, 3);
    ball_xc->get_mutable_vector()->SetAtIndex(1, 9);
  }

  // Returns the continuous state of the given @p system.
  ContinuousState<double>* GetMutableContinuousState(
              const ExampleHybridAutomaton::ModalSubsystem* modal_subsystem) {
    return hybrid_aut_->
      GetMutableSubsystemState(context_.get(), modal_subsystem)
      ->get_mutable_continuous_state();
  }
*/
  /*
  void ExpectDefaultOutputs() {
    Eigen::Vector3d expected_output0;
    expected_output0 << 1 + 8 + 64, 2 + 16 + 128, 4 + 32 + 256;  // B

    Eigen::Vector3d expected_output1;
    expected_output1 << 1 + 8, 2 + 16, 4 + 32;  // A
    expected_output1 += expected_output0;       // A + B

    Eigen::Vector3d expected_output2;
    expected_output2 << 81, 243, 729;  // state of integrator1_

    const BasicVector<double>* output0 = output_->get_vector_data(0);
    ASSERT_TRUE(output0 != nullptr);
    EXPECT_EQ(expected_output0[0], output0->get_value()[0]);
    EXPECT_EQ(expected_output0[1], output0->get_value()[1]);
    EXPECT_EQ(expected_output0[2], output0->get_value()[2]);

    const BasicVector<double>* output1 = output_->get_vector_data(1);
    ASSERT_TRUE(output1 != nullptr);
    EXPECT_EQ(expected_output1[0], output1->get_value()[0]);
    EXPECT_EQ(expected_output1[1], output1->get_value()[1]);
    EXPECT_EQ(expected_output1[2], output1->get_value()[2]);

    const BasicVector<double>* output2 = output_->get_vector_data(2);
    ASSERT_TRUE(output2 != nullptr);
    EXPECT_EQ(expected_output2[0], output2->get_value()[0]);
    EXPECT_EQ(expected_output2[1], output2->get_value()[1]);
    EXPECT_EQ(expected_output2[2], output2->get_value()[2]);
  }

  void AttachInputs() {
    context_->SetInputPort(0, MakeInput(std::move(input0_)));
    context_->SetInputPort(1, MakeInput(std::move(input1_)));
    context_->SetInputPort(2, MakeInput(std::move(input2_)));
  }
  const System<double>* ball() { return hybrid_aut_->ball(); }
  const ExampleHybridAutomaton::ModalSubsystem* ball_subsystem() {
    return hybrid_aut_->ball_subsystem();
  }

  const int kSize = 1;

  std::unique_ptr<ExampleHybridAutomaton> hybrid_aut_;

  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};
  */
  /*
// Tests that the diagram exports the correct topology.
TEST_F(DiagramTest, Topology) {
  ASSERT_EQ(kSize, diagram_->get_num_input_ports());
  for (const auto& descriptor : diagram_->get_input_ports()) {
    EXPECT_EQ(diagram_.get(), descriptor.get_system());
    EXPECT_EQ(kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(kInputPort, descriptor.get_face());
    EXPECT_EQ(kSize, descriptor.get_size());
    EXPECT_EQ(kInheritedSampling, descriptor.get_sampling());
  }

  ASSERT_EQ(kSize, diagram_->get_num_output_ports());
  for (const auto& descriptor : diagram_->get_output_ports()) {
    EXPECT_EQ(diagram_.get(), descriptor.get_system());
    EXPECT_EQ(kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(kOutputPort, descriptor.get_face());
    EXPECT_EQ(kSize, descriptor.get_size());
  }

  // The adder output ports have inherited sampling.
  EXPECT_EQ(kInheritedSampling, diagram_->get_output_port(0).get_sampling());
  EXPECT_EQ(kInheritedSampling, diagram_->get_output_port(1).get_sampling());
  // The integrator output port has continuous sampling.
  EXPECT_EQ(kContinuousSampling, diagram_->get_output_port(2).get_sampling());

  // The diagram has direct feedthrough.
  EXPECT_TRUE(diagram_->has_any_direct_feedthrough());
}
  */

  /*
  // TODO: what does the path buy us?
TEST_F(DiagramTest, Path) {
  const std::string path = adder0()->GetPath();
  EXPECT_EQ("::Unicode Snowman's Favorite Diagram!!1!☃!::adder0", path);
}

// Tests that the diagram computes the correct sum.
TEST_F(DiagramTest, EvalOutput) {
  AttachInputs();
  diagram_->EvalOutput(*context_, output_.get());

  ASSERT_EQ(kSize, output_->get_num_ports());
  ExpectDefaultOutputs();
}

TEST_F(DiagramTest, EvalTimeDerivatives) {
  AttachInputs();
  std::unique_ptr<ContinuousState<double>> derivatives =
      diagram_->AllocateTimeDerivatives();

  diagram_->EvalTimeDerivatives(*context_, derivatives.get());

  ASSERT_EQ(6, derivatives->size());
  ASSERT_EQ(0, derivatives->get_generalized_position().size());
  ASSERT_EQ(0, derivatives->get_generalized_velocity().size());
  ASSERT_EQ(6, derivatives->get_misc_continuous_state().size());

  // The derivative of the first integrator is A.
  const ContinuousState<double>* integrator0_xcdot =
      diagram_->GetSubsystemDerivatives(*derivatives, integrator0());
  ASSERT_TRUE(integrator0_xcdot != nullptr);
  EXPECT_EQ(1 + 8, integrator0_xcdot->get_vector().GetAtIndex(0));
  EXPECT_EQ(2 + 16, integrator0_xcdot->get_vector().GetAtIndex(1));
  EXPECT_EQ(4 + 32, integrator0_xcdot->get_vector().GetAtIndex(2));

  // The derivative of the second integrator is the state of the first.
  const ContinuousState<double>* integrator1_xcdot =
      diagram_->GetSubsystemDerivatives(*derivatives, integrator1());
  ASSERT_TRUE(integrator1_xcdot != nullptr);
  EXPECT_EQ(3, integrator1_xcdot->get_vector().GetAtIndex(0));
  EXPECT_EQ(9, integrator1_xcdot->get_vector().GetAtIndex(1));
  EXPECT_EQ(27, integrator1_xcdot->get_vector().GetAtIndex(2));
}

// Tests that the same diagram can be evaluated into the same output with
// different contexts interchangeably.
TEST_F(DiagramTest, Clone) {
  context_->SetInputPort(0, MakeInput(std::move(input0_)));
  context_->SetInputPort(1, MakeInput(std::move(input1_)));
  context_->SetInputPort(2, MakeInput(std::move(input2_)));

  // Compute the output with the default inputs and sanity-check it.
  diagram_->EvalOutput(*context_, output_.get());
  ExpectDefaultOutputs();

  // Create a clone of the context and change an input.
  auto clone = context_->Clone();

  auto next_input_0 = std::make_unique<BasicVector<double>>(kSize);
  next_input_0->get_mutable_value() << 3, 6, 9;
  clone->SetInputPort(0, MakeInput(std::move(next_input_0)));

  // Recompute the output and check the values.
  diagram_->EvalOutput(*clone, output_.get());

  Eigen::Vector3d expected_output0;
  expected_output0 << 3 + 8 + 64, 6 + 16 + 128, 9 + 32 + 256;  // B
  const BasicVector<double>* output0 = output_->get_vector_data(0);
  ASSERT_TRUE(output0 != nullptr);
  EXPECT_EQ(expected_output0[0], output0->get_value()[0]);
  EXPECT_EQ(expected_output0[1], output0->get_value()[1]);
  EXPECT_EQ(expected_output0[2], output0->get_value()[2]);

  Eigen::Vector3d expected_output1;
  expected_output1 << 3 + 8, 6 + 16, 9 + 32;  // A
  expected_output1 += expected_output0;       // A + B
  const BasicVector<double>* output1 = output_->get_vector_data(1);
  ASSERT_TRUE(output1 != nullptr);
  EXPECT_EQ(expected_output1[0], output1->get_value()[0]);
  EXPECT_EQ(expected_output1[1], output1->get_value()[1]);
  EXPECT_EQ(expected_output1[2], output1->get_value()[2]);

  // Check that the context that was cloned is unaffected.
  diagram_->EvalOutput(*context_, output_.get());
  ExpectDefaultOutputs();
}

// Tests that, when asked for the state derivatives of Systems that are
// stateless, Diagram returns an empty state.
TEST_F(DiagramTest, DerivativesOfStatelessSystemAreEmpty) {
  std::unique_ptr<ContinuousState<double>> derivatives =
      diagram_->AllocateTimeDerivatives();
  EXPECT_EQ(0,
            diagram_->GetSubsystemDerivatives(*derivatives, adder0())->size());
}

class DiagramOfDiagramsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    DiagramBuilder<double> builder;
    subdiagram0_ = builder.AddSystem<ExampleDiagram>(kSize);
    subdiagram0_->set_name("subdiagram0");
    subdiagram1_ = builder.AddSystem<ExampleDiagram>(kSize);
    subdiagram1_->set_name("subdiagram1");

    // Hook up the two diagrams in portwise series.
    for (int i = 0; i < 3; i++) {
      builder.ExportInput(subdiagram0_->get_input_port(i));
      builder.Connect(subdiagram0_->get_output_port(i),
                      subdiagram1_->get_input_port(i));
      builder.ExportOutput(subdiagram1_->get_output_port(i));
    }

    diagram_ = builder.Build();
    diagram_->set_name("DiagramOfDiagrams");

    context_ = diagram_->CreateDefaultContext();
    output_ = diagram_->AllocateOutput(*context_);

    input0_ = BasicVector<double>::Make({8});
    input1_ = BasicVector<double>::Make({64});
    input2_ = BasicVector<double>::Make({512});

    context_->SetInputPort(0, MakeInput(std::move(input0_)));
    context_->SetInputPort(1, MakeInput(std::move(input1_)));
    context_->SetInputPort(2, MakeInput(std::move(input2_)));

    // Initialize the integrator states.
    Context<double>* d0_context =
        diagram_->GetMutableSubsystemContext(context_.get(), subdiagram0_);
    Context<double>* d1_context =
        diagram_->GetMutableSubsystemContext(context_.get(), subdiagram1_);

    State<double>* integrator0_x = subdiagram0_->GetMutableSubsystemState(
        d0_context, subdiagram0_->integrator0());
    integrator0_x->get_mutable_continuous_state()
        ->get_mutable_vector()->SetAtIndex(0, 3);

    State<double>* integrator1_x = subdiagram0_->GetMutableSubsystemState(
        d0_context, subdiagram0_->integrator1());
    integrator1_x->get_mutable_continuous_state()
        ->get_mutable_vector()->SetAtIndex(0, 9);

    State<double>* integrator2_x = subdiagram1_->GetMutableSubsystemState(
        d1_context, subdiagram1_->integrator0());
    integrator2_x->get_mutable_continuous_state()
        ->get_mutable_vector()->SetAtIndex(0, 27);

    State<double>* integrator3_x = subdiagram1_->GetMutableSubsystemState(
        d1_context, subdiagram1_->integrator1());
    integrator3_x->get_mutable_continuous_state()
        ->get_mutable_vector()->SetAtIndex(0, 81);
  }

  const int kSize = 1;

  std::unique_ptr<Diagram<double>> diagram_ = nullptr;
  ExampleDiagram* subdiagram0_ = nullptr;
  ExampleDiagram* subdiagram1_ = nullptr;

  std::unique_ptr<BasicVector<double>> input0_;
  std::unique_ptr<BasicVector<double>> input1_;
  std::unique_ptr<BasicVector<double>> input2_;

  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests the failure to build a hybrid automaton composed of other
// hybrid automata.
TEST_F(DiagramOfDiagramsTest, EvalOutput) {
  diagram_->EvalOutput(*context_, output_.get());
  // The outputs of subsystem0_ are:
  //   output0 = 8 + 64 + 512 = 584
  //   output1 = output0 + 8 + 64 = 656
  //   output2 = 9 (state of integrator1_)

  // So, the outputs of subsytem1_, and thus of the whole diagram, are:
  //   output0 = 584 + 656 + 9 = 1249
  //   output1 = output0 + 584 + 656 = 2489
  //   output2 = 81 (state of integrator1_)
  EXPECT_EQ(1249, output_->get_vector_data(0)->get_value().x());
  EXPECT_EQ(2489, output_->get_vector_data(1)->get_value().x());
  EXPECT_EQ(81, output_->get_vector_data(2)->get_value().x());
}

// PublishingSystem has an input port for a single double. It publishes that
// double to a function provided in the constructor.
class PublishingSystem : public LeafSystem<double> {
 public:
  explicit PublishingSystem(std::function<void(int)> callback)
      : callback_(callback) {
    this->DeclareInputPort(kVectorValued, 1, kInheritedSampling);
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {}

 protected:
  void DoPublish(const Context<double>& context) const override {
    CheckValidContext(context);
    callback_(this->EvalVectorInput(context, 0)->get_value()[0]);
  }

 private:
  std::function<void(int)> callback_;
};

  // TODO: Question: What does it mean to publish a variable?
// PublishNumberDiagram publishes a double provided to its constructor.
class PublishNumberDiagram : public Diagram<double> {
 public:
  explicit PublishNumberDiagram(double constant) : Diagram<double>() {
    DiagramBuilder<double> builder;

    constant_ =
        builder.AddSystem<ConstantVectorSource<double>>(Vector1d{constant});
    publisher_ =
        builder.AddSystem<PublishingSystem>([this](double v) { this->set(v); });

    builder.Connect(constant_->get_output_port(),
                    publisher_->get_input_port(0));
    builder.BuildInto(this);
  }

  double get() const { return published_value_; }

 private:
  void set(double value) { published_value_ = value; }

  ConstantVectorSource<double>* constant_ = nullptr;
  PublishingSystem* publisher_ = nullptr;
  double published_value_ = 0;
};

GTEST_TEST(DiagramPublishTest, Publish) {
  PublishNumberDiagram publishing_diagram(42.0);
  EXPECT_EQ(0, publishing_diagram.get());
  auto context = publishing_diagram.CreateDefaultContext();
  publishing_diagram.Publish(*context);
  EXPECT_EQ(42.0, publishing_diagram.get());
}
  */

}  // namespace
}  // namespace systems
}  // namespace drake
