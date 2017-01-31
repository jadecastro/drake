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

/// Set up an example HybridAutomaton, consisting of a bouncing ball plant.
constexpr int kModeIdBall = 0;
constexpr int kNumInports = 0;
constexpr int kNumOutports = 1;
constexpr int kStateDimension = 2;
constexpr double kCoeffOfRestitution = 0.7;

class BouncingBall : public HybridAutomaton<double> {
 public:
  explicit BouncingBall() {
    HybridAutomatonBuilder<double> builder;

    builder.set_num_expected_input_ports(kNumInports);
    builder.set_num_expected_output_ports(kNumOutports);

    std::vector<PortId> inports, outports;
    outports.push_back(0);

    // TODO(jadecastro): A cleaner way of accomplishing this (note the required
    // linker files)?
    std::unique_ptr<bouncing_ball::Ball<double>> ball;
    ball.reset(new bouncing_ball::Ball<double>);
    //bouncing_ball::Ball<double> ball;

    ModalSubsystem<double> mss = builder.AddModalSubsystem(
        std::move(ball), inports, outports, kModeIdBall);

    ball_subsystem_ = &mss;
    ball_ = ball_subsystem_->get_system();
    // TODO(jadecastro): Variable -> Variables?
    const std::vector<symbolic::Variable> x =
        ball_subsystem_->get_symbolic_continuous_states();

    // TODO(jadecastro): Decide on a cleaner way of auto-generate the named
    // getters from states, possibly leverage the .sh script for states.
    const symbolic::Expression y{x[0]};
    const symbolic::Expression ydot{x[1]};

    // Define the invariant to be the open half space y > 0.
    symbolic::Expression invariant_ball{y};
    builder.AddInvariant(ball_subsystem_, invariant_ball);

    ModeTransition<double> trans =
        builder.AddModeTransition(*ball_subsystem_);
    bounce_transition_ = &trans;

    // Define a reset map that negates the velocity and decrements it by a
    // factor of kCoeffOfRestitution.
    std::vector<symbolic::Expression> reset{y, -kCoeffOfRestitution * ydot};
    builder.AddReset(bounce_transition_, reset);
    std::cerr << " Reset size: " << bounce_transition_->get_reset().size()
              << std::endl;

    builder.BuildInto(this);
  }

  // Accessors.
  const System<double>* ball() { return ball_; }
  const ModalSubsystem<double>* ball_subsystem() { return ball_subsystem_; }

 private:
  ModalSubsystem<double>* ball_subsystem_ = nullptr;
  ModeTransition<double>* bounce_transition_ = nullptr;
  const System<double>* ball_ = nullptr;
};

class HybridAutomatonTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<BouncingBall>();

    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);

    input0_ = BasicVector<double>::Make({1});
    // Initialize the states.
    auto ball_xc = dut_->GetMutableSubsystemContext(context_.get())->
        get_mutable_continuous_state();
    ASSERT_TRUE(ball_xc != nullptr);
    ball_xc->get_mutable_vector()->SetAtIndex(0, 3);
  }

  void AttachInputs() {
    context_->SetInputPort(
        0, std::make_unique<FreestandingInputPort>(std::move(input0_)));
  }

  void SetInitialConditions(const Vector2<double>& x0) {
    systems::ContinuousState<double>* xc =
        dut_->GetMutableSubsystemContext(context_.get())->
        get_mutable_continuous_state();
    xc->SetFromVector(x0);
  }

  const System<double>* ball() { return dut_->ball(); }
  const ModalSubsystem<double>* ball_subsystem() {
    return dut_->ball_subsystem();
  }

  std::unique_ptr<BouncingBall> dut_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<BasicVector<double>> input0_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the diagram exports the correct topology.
TEST_F(HybridAutomatonTest, Topology) {
  ASSERT_EQ(kNumInports, dut_->get_num_input_ports());
  ASSERT_EQ(kNumOutports, dut_->get_num_output_ports());
  /*
    for (const auto& descriptor : dut_->get_input_ports()) {
    EXPECT_EQ(dut_.get(), descriptor.get_system());
    EXPECT_EQ(kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(kInputPort, descriptor.get_face());
    EXPECT_EQ(kSize, descriptor.get_size());
    }

    ASSERT_EQ(kSize, dut_->get_num_output_ports());
    for (const auto& descriptor : dut_->get_output_ports()) {
    EXPECT_EQ(dut_.get(), descriptor.get_system());
    EXPECT_EQ(kVectorValued, descriptor.get_data_type());
    EXPECT_EQ(kOutputPort, descriptor.get_face());
    EXPECT_EQ(kSize, descriptor.get_size());
    }
  */
  EXPECT_FALSE(dut_->has_any_direct_feedthrough());
}

  /*
  // TODO: what does the path buy us?
TEST_F(DutTest, Path) {
  const std::string path = adder0()->GetPath();
  EXPECT_EQ("::Unicode Snowman's Favorite Diagram!!1!☃!::adder0", path);
}
  */
TEST_F(HybridAutomatonTest, DoCalcTimeDerivatives) {
  //AttachInputs();
  std::unique_ptr<ContinuousState<double>> derivatives =
      dut_->AllocateTimeDerivatives();

  // Set the initial conditions.
  Vector2<double> x0;
  x0 << 10., 0.;  /* pos. [m], vel. [m/s] */
  SetInitialConditions(x0);

  dut_->DoCalcTimeDerivatives(*context_, derivatives.get());

  ASSERT_EQ(kStateDimension, derivatives->size());
  ASSERT_EQ(1, derivatives->get_generalized_position().size());
  ASSERT_EQ(1, derivatives->get_generalized_velocity().size());

  // Evaluate the derivative.
  EXPECT_EQ(0, derivatives->get_vector().GetAtIndex(0));
  EXPECT_EQ(-9.81, derivatives->get_vector().GetAtIndex(1));
}

// Tests that the same dut can be evaluated into the same output with
// different contexts interchangeably.
TEST_F(HybridAutomatonTest, CloneContext) {
  // Set the initial conditions.
  Vector2<double> x0;
  x0 << 10., 2.4;  /* pos. [m], vel. [m/s] */
  SetInitialConditions(x0);

  // Compute the output with the default inputs and sanity-check it.
  dut_->DoCalcOutput(*context_, output_.get());

  Vector2<double> expected_output0;
  expected_output0 << 10., 2.4;
  const BasicVector<double>* output0 = output_->get_vector_data(0);
  ASSERT_TRUE(output0 != nullptr);
  EXPECT_EQ(expected_output0[0], output0->get_value()[0]);
  EXPECT_EQ(expected_output0[1], output0->get_value()[1]);

  // Create a clone of the context and change the initial conditions.
  auto clone = context_->Clone();

  x0 << 6.7, 3.3;  /* pos. [m], vel. [m/s] */
  SetInitialConditions(x0);

  // Recompute the output and check the values.
  dut_->DoCalcOutput(*clone, output_.get());

  Vector2<double> expected_output1;
  expected_output1 << 6.7, 3.3;
  const BasicVector<double>* output1 = output_->get_vector_data(0);
  ASSERT_TRUE(output1 != nullptr);
  EXPECT_EQ(expected_output1[0], output1->get_value()[0]);
  EXPECT_EQ(expected_output1[1], output1->get_value()[1]);
}

// Tests that the invariant, guard, initial conditions and reset are evaluated
// correctly.
TEST_F(HybridAutomatonTest, EvaluateSymbolicQuantities) {
  // Set the initial conditions.
  Vector2<double> x0;
  x0 << 2.7, -2.;  /* pos. [m], vel. [m/s] */
  SetInitialConditions(x0);

  // Recompute the output and check the values.
  auto hybrid_context =
      dynamic_cast<HybridAutomatonContext<double>*>(context_.get());
  const std::vector<double> invariant_value =
      dut_->EvalInvariant(*hybrid_context);

  const double expected_invariant = 2.7;
  EXPECT_EQ(expected_invariant, invariant_value[0]);
}

// Tests the mode transition.
TEST_F(HybridAutomatonTest, ModeTransition) {
  // Set the initial conditions.
  Vector2<double> x0;
  x0 << 0., -2.;  /* pos. [m], vel. [m/s] */
  SetInitialConditions(x0);

  // Recompute the output and check the values.
  dut_->PerformTransition(0, context_.get());

  // Compute the output.
  dut_->DoCalcOutput(*context_, output_.get());

  Vector2<double> expected_output0;
  expected_output0 << 10., 0.;
  const BasicVector<double>* output0 = output_->get_vector_data(0);
  ASSERT_TRUE(output0 != nullptr);
  EXPECT_EQ(expected_output0[0], output0->get_value()[0]);
  EXPECT_EQ(expected_output0[1], output0->get_value()[1]);
}

}  // namespace
}  // namespace systems
}  // namespace drake
