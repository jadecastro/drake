#include "drake/systems/controllers/linear_model_predictive_controller.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/discrete_algebraic_riccati_equation.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {
namespace {

using math::DiscreteAlgebraicRiccatiEquation;

GTEST_TEST(TestTimeScheduledPiecewiseAffine, EquilibriumSystem) {
  // TODO Test fixture!!

  const int kNumSampleTimes = 10;
  const double kTimeStep = 0.1;

  std::vector<double> times(kNumSampleTimes);
  std::iota(std::begin(times), std::end(times), 0);  // Monitonically-increasing
                                                     // vector.
  const std::vector<Eigen::Matrix<double, -1, -1>> x0_vector(
      kNumSampleTimes, Eigen::Vector2d::Zero());
  auto x0 = std::make_unique<PiecewisePolynomialTrajectory>(
      PiecewisePolynomial<double>::FirstOrderHold(times, x0_vector));
  const std::vector<Eigen::Matrix<double, -1, -1>> u0_vector(kNumSampleTimes,
                                                             Vector1d::Zero());
  auto u0 = std::make_unique<PiecewisePolynomialTrajectory>(
      PiecewisePolynomial<double>::FirstOrderHold(times, u0_vector));

  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  A << 1, 0.1, 0, 1;
  B << 0.005, 0.1;
  const auto C = Eigen::Matrix<double, 2, 2>::Identity();
  const auto D = Eigen::Matrix<double, 2, 1>::Zero();

  std::unique_ptr<LinearSystem<double>> system =
      std::make_unique<LinearSystem<double>>(A, B, C, D, kTimeStep);
  const auto eq_system =
      std::make_unique<EquilibriumSystem<double>>(*system, kTimeStep);

  const auto eq_context = eq_system->CreateDefaultContext();
  eq_context->FixInputPort(0, u0->value(0.));
  eq_context->get_mutable_discrete_state(0)->SetFromVector(x0->value(0.));
  auto updates = eq_context->get_mutable_discrete_state();
  eq_system->CalcDiscreteVariableUpdates(*eq_context, updates);
  EXPECT_TRUE(CompareMatrices(updates->get_vector()->CopyToVector(),
                              A * x0->value(0.) + B * u0->value(0.)));
  EXPECT_TRUE(
      is_autodiffxd_convertible(*eq_system, [&](const auto& converted) {}));
}

class TestMpcWithDoubleIntegrator : public ::testing::Test {
 protected:
  void SetUp() override {
    const double kTimeStep = 0.1;     // discrete time step.
    const double kTimeHorizon = 10.;  // Time horizon.

    // A discrete approximation of a double integrator.
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    A << 1, 0.1, 0, 1;
    B << 0.005, 0.1;
    const auto C = Eigen::Matrix<double, 2, 2>::Identity();
    const auto D = Eigen::Matrix<double, 2, 1>::Zero();

    std::unique_ptr<LinearSystem<double>> system =
        std::make_unique<LinearSystem<double>>(A, B, C, D, kTimeStep);

    // Nominal, fixed reference condition.
    const Eigen::Vector2d x0 = Eigen::Vector2d::Zero();
    const Vector1d u0 = Vector1d::Zero();

    std::unique_ptr<Context<double>> system_context =
        system->CreateDefaultContext();
    system_context->FixInputPort(0, u0);
    system_context->get_mutable_discrete_state(0)->SetFromVector(x0);

    dut_.reset(new LinearModelPredictiveController<double>(
        std::move(system), std::move(system_context), Q_, R_, kTimeStep,
        kTimeHorizon));

    // Store another copy of the linear plant model.
    system_.reset(new LinearSystem<double>(A, B, C, D, kTimeStep));
  }

  // Cost matrices.
  const Eigen::Matrix2d Q_ = Eigen::Matrix2d::Identity();
  const Vector1d R_ = Vector1d::Constant(1.);

  std::unique_ptr<LinearModelPredictiveController<double>> dut_;
  std::unique_ptr<LinearSystem<double>> system_;
};

TEST_F(TestMpcWithDoubleIntegrator, TestAgainstInfiniteHorizonSolution) {
  const double kTolerance = 1e-5;

  const Eigen::Matrix2d A = system_->A();
  const Eigen::Matrix<double, 2, 1> B = system_->B();

  // Analytical solution to the LQR problem.
  const Eigen::Matrix2d S = DiscreteAlgebraicRiccatiEquation(A, B, Q_, R_);
  const Eigen::Matrix<double, 1, 2> K =
      -(R_ + B.transpose() * S * B).inverse() * (B.transpose() * S * A);

  const Eigen::Matrix<double, 2, 1> x0 = Eigen::Vector2d::Ones();

  auto context = dut_->CreateDefaultContext();
  context->FixInputPort(0, BasicVector<double>::Make(x0(0), x0(1)));
  std::unique_ptr<SystemOutput<double>> output = dut_->AllocateOutput(*context);

  dut_->CalcOutput(*context, output.get());

  EXPECT_TRUE(CompareMatrices(K * x0, output->get_vector_data(0)->get_value(),
                              kTolerance));
}

namespace {

// A discrete-time cubic polynomial system.
template <typename T>
class CubicPolynomialSystem final : public LeafSystem<T> {
 public:
  explicit CubicPolynomialSystem(double time_step)
      : LeafSystem<T>(
            SystemTypeTag<systems::controllers::CubicPolynomialSystem>{}),
        time_step_(time_step) {
    this->DeclareInputPort(systems::kVectorValued, 1);
    this->DeclareVectorOutputPort(BasicVector<T>(2),
                                  &CubicPolynomialSystem::OutputState);
    this->DeclareDiscreteState(2);
    this->DeclarePeriodicDiscreteUpdate(time_step);
  }

  template <typename U>
  CubicPolynomialSystem(const CubicPolynomialSystem<U>& other)
      : CubicPolynomialSystem(other.time_step_) {}

 private:
  template <typename>
  friend class CubicPolynomialSystem;

  // x1(k+1) = u(k)
  // x2(k+1) = -x1³(k)
  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>&,
      DiscreteValues<T>* next_state) const final {
    using std::pow;
    const T& x1 = context.get_discrete_state(0)->get_value()[0];
    const T& u = this->EvalVectorInput(context, 0)->get_value()[0];
    next_state->get_mutable_vector(0)->SetAtIndex(0, u);
    next_state->get_mutable_vector(0)->SetAtIndex(1, pow(x1, 3.));
  }

  void OutputState(const systems::Context<T>& context,
                   BasicVector<T>* output) const {
    output->set_value(context.get_discrete_state(0)->get_value());
  }

  // TODO(jadecastro) We know a discrete system of this format does not have
  // direct feedthrough, even though sparsity reports the opposite.  This is a
  // hack to patch in the correct result.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

  const double time_step_{0.};
};

template class CubicPolynomialSystem<double>;
template class CubicPolynomialSystem<AutoDiffXd>;

}  // namespace

class TestMpcWithCubicSystem : public ::testing::Test {
 protected:
  // TODO(jadecastro): Incorporate into Diagram -- I find myself using this
  // frequently.
  const System<double>& GetSystemByName(std::string name,
                                        const Diagram<double>& diagram) {
    const System<double>* result{nullptr};
    for (const System<double>* system : diagram.GetSystems()) {
      if (system->get_name() == name) result = system;
    }
    return *result;
  }

  void MakeTimeInvariantMpcController() {
    EXPECT_NE(nullptr, system_);
    auto context = system_->CreateDefaultContext();

    // Set nominal input to zero => equilibrium at zero state.
    context->FixInputPort(0, Vector1d::Constant(0.));

    // Set the nominal state.
    BasicVector<double>* x =
        context->get_mutable_discrete_state()->get_mutable_vector();
    x->SetFromVector(Eigen::Vector2d::Zero());  // Fixed point is zero.

    dut_.reset(new LinearModelPredictiveController<double>(
        std::move(system_), std::move(context), Q_, R_, time_step_,
        time_horizon_));
  }

  void MakeTimeVaryingMpcController() {
    EXPECT_NE(nullptr, system_);

    // Create trajectories for the states and inputs to force construction of
    // the time-varying MPC.
    const int kNumSampleTimes = (int)(time_horizon_ / time_step_ + 0.5);
    std::vector<double> times(kNumSampleTimes);
    std::vector<MatrixX<double>> x0_vector{};
    std::vector<MatrixX<double>> u0_vector{};

    Eigen::Vector2d x0_next_values;
    x0_next_values << 0., 0.;
    for (int i{0}; i < kNumSampleTimes; ++i) {
      times[i] = i * time_step_;
      const double u0_value = std::sin(times[i]);
      Eigen::Vector2d x0_values;
      x0_values << x0_next_values;

      x0_next_values << x0_values(0) + u0_value,
          x0_values(0) + std::pow(x0_values(0), 3.);
      // TODO Use update eqn?

      x0_vector.emplace_back(x0_values);
      u0_vector.emplace_back(Vector1d(u0_value));
    }
    auto x0_traj = std::make_unique<PiecewisePolynomialTrajectory>(
        PiecewisePolynomial<double>::FirstOrderHold(times, x0_vector));
    auto u0_traj = std::make_unique<PiecewisePolynomialTrajectory>(
        PiecewisePolynomial<double>::FirstOrderHold(times, u0_vector));
    x0_traj_ = x0_traj.get();
    dut_.reset(new LinearModelPredictiveController<double>(
        std::move(system_), std::move(x0_traj), std::move(u0_traj), Q_, R_,
        time_step_, time_horizon_));
  }

  void MakeControlledSystem(bool is_time_varying) {
    EXPECT_EQ(nullptr, diagram_);

    system_.reset(new CubicPolynomialSystem<double>(time_step_));
    EXPECT_FALSE(system_->HasAnyDirectFeedthrough());

    DiagramBuilder<double> builder;
    auto cubic_system = builder.AddSystem<CubicPolynomialSystem>(time_step_);
    cubic_system->set_name("cubic_system");

    if (is_time_varying) {
      MakeTimeVaryingMpcController();
    } else {
      MakeTimeInvariantMpcController();
    }
    EXPECT_NE(nullptr, dut_);
    auto controller = builder.AddSystem(std::move(dut_));
    controller->set_name("controller");

    builder.Connect(cubic_system->get_output_port(0),
                    controller->get_state_port());
    builder.Connect(controller->get_control_port(),
                    cubic_system->get_input_port(0));

    diagram_ = builder.Build();
  }

  void Simulate(double sim_time) {
    EXPECT_NE(nullptr, diagram_);
    EXPECT_EQ(nullptr, simulator_);

    simulator_.reset(new Simulator<double>(*diagram_));

    const auto& cubic_system = GetSystemByName("cubic_system", *diagram_);
    Context<double>& cubic_system_context =
        diagram_->GetMutableSubsystemContext(cubic_system,
                                             simulator_->get_mutable_context());
    BasicVector<double>* x0 =
        cubic_system_context.get_mutable_discrete_state()->get_mutable_vector();

    // Set an initial condition near the fixed point.
    x0->SetFromVector(x_init_);

    simulator_->set_target_realtime_rate(1.);
    simulator_->Initialize();
    simulator_->StepTo(sim_time);
  }

  const double time_step_ = 0.05;
  const double time_horizon_ = 3.;

  const Eigen::Vector2d x_init_ = 10. * Eigen::Vector2d::Ones();

  // Set up the quadratic cost matrices.
  const Eigen::Matrix2d Q_ = Eigen::Matrix2d::Identity();
  const Vector1d R_ = Vector1d::Constant(1.);

  PiecewisePolynomialTrajectory* x0_traj_{nullptr};

  std::unique_ptr<Simulator<double>> simulator_;

 private:
  std::unique_ptr<LinearModelPredictiveController<double>> dut_;  // <--make
                                                                  // local.
  std::unique_ptr<System<double>> system_;
  std::unique_ptr<Diagram<double>> diagram_;
};

TEST_F(TestMpcWithCubicSystem, TimeInvariantCase) {
  const double kTolerance = 1e-9;
  MakeControlledSystem(false /*is NOT time-varying */);
  Simulate(1.);

  // Result should be deadbeat; expect convergence to within a tiny tolerance in
  // one step.
  Eigen::Vector2d result =
      simulator_->get_mutable_context()->get_discrete_state(0)->get_value();
  EXPECT_TRUE(CompareMatrices(result, Eigen::Vector2d::Zero(), kTolerance));
}

TEST_F(TestMpcWithCubicSystem, TimeVaryingCase) {
  const double kTolerance = 1e-9;
  const double kSimTime = 20 * time_step_;

  MakeControlledSystem(true /* is time varying */);
  Simulate(kSimTime);

  // Result should be deadbeat; expect convergence to within a tiny tolerance in
  // one step.
  Eigen::Vector2d result =
      simulator_->get_mutable_context()->get_discrete_state(0)->get_value();
  EXPECT_TRUE(CompareMatrices(result, x0_traj_->value(kSimTime + time_step_),
                              kTolerance));
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
