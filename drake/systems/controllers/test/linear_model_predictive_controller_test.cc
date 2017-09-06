#include "drake/systems/controllers/linear_model_predictive_controller.h"

//#include <gflags/gflags.h>
#include <gtest/gtest.h>

//#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {
namespace {

using examples::acrobot::AcrobotPlant;
using examples::acrobot::AcrobotStateVector;

GTEST_TEST(TestMpc, DiscreteDoubleIntegrator) {
  // Double integrator dynamics: qddot = u, where q is the position coordinate.
  const double T = 0.1;  // discrete time step.
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  A << 1, 0.1, 0, 1;
  B << 0.005, 0.1;
  auto sys = std::make_unique<LinearSystem<double>>(
      A, B, Eigen::Matrix<double, 2, 2>::Identity(),
      Eigen::Matrix<double, 2, 1>::Zero(), T);

  // Trivial cost:
  Eigen::Matrix2d Q;
  Eigen::Matrix<double, 1, 1> R;
  Q << 1, 0, 0, 1;
  R << 1;

  int n = sys->A().rows();
  int m = sys->B().cols();
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(m);

  auto context = sys->CreateDefaultContext();
  context->FixInputPort(0, u0);
  context->get_mutable_discrete_state(0)->SetFromVector(x0);

  auto mpc = std::make_unique<LinearModelPredictiveController<double>>(
      std::move(sys), std::move(context), Q, R, T, 10.);
  //EXPECT_TRUE(CompareMatrices(K_known, result.K, tolerance,
  //                            MatrixCompareType::absolute));

}

namespace {

template <typename T>
class CubicPolynomialSystem final : public systems::LeafSystem<T> {
 public:
  explicit CubicPolynomialSystem(double time_step)
      : time_step_(time_step) {
    this->DeclareDiscreteState(1);
    this->DeclarePeriodicDiscreteUpdate(time_step);
  }

  // Scalar-converting copy constructor.
  template <typename U>
  explicit CubicPolynomialSystem(const CubicPolynomialSystem<U>& system)
      : CubicPolynomialSystem(system.timestep()) {}

  double time_step() const { return time_step_; }

 private:
  // x(k+1) = x³(k) + u(k)
  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>&,
      DiscreteValues<T>* discrete_state) const final {
    using std::pow;
    const T& x = context.get_discrete_state(0)->get_value()[0];
    const T& u = this->EvalVectorInput(context, 0)->get_value()[0];
    discrete_state->get_mutable_vector(0)->SetAtIndex(0, pow(x, 3.) + u);
  }

  const double time_step_{0.};
};

}  // namespace

// TODO(jadecastro) This is errantly using the acrobot dev model. Move this code
// into an executable within //drake/examples/acrobot/ and turn off global
// visibility in that BUILD file.

std::unique_ptr<LinearModelPredictiveController<double>>
MpcController(std::unique_ptr<CubicPolynomialSystem<double>> system,
              double time_step, double time_horizon) {
  auto context = system->CreateDefaultContext();

  // Set nominal input to zero.
  const Eigen::VectorXd u0 = Vector1d::Constant(0.);
  context->FixInputPort(0, u0);

  // Set the nominal state.
  BasicVector<double>* x =
      context->get_mutable_discrete_state()->get_mutable_vector();
  x->set_theta1(M_PI);

  // Set up the quadratic cost matrices.
  const Vector1d Q = Vector1d::Constant(10.);
  const Vector1d R = Vector1d::Constant(1.);

  //return std::make_unique<LinearModelPredictiveController<double>>(
  //    std::move(acrobot), std::move(context), Q, R, time_step, time_horizon);

  // Create trivial trajectories for the states and inputs to force construction
  // of the time-varying MPC, as an initial spike test.
  const int kNumSampleTimes = (int)(time_horizon / time_step + 0.5);
  std::vector<double> times(kNumSampleTimes);
  std::vector<MatrixX<double>> x0_vector{};
  std::vector<MatrixX<double>> u0_vector{};
  for (int i{0}; i < kNumSampleTimes; ++i) {
    times[i] = i * time_step;
    x0_vector.emplace_back(x->CopyToVector());
    u0_vector.emplace_back(u0);
  }
  auto x0_traj = std::make_unique<PiecewisePolynomialTrajectory>(
      PiecewisePolynomial<double>::ZeroOrderHold(times, x0_vector));
  auto u0_traj = std::make_unique<PiecewisePolynomialTrajectory>(
      PiecewisePolynomial<double>::ZeroOrderHold(times, u0_vector));
  return std::make_unique<LinearModelPredictiveController<double>>(
      std::move(acrobot), std::move(x0_traj), std::move(u0_traj), Q, R,
      time_step, time_horizon);
}

std::unique_ptr<systems::Diagram<double>> MakeControlledSystem(
    double time_step, double time_horizon, double actual_time_step) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf"),
      multibody::joints::kFixed, tree.get());

  DiagramBuilder<double> builder;
  auto acrobot = builder.AddSystem<AcrobotPlant>(actual_time_step);
  acrobot->set_name("acrobot");

  // TODO(jadecastro) Reinsert the publisher in the DT case once we've resolved
  // why LCM is segfaulting.
  if (actual_time_step == 0.) {
    drake::lcm::DrakeLcm lcm;
    auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
    publisher->set_name("publisher");
    builder.Connect(acrobot->get_output_port(0), publisher->get_input_port(0));
  }

  auto controller =
      builder.AddSystem(AcrobotBalancingMpcController(
          std::make_unique<AcrobotPlant<double>>(time_step), time_step,
          time_horizon));
  controller->set_name("controller");
  builder.Connect(acrobot->get_output_port(0), controller->get_state_port());
  builder.Connect(controller->get_control_port(), acrobot->get_input_port(0));

  return builder.Build();
}

// TODO(jadecastro): Incorporate into Diagram -- I find myself using this
// frequently.
const systems::System<double>& GetSystemByName(
    std::string name, const Diagram<double>& diagram) {
  const systems::System<double>* result{nullptr};
  for (const systems::System<double>* system : diagram.GetSystems()) {
    if (system->get_name() == name) {
      DRAKE_THROW_UNLESS(!result);
      result = system;
    }
  }
  DRAKE_THROW_UNLESS(result);
  return *result;
}

//int do_main(int argc, char* argv[]) {
//  gflags::ParseCommandLineFlags(&argc, &argv, true);
GTEST_TEST(TestMpc, TestAcrobotSimulation) {
  const double kTimeStep = 0.08;
  const double kTimeHorizon = 5.;
  const double kActualTimeStep = 0.08;

  auto diagram = MakeControlledSystem(kTimeStep, kTimeHorizon, kActualTimeStep);
  Simulator<double> simulator(*diagram);
  const auto& acrobot = GetSystemByName("acrobot", *diagram);
  Context<double>& acrobot_context = diagram->GetMutableSubsystemContext(
      acrobot, simulator.get_mutable_context());

  // Set an initial condition near the upright fixed point.
  AcrobotStateVector<double>* x0;
  if (kActualTimeStep == 0.) {
    x0 = dynamic_cast<AcrobotStateVector<double>*>(
        acrobot_context.get_mutable_continuous_state_vector());
  } else {
    x0 = dynamic_cast<AcrobotStateVector<double>*>(
        acrobot_context.get_mutable_discrete_state()->get_mutable_vector());
  }
  DRAKE_DEMAND(x0 != nullptr);
  x0->set_theta1(M_PI + 0.1);
  x0->set_theta2(-.1);
  x0->set_theta1dot(0.0);
  x0->set_theta2dot(0.0);

  simulator.set_target_realtime_rate(1.);
  simulator.Initialize();
  simulator.StepTo(10.);
  std::cout << " DONE. " << std::endl;
}

//int do_main(int argc, char* argv[]) {
//  gflags::ParseCommandLineFlags(&argc, &argv, true);
GTEST_TEST(TestMpc, TestTimeVaryingAcrobotSimulation) {
  const double kTimeStep = 0.08;
  const double kTimeHorizon = 5.;
  const double kActualTimeStep = 0.08;

  auto diagram = MakeControlledSystem(kTimeStep, kTimeHorizon, kActualTimeStep);
  Simulator<double> simulator(*diagram);
  const auto& acrobot = GetSystemByName("acrobot", *diagram);
  Context<double>& acrobot_context = diagram->GetMutableSubsystemContext(
      acrobot, simulator.get_mutable_context());

  // Set an initial condition near the upright fixed point.
  AcrobotStateVector<double>* x0;
  if (kActualTimeStep == 0.) {
    x0 = dynamic_cast<AcrobotStateVector<double>*>(
        acrobot_context.get_mutable_continuous_state_vector());
  } else {
    x0 = dynamic_cast<AcrobotStateVector<double>*>(
        acrobot_context.get_mutable_discrete_state()->get_mutable_vector());
  }
  DRAKE_DEMAND(x0 != nullptr);
  x0->set_theta1(M_PI + 0.1);
  x0->set_theta2(-.1);
  x0->set_theta1dot(0.0);
  x0->set_theta2dot(0.0);

  simulator.set_target_realtime_rate(1.);
  simulator.Initialize();
  simulator.StepTo(10.);
  std::cout << " DONE. " << std::endl;
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake

//int main(int argc, char* argv[]) {
//  return drake::systems::controllers::do_main(argc, argv);
//}
