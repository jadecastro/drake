#include "drake/systems/controllers/linear_model_predictive_controller.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/find_resource.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/gen/acrobot_state_vector.h"
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

// TODO(jadecastro) This is errantly using the acrobot dev model. Move to an
// executable within //drake/examples/acrobot/ and turn off visibility in that
// BUILD file.

std::unique_ptr<LinearModelPredictiveController<double>>
AcrobotBalancingMpcController(std::unique_ptr<AcrobotPlant<double>> acrobot,
                              double time_step, double time_horizon) {
  auto context = acrobot->CreateDefaultContext();

  // Set nominal torque to zero.
  context->FixInputPort(0, Vector1d::Constant(0.0));

  // Set nominal state to the upright fixed point.
  AcrobotStateVector<double>* x = dynamic_cast<AcrobotStateVector<double>*>(
      context->get_mutable_discrete_state()->get_mutable_vector());
  DRAKE_ASSERT(x != nullptr);
  x->set_theta1(M_PI);
  x->set_theta2(0.0);
  x->set_theta1dot(0.0);
  x->set_theta2dot(0.0);

  // Setup quadratic cost matrices (penalize position error 10x more than
  // velocity to roughly address difference in units, using sqrt(g/l) as the
  // time constant.
  Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
  Q(0, 0) = 10.;
  Q(1, 1) = 10.;
  Vector1d R = Vector1d::Constant(1.);

  return std::make_unique<LinearModelPredictiveController<double>>(
      std::move(acrobot), std::move(context), Q, R, time_step, time_horizon);
}

std::unique_ptr<systems::Diagram<double>> MakeControlledSystem(
    double time_step, double time_horizon) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf"),
      multibody::joints::kFixed, tree.get());

  DiagramBuilder<double> builder;
  auto acrobot = builder.AddSystem<AcrobotPlant>(time_step);  //NB. Try this in
                                                              //CT.
  acrobot->set_name("acrobot");
  drake::lcm::DrakeLcm lcm;
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");
  builder.Connect(acrobot->get_output_port(0), publisher->get_input_port(0));

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
  DRAKE_THROW_UNLESS(!result);
  return *result;
}

GTEST_TEST(TestMpc, TestAcrobotSimulation) {
  const double kTimeStep = 0.1;
  const double kTimeHorizon = 10.;

  auto diagram = MakeControlledSystem(kTimeStep, kTimeHorizon);
  Simulator<double> simulator(*diagram);
  const auto& acrobot = GetSystemByName("acrobot", *diagram);
  Context<double>& acrobot_context = diagram->GetMutableSubsystemContext(
      acrobot, simulator.get_mutable_context());

  // Set an initial condition near the upright fixed point.
  AcrobotStateVector<double>* x0 = dynamic_cast<AcrobotStateVector<double>*>(
      acrobot_context.get_mutable_discrete_state()->get_mutable_vector());
  DRAKE_DEMAND(x0 != nullptr);
  x0->set_theta1(M_PI + 0.1);
  x0->set_theta2(-.1);
  x0->set_theta1dot(0.0);
  x0->set_theta2dot(0.0);

  simulator.set_target_realtime_rate(1.);
  simulator.Initialize();
  simulator.StepTo(10.);
}

/*
GTEST_TEST(TestMPC, TestException) {
  Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
  Eigen::Vector2d B = Eigen::Vector2d::Zero();

  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Eigen::Matrix<double, 1, 1> R = Eigen::MatrixXd::Identity(1, 1);

  EXPECT_NO_THROW(LinearQuadraticRegulator(A, B, Q, R, N));
  EXPECT_NO_THROW(LinearQuadraticRegulator(A, B, Q, R));

  // R is not positive definite, should throw exception.
  EXPECT_THROW(LinearQuadraticRegulator(
        A, B, Q, Eigen::Matrix<double, 1, 1>::Zero()), std::runtime_error);
  EXPECT_THROW(LinearQuadraticRegulator(
        A, B, Q, Eigen::Matrix<double, 1, 1>::Zero(), N), std::runtime_error);
}

void TestLQRAgainstKnownSolution(
    double tolerance,
    const Eigen::Ref<const Eigen::MatrixXd>& K_known,
    const Eigen::Ref<const Eigen::MatrixXd>& S_known,
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N =
        Eigen::Matrix<double, 0, 0>::Zero()) {
  LinearQuadraticRegulatorResult result =
      LinearQuadraticRegulator(A, B, Q, R, N);
  EXPECT_TRUE(CompareMatrices(K_known, result.K, tolerance,
        MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(S_known, result.S, tolerance,
        MatrixCompareType::absolute));
}

void TestLQRLinearSystemAgainstKnownSolution(
    double tolerance,
    const LinearSystem<double>& sys,
    const Eigen::Ref<const Eigen::MatrixXd>& K_known,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N =
        Eigen::Matrix<double, 0, 0>::Zero()) {
  std::unique_ptr<LinearSystem<double>> linear_lqr =
      LinearQuadraticRegulator(sys, Q, R, N);

  int n = sys.A().rows();
  int m = sys.B().cols();
  EXPECT_TRUE(CompareMatrices(linear_lqr->A(),
                              Eigen::Matrix<double, 0, 0>::Zero(), tolerance,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(linear_lqr->B(),
                              Eigen::MatrixXd::Zero(0, n), tolerance,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(linear_lqr->C(),
                              Eigen::MatrixXd::Zero(m, 0), tolerance,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(linear_lqr->D(), -K_known, tolerance,
                              MatrixCompareType::absolute));
}

void TestLQRAffineSystemAgainstKnownSolution(
    double tolerance,
    const LinearSystem<double>& sys,
    const Eigen::Ref<const Eigen::MatrixXd>& K_known,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N =
        Eigen::Matrix<double, 0, 0>::Zero()) {
  int n = sys.A().rows();
  int m = sys.B().cols();

  auto context = sys.CreateDefaultContext();
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(m);

  context->FixInputPort(0, u0);
  context->get_mutable_continuous_state()->SetFromVector(x0);
  std::unique_ptr<AffineSystem<double>> lqr =
      LinearQuadraticRegulator(sys, *context, Q, R, N);

  EXPECT_TRUE(CompareMatrices(lqr->A(), Eigen::Matrix<double, 0, 0>::Zero(),
                              tolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->B(), Eigen::MatrixXd::Zero(0, n),
                              tolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->f0(), Eigen::Matrix<double, 0, 1>::Zero(),
                              tolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->C(), Eigen::MatrixXd::Zero(m, 0),
                              tolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->D(), -K_known,
                              tolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->y0(), u0 + K_known * x0,
                              tolerance, MatrixCompareType::absolute));
}

GTEST_TEST(TestMpc, DoubleIntegrator) {
  // Double integrator dynamics: qddot = u, where q is the position coordinate.
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  A << 0, 1, 0, 0;
  B << 0, 1;
  LinearSystem<double> sys(A, B, Eigen::Matrix<double, 0, 2>::Zero(),
                           Eigen::Matrix<double, 0, 1>::Zero());

  // Trivial cost:
  Eigen::Matrix2d Q;
  Eigen::Matrix<double, 1, 1> R;
  Q << 1, 0, 0, 1;
  R << 1;

  // Analytical solution
  Eigen::Matrix<double, 1, 2> K;
  K << 1, std::sqrt(3);

  Eigen::Matrix<double, 2, 2> S;
  S << std::sqrt(3), 1, 1, std::sqrt(3);

  double tol = 1e-10;

  TestLQRAgainstKnownSolution(tol, K, S, A, B, Q, R);

  // Test LinearSystem version of the LQR
  TestLQRLinearSystemAgainstKnownSolution(tol, sys, K, Q, R);

  // Call it as a generic System (by passing in a Context).
  // Should get the same result, but as an affine system.
  TestLQRAffineSystemAgainstKnownSolution(tol, sys, K, Q, R);

  // A different cost function with the same Q and R, and an extra N = [1; 0].
  Eigen::Vector2d N(1, 0);
  // Known solution
  K = Eigen::Vector2d(1, 1);
  S = Eigen::Matrix2d::Identity();
  TestLQRAgainstKnownSolution(tol, K, S, A, B, Q, R, N);

  // Test LinearSystem version of the LQR
  TestLQRLinearSystemAgainstKnownSolution(tol, sys, K, Q, R, N);

  // Test AffineSystem version of the LQR
  TestLQRAffineSystemAgainstKnownSolution(tol, sys, K, Q, R, N);
}
*/

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
