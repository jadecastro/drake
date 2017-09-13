#include <memory>

#include "drake/common/find_resource.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/gen/acrobot_state_vector.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_model_predictive_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

using systems::controllers::LinearModelPredictiveController;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::System;

std::unique_ptr<LinearModelPredictiveController<double>>
AcrobotBalancingMpcController(std::unique_ptr<AcrobotPlant<double>> acrobot,
                              double time_step, double time_horizon) {
  auto context = acrobot->CreateDefaultContext();

  // Set nominal torque to zero.
  const Eigen::VectorXd u0 = Vector1d::Constant(0.0);
  context->FixInputPort(0, u0);

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
const System<double>& GetSystemByName(
    std::string name, const Diagram<double>& diagram) {
  const System<double>* result{nullptr};
  for (const System<double>* system : diagram.GetSystems()) {
    if (system->get_name() == name) {
      DRAKE_THROW_UNLESS(!result);
      result = system;
    }
  }
  DRAKE_THROW_UNLESS(result);
  return *result;
}

int do_main(int, char**) {
  const double kTimeStep = 0.08;
  const double kTimeHorizon = 5.;
  const double kActualTimeStep = 0.08;

  const auto diagram =
      MakeControlledSystem(kTimeStep, kTimeHorizon, kActualTimeStep);
  systems::Simulator<double> simulator(*diagram);
  const auto& acrobot = GetSystemByName("acrobot", *diagram);
  systems::Context<double>& acrobot_context =
      diagram->GetMutableSubsystemContext(acrobot,
                                          simulator.get_mutable_context());

  // Set an initial condition near the upright fixed point.
  AcrobotStateVector<double>* x0 = (kActualTimeStep == 0.) ?
      dynamic_cast<AcrobotStateVector<double>*>(
          acrobot_context.get_mutable_continuous_state_vector()) :
      dynamic_cast<AcrobotStateVector<double>*>(
          acrobot_context.get_mutable_discrete_state()->get_mutable_vector());
  DRAKE_DEMAND(x0 != nullptr);
  x0->set_theta1(M_PI + 0.01);
  x0->set_theta2(-0.01);
  x0->set_theta1dot(0.0);
  x0->set_theta2dot(0.0);

  simulator.set_target_realtime_rate(1.);
  simulator.Initialize();
  simulator.StepTo(10.);
  std::cout << " DONE. " << std::endl;

  return false;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
  return drake::examples::acrobot::do_main(argc, argv);
}
