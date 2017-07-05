#include <memory>

#include <gflags/gflags.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/simple_car.h"
#include "drake/common/call_matlab.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/autodiff.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace automotive {
namespace {

static constexpr int kNumDragwayLanes = 2;
static constexpr double kLaneWidth = 2.;
static constexpr double kDragwayLength = 150.;

static void SetZeroPartials(const AutoDiffXd& model_value, AutoDiffXd* x) {
  const int num_partials = model_value.derivatives().size();
  auto& derivs = (*x).derivatives();
  if (derivs.size() == 0) {
    derivs.resize(num_partials);
    derivs.setZero();
  }
}

static AutoDiffXd EvalInstantaneousLossFunction(
    const VectorX<AutoDiffXd>& state_vector) {
  using std::pow;

  AutoDiffVecXd states(state_vector);

  // *** TODO: Figure out some systematic way of referencing the cars.
  AutoDiffXd x_ego{states(4)};
  AutoDiffXd y_ego{states(5)};
  AutoDiffXd x_traffic{states(0)};
  AutoDiffXd y_traffic{kLaneWidth / 2.};
  SetZeroPartials(states(0), &y_traffic);

  // The loss is the inverse sums of squares distance between the cars.
  // TODO(jadecastro): Generalize to n cars via the `min` operation.
  return {1. / (pow(x_ego - x_traffic, 2.) + pow(y_ego - y_traffic, 2.))};
}

static AutoDiffXd EvalLossFunction(
    std::unique_ptr<systems::Simulator<AutoDiffXd>> simulator,
    const AutoDiffXd& time_step, const AutoDiffXd& time_horizon) {
  using std::max;

  AutoDiffXd t = simulator->get_context().get_time();
  AutoDiffXd loss{-std::numeric_limits<AutoDiffXd>::infinity()};
  while (t < time_horizon) {
    simulator->StepTo(t + time_step);
    const auto current_state =
        simulator->get_context().get_continuous_state_vector().CopyToVector();
    t = simulator->get_context().get_time();

    // Evaluate the loss function at this time step.
    const AutoDiffXd running_loss =
        EvalInstantaneousLossFunction(current_state);
    loss = max(loss, running_loss);

    // Report the returned value and the partial derivatives for that value.
    std::cout << " Current state: " << current_state << std::endl;
    std::cout << " Current time: " << simulator->get_context().get_time()
              << std::endl;
    std::cout << " Loss: " << running_loss << std::endl;
    std::cout << " Loss partials: " << running_loss.derivatives() << std::endl;
  }
  return loss;
}

template <typename T>
static std::unique_ptr<AutomotiveSimulator<T>> SetupSimulator() {
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry =
      std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"dragway"}),
          kNumDragwayLanes, kDragwayLength, kLaneWidth, 0. /* shoulder width */,
          5. /* maximum_height */,
          std::numeric_limits<T>::epsilon() /* linear_tolerance */,
          std::numeric_limits<T>::epsilon() /* angular_tolerance */);

  // Construct without LCM enabled as it is not yet AutoDiff-supported.
  auto simulator = std::make_unique<AutomotiveSimulator<T>>();

  auto simulator_road = simulator->SetRoadGeometry(std::move(road_geometry));
  auto dragway_road_geometry =
      dynamic_cast<const maliput::dragway::RoadGeometry*>(simulator_road);

  const int lane_index = 1;
  const maliput::api::Lane* to_lane =
      dragway_road_geometry->junction(0)->segment(0)->lane(lane_index);

  // Add one ego car.
  const auto& params_ego = CreateTrajectoryParamsForDragway(
      *dragway_road_geometry, lane_index, T(0.), T(0.));
  simulator->AddIdmControlledCar("ego_car", std::get<0>(params_ego), T(0.),
                                 T(0.), to_lane);

  // Add two additional traffic cars.
  for (int i{0}; i < 2; ++i) {
    const int lane_index_traffic = i % kNumDragwayLanes;
    const auto& params_traffic = CreateTrajectoryParamsForDragway(
        *dragway_road_geometry, lane_index_traffic, T(0.), T(0.));
    simulator->AddPriusTrajectoryCar(
        "traffic_trajectory_car_" + std::to_string(i),
        std::get<0>(params_traffic), T(0.), T(0.));
  }
  return std::unique_ptr<AutomotiveSimulator<T>>(simulator.release());
}

int DoMain(void) {
  // Build the scenario.
  // TODO(jadecastro): Place this into a function
  auto automotive_simulator = SetupSimulator<double>();
  automotive_simulator->BuildAndInitialize();

  const auto& plant = automotive_simulator->GetDiagram();
  const auto& plant_autodiff = plant.ToAutoDiffXd();
  auto context_autodiff = plant_autodiff->CreateDefaultContext();

  const int num_states =
      context_autodiff->get_continuous_state_vector().CopyToVector().size();
  Eigen::VectorXd context_vector(num_states + 1);

  // Define the initial context values.
  context_vector << 0., 5., 5., 60., 5., 50., kLaneWidth / 2., 0., 5.;
  // Declare that the partials for each value are with respect to the initial
  // states and time.
  const auto autodiff_context_vector = math::initializeAutoDiff(context_vector);

  const AutoDiffXd time = autodiff_context_vector(0);
  context_autodiff->set_time(time);
  const auto states = autodiff_context_vector.segment(1, num_states);
  context_autodiff->get_mutable_continuous_state()->SetFromVector(states);

  // Generate a feasible trajetory starting at the specified initial condition.
  // TODO(jadecastro): Place this into a function
  std::unique_ptr<systems::Simulator<AutoDiffXd>> simulator =
      std::make_unique<systems::Simulator<AutoDiffXd>>(*plant_autodiff);
  simulator->reset_context(std::move(context_autodiff));
  simulator->set_target_realtime_rate(1.);
  simulator->get_mutable_integrator()->set_maximum_step_size(0.01);
  simulator->get_mutable_integrator()->set_requested_minimum_step_size(0.01);
  simulator->Initialize();

  AutoDiffXd loss =
      EvalLossFunction(std::move(simulator),
                       0.1 /* time step */, 5. /* time horizon */);

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::automotive::DoMain();
}
