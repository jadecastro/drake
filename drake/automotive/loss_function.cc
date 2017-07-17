#include <memory>

#include <gflags/gflags.h>

#include "drake/automotive/autodiff_utils.h"
#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/automotive_simulator_setup.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/common/call_matlab.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace automotive {
namespace {

static constexpr int kNumDragwayLanes = 2;
static constexpr double kLaneWidth = 2.;
static constexpr double kDragwayLength = 150.;

static AutoDiffXd EvalInstantaneousLossFunction(
    systems::Simulator<AutoDiffXd>& simulator) {
  using std::max;
  using std::pow;

  const auto& plant = simulator.get_system();
  auto context = simulator.get_mutable_context();

  SimpleCarState<AutoDiffXd>* ego_state = nullptr;
  std::vector<TrajectoryCarState<AutoDiffXd>*> traffic_states{};
  std::tie(ego_state, traffic_states) =
      sim_setup::GetAutomotiveSubsystemStates(plant, context);

  // TODO(jadecastro): Figure out some systematic way of referencing the cars.
  AutoDiffXd x_ego = ego_state->x();
  AutoDiffXd y_ego = ego_state->y();

  // The loss is the inverse sums of squares distance between the cars.
  AutoDiffXd result{-std::numeric_limits<AutoDiffXd>::infinity()};
  for (auto& traffic_state : traffic_states) {
    AutoDiffXd x_traffic = traffic_state->position();
    AutoDiffXd y_traffic{kLaneWidth / 2.};  // <--- bring actual value in via
                                            // system getter.
    autodiff::SetZeroPartials(y_ego, &y_traffic);  // Use any state as the model
                                                   // vector
    const AutoDiffXd car_loss =
        1. / (pow(x_ego - x_traffic, 2.) + pow(y_ego - y_traffic, 2.));
    result = max(result, car_loss);
  }
  return result;
}

static AutoDiffXd EvalLossFunction(
    std::unique_ptr<systems::Simulator<AutoDiffXd>> simulator,
    const AutoDiffXd& time_step, const AutoDiffXd& time_horizon) {
  using std::max;

  AutoDiffXd t = simulator->get_context().get_time();
  AutoDiffXd result{-std::numeric_limits<AutoDiffXd>::infinity()};
  while (t < time_horizon) {
    simulator->StepTo(t + time_step);
    t = simulator->get_context().get_time();

    // Evaluate the loss function at this time step.
    const AutoDiffXd running_loss = EvalInstantaneousLossFunction(*simulator);
    // Take the max loss encountered over all time instants.
    result = max(result, running_loss);

    // std::cout << " Current time: " << simulator->get_context().get_time()
    //         << std::endl;
  }
  return result;
}

int DoMain(void) {
  // Build the scenario.
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry =
      std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"dragway"}), kNumDragwayLanes,
          kDragwayLength, kLaneWidth, 0. /* shoulder width */,
          5. /* maximum height above road */,
          std::numeric_limits<double>::epsilon() /* linear tolerance */,
          std::numeric_limits<double>::epsilon() /* angular tolerance */);
  const int num_traffic_cars{kNumDragwayLanes};

  // Build an AutomotiveSimulator containing traffic cars and an ego car.
  auto automotive_simulator = sim_setup::SetupAutomotiveSimulator<double>(
      std::move(road_geometry), num_traffic_cars, 1 /* requested lane index */,
      kNumDragwayLanes);
  const auto plant = automotive_simulator->GetDiagram().ToAutoDiffXd();
  auto context = plant->CreateDefaultContext();

  // Retrieve a vector of states corresponding to each car subsystem.
  SimpleCarState<AutoDiffXd>* ego_state = nullptr;
  std::vector<TrajectoryCarState<AutoDiffXd>*> traffic_states{};
  std::tie(ego_state, traffic_states) =
      sim_setup::GetAutomotiveSubsystemStates(*plant, context.get());
  DRAKE_DEMAND(traffic_states.size() == num_traffic_cars);

  // Set all of the initial states by name.
  ego_state->set_x(50.);
  ego_state->set_y(kLaneWidth / 2.);
  ego_state->set_heading(0.);
  ego_state->set_velocity(5.);
  traffic_states[0]->set_position(60.);
  traffic_states[0]->set_speed(5.);
  traffic_states[1]->set_position(5.);
  traffic_states[1]->set_speed(5.);

  autodiff::InitializeAutoDiffContext(context.get());

  // Initialize the Drake Simulator.
  std::unique_ptr<systems::Simulator<AutoDiffXd>> simulator =
      sim_setup::SetupSimulator<AutoDiffXd>(*plant, std::move(context),
                                            0.01 /* simulator time step */);

  // Simulate forward up to the specified horizon, and evaluate the loss
  // function at the desired time steps along the way.
  AutoDiffXd loss =
      EvalLossFunction(std::move(simulator), 0.1 /* evaluation time step */,
                       5. /* time horizon */);
  // Report the returned loss value and its partial derivatives.
  std::cout << "\n Loss: " << loss << std::endl;
  std::cout << " Loss partials:\n" << loss.derivatives() << std::endl;

  // TODO(jadecastro): Use a python wrapper or our usual LCM tricks to grab the
  // data.
  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::automotive::DoMain();
}
