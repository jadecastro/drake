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

template <typename T>
static std::unique_ptr<AutomotiveSimulator<T>> SetupAutomotiveSimulator(
    std::unique_ptr<const maliput::api::RoadGeometry> road_geometry,
    int num_traffic_cars, int requested_lane_index) {
  // Construct without LCM enabled as it is not yet AutoDiff-supported.
  auto simulator = std::make_unique<AutomotiveSimulator<T>>();
  auto simulator_road = simulator->SetRoadGeometry(std::move(road_geometry));
  auto dragway_road_geometry =
      dynamic_cast<const maliput::dragway::RoadGeometry*>(simulator_road);
  DRAKE_DEMAND(dragway_road_geometry != nullptr);

  // Add one ego car (simple car) situated within the desired lane.
  // TODO(jadecastro): Replace with MobilControlledSimpleCar once trajectory
  // optimization can be formulated as a MI problem.
  const auto segment = dragway_road_geometry->junction(0)->segment(0);
  DRAKE_DEMAND(requested_lane_index > 0 &&
               requested_lane_index < segment->num_lanes());
  const maliput::api::Lane* requested_lane =
      segment->lane(requested_lane_index);
  const auto& params_ego = CreateTrajectoryParamsForDragway(
      *dragway_road_geometry, requested_lane_index, T(0.), T(0.));
  simulator->AddIdmControlledCar("ego_car", std::get<0>(params_ego), T(0.),
                                 T(0.), requested_lane);

  // Add traffic cars (trajectory cars).
  // TODO(jadecastro): Make these IDM-controlled.
  for (int i{0}; i < num_traffic_cars; ++i) {
    const int lane_index_traffic = i % kNumDragwayLanes;
    const auto& params_traffic = CreateTrajectoryParamsForDragway(
        *dragway_road_geometry, lane_index_traffic, T(0.), T(0.));
    simulator->AddPriusTrajectoryCar("traffic_car_" + std::to_string(i),
                                     std::get<0>(params_traffic), T(0.), T(0.));
  }

  simulator->BuildAndInitialize();
  return std::move(simulator);
}

template <typename T>
static std::unique_ptr<systems::Simulator<T>> SetupSimulator(
    const systems::System<T>& plant,
    std::unique_ptr<systems::Context<T>> context, double simulator_time_step) {
  // Construct a simulator and intialize it.
  std::unique_ptr<systems::Simulator<T>> simulator =
      std::make_unique<systems::Simulator<T>>(plant);
  simulator->reset_context(std::move(context));
  simulator->set_target_realtime_rate(100.);
  simulator->get_mutable_integrator()->set_maximum_step_size(
      simulator_time_step);
  simulator->get_mutable_integrator()->set_requested_minimum_step_size(
      simulator_time_step);
  simulator->Initialize();

  return std::move(simulator);
}

template <typename T>
static std::pair<SimpleCarState<T>*, std::vector<TrajectoryCarState<T>*>>
GetAutomotiveSubsystemStates(const systems::System<T>& plant,
                             systems::Context<T>* context) {
  const auto diagram = dynamic_cast<const systems::Diagram<T>*>(&plant);
  DRAKE_DEMAND(diagram != nullptr);
  auto diagram_context = dynamic_cast<systems::DiagramContext<T>*>(context);
  DRAKE_DEMAND(diagram_context != nullptr);

  // Define the initial context values.  Parse the cars, one by one.
  // Note: context is dual purpose: we write these, and then over-write them
  // with a vector containing the endowed with the desired partial derivatives.
  std::vector<TrajectoryCarState<T>*> traffic_states{};
  SimpleCarState<T>* ego_state = nullptr;
  for (int i{0}; i < (int)diagram->GetSystems().size(); ++i) {
    systems::VectorBase<T>* state =
        diagram_context->GetMutableSubsystemContext(i)
            ->get_mutable_continuous_state_vector();
    if (state->size() == 0) continue;  // Skip any stateless systems.
    auto traffic_state = dynamic_cast<TrajectoryCarState<T>*>(state);
    ego_state = dynamic_cast<SimpleCarState<T>*>(state);
    if (traffic_state != nullptr) {
      traffic_states.emplace_back(traffic_state);
    } else if (ego_state != nullptr) {
      continue;
    } else {
      throw std::runtime_error(
          "System " + std::to_string(i) +
          " is stateful but does not match any automotive state vectors.");
    }
  }
  return std::make_pair(ego_state, traffic_states);
}

static void InitializeAutoDiffContext(systems::Context<AutoDiffXd>* context) {
  const auto& states = context->get_continuous_state_vector();
  const int num_states = states.size();
  Eigen::VectorXd context_vector(num_states + 1);  // time + states.
  for (int i{0}; i < num_states; ++i) {
    context_vector(i) = states.GetAtIndex(i).value();
  }
  context_vector(num_states) = 0.;  // initial time

  const auto autodiff_context_vector = math::initializeAutoDiff(context_vector);

  const AutoDiffXd time_autodiff = autodiff_context_vector(num_states);
  context->set_time(time_autodiff);
  const auto states_autodiff = autodiff_context_vector.segment(0, num_states);
  context->get_mutable_continuous_state()->SetFromVector(states_autodiff);
}

static AutoDiffXd EvalInstantaneousLossFunction(
    systems::Simulator<AutoDiffXd>& simulator) {
  using std::max;
  using std::pow;

  const auto& plant = simulator.get_system();
  auto context = simulator.get_mutable_context();

  SimpleCarState<AutoDiffXd>* ego_state = nullptr;
  std::vector<TrajectoryCarState<AutoDiffXd>*> traffic_states{};
  std::tie(ego_state, traffic_states) =
      GetAutomotiveSubsystemStates(plant, context);

  // TODO(jadecastro): Figure out some systematic way of referencing the cars.
  AutoDiffXd x_ego = ego_state->x();
  AutoDiffXd y_ego = ego_state->y();

  // The loss is the inverse sums of squares distance between the cars.
  AutoDiffXd result{-std::numeric_limits<AutoDiffXd>::infinity()};
  for (auto& traffic_state : traffic_states) {
    AutoDiffXd x_traffic = traffic_state->position();
    AutoDiffXd y_traffic{kLaneWidth / 2.};  // <--- bring actual value in via
                                            // system getter.
    SetZeroPartials(y_ego, &y_traffic);  // Use any state as the model vector
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

    std::cout << " Current time: " << simulator->get_context().get_time()
              << std::endl;
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
  auto automotive_simulator = SetupAutomotiveSimulator<double>(
      std::move(road_geometry), num_traffic_cars, 1 /* requested lane index */);
  const auto plant = automotive_simulator->GetDiagram().ToAutoDiffXd();
  auto context = plant->CreateDefaultContext();

  // Retrieve a vector of states corresponding to each car subsystem.
  SimpleCarState<AutoDiffXd>* ego_state = nullptr;
  std::vector<TrajectoryCarState<AutoDiffXd>*> traffic_states{};
  std::tie(ego_state, traffic_states) =
      GetAutomotiveSubsystemStates(*plant, context.get());
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

  InitializeAutoDiffContext(context.get());

  // Initialize the Drake Simulator.
  std::unique_ptr<systems::Simulator<AutoDiffXd>> simulator =
      SetupSimulator<AutoDiffXd>(*plant, std::move(context),
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
