#include "drake/automotive/automotive_simulator_setup.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/automotive/autodiff_utils.h"
#include "drake/common/call_matlab.h"

namespace drake {
namespace automotive {
namespace sim_setup {
namespace {

static constexpr int kNumDragwayLanes = 2;
static constexpr double kLaneWidth = 2.;
static constexpr double kDragwayLength = 150.;

// TODO(jadecastro): Remove analysis::Simulator().

// 
GTEST_TEST(AutomotiveSimulatorSetupTest, TestAutoDiff) {
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
  //
  // Note: The order in which each entry appears below corresponds to the
  // ordering of the partial derivatives.
  // TODO(jadecastro): Smarter way of identifying these derivatives, such as
  // creating a std::map whose indices are assigned right after the call to
  // InitializeAutoDiffContext().
  traffic_states[0]->set_position(60.);
  traffic_states[0]->set_speed(5.);
  traffic_states[1]->set_position(5.);
  traffic_states[1]->set_speed(5.);
  ego_state->set_x(52.);
  ego_state->set_y(kLaneWidth / 2.);
  ego_state->set_heading(0.);
  ego_state->set_velocity(5.);

  autodiff::InitializeAutoDiffContext(context.get());

  std::cout << "\n x: " << ego_state->x() << std::endl;
  std::cout << " x partials:\n" << ego_state->x().derivatives() << std::endl;

  // Initialize the Drake Simulator.
  double sim_time_step{0.01};
  std::unique_ptr<systems::Simulator<AutoDiffXd>> simulator =
      sim_setup::SetupSimulator<AutoDiffXd>(*plant, std::move(context),
                                            sim_time_step);

  // Simulate and extract the state.
  AutoDiffXd t = simulator->get_context().get_time();
  simulator->StepTo(t + sim_time_step);  // simulate for the shortest possible
                                         // time.
  t = simulator->get_context().get_time();
  std::tie(ego_state, traffic_states) =
      sim_setup::GetAutomotiveSubsystemStates(*plant,
                                              simulator->get_mutable_context());

  // Report the resulting partial derivatives.
  std::cout << "\n Time: " << t << std::endl;
  std::cout << " Time partials:\n" << t.derivatives() << std::endl;

  std::cout << "\n position car 0: " << traffic_states[0]->position()
            << std::endl;
  std::cout << " position car 0 partials:\n"
            << traffic_states[0]->position().derivatives() << std::endl;
  std::cout << "\n speed car 0: " << traffic_states[0]->speed()
            << std::endl;
  std::cout << " speed car 0 partials:\n"
            << traffic_states[0]->speed().derivatives() << std::endl;

  std::cout << "\n position car 1: " << traffic_states[1]->position()
            << std::endl;
  std::cout << " position car 1 partials:\n"
            << traffic_states[1]->position().derivatives() << std::endl;
  std::cout << "\n speed car 1: " << traffic_states[0]->speed()
            << std::endl;
  std::cout << " speed car 1 partials:\n"
            << traffic_states[1]->speed().derivatives() << std::endl;

  std::cout << "\n x: " << ego_state->x() << std::endl;
  std::cout << " x partials:\n" << ego_state->x().derivatives() << std::endl;
  std::cout << "\n y: " << ego_state->y() << std::endl;
  std::cout << " y partials:\n" << ego_state->y().derivatives() << std::endl;
  std::cout << "\n heading: " << ego_state->heading() << std::endl;
  std::cout << " heading partials:\n" << ego_state->heading().derivatives()
            << std::endl;
  std::cout << "\n velocity: " << ego_state->velocity() << std::endl;
  std::cout << " velocity partials:\n" << ego_state->velocity().derivatives()
            << std::endl;
}

}  // namespace
}  // namespace sim_setup
}  // namespace automotive
}  // namespace drake
