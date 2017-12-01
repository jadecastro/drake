#include <memory>

#include <gflags/gflags.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/automotive_simulator_setup.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/common/autodiff.h"
#include "drake/common/autodiffxd_make_coherent.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace automotive {
namespace {

static constexpr double kLaneWidth = 2.;
static constexpr double kDragwayLength = 150.;

static std::pair<SimpleCarState<AutoDiffXd>*,
                 std::vector<TrajectoryCarStruct<AutoDiffXd>>>
EvalInstantaneous(systems::Simulator<AutoDiffXd>& simulator,
                  const maliput::api::RoadGeometry& road,
                  std::vector<int> traffic_lane_indices) {
  const auto& plant = simulator.get_system();
  auto& context = simulator.get_mutable_context();

  return sim_setup::GetAutomotiveSubsystemStates(plant, traffic_lane_indices,
                                                 context);
}

static std::pair<SimpleCarState<AutoDiffXd>*,
                 std::vector<TrajectoryCarStruct<AutoDiffXd>>>
EvalStep(std::unique_ptr<systems::Simulator<AutoDiffXd>> simulator,
         const maliput::api::RoadGeometry& road,
         std::vector<int> traffic_lane_indices, const AutoDiffXd& time_step,
         const AutoDiffXd& time_horizon) {
  AutoDiffXd t = simulator->get_context().get_time();

  std::pair<SimpleCarState<AutoDiffXd>*,
            std::vector<TrajectoryCarStruct<AutoDiffXd>>> running_result;

  while (t < time_horizon) {
    simulator->StepTo(t + time_step);
    t = simulator->get_context().get_time();

    // Evaluate the loss function at this time step.
    running_result =
        EvalInstantaneous(*simulator, road, traffic_lane_indices);

    // std::cout << " Current time: " << simulator->get_context().get_time()
    //         << std::endl;
  }
  return running_result;
}

int DoMain(void) {
  // ==================================
  // Parameters and initial conditions.
  const int num_dragway_lanes = 2;

  // Ego car parameters and initial conditions.
  //const int ego_initial_lane_index = 0;
  const int ego_desired_lane_index = 1;  // Remove this once we have MOBIL.
  //const double ego_x = 50.;
  //const double ego_heading = 0.;
  //const double ego_velocity = 5.;

  // Traffic car initial conditions.
  const std::vector<int> traffic_lane_indices{0, 1};  // N.B. vector's size
                                                      // determines the number
                                                      // of traffic cars.
  std::vector<double> traffic_pos(traffic_lane_indices.size());
  std::vector<double> traffic_speed(traffic_lane_indices.size());
  traffic_pos[0] = 60.;
  traffic_speed[0] = 5.;
  traffic_pos[1] = 5.;
  traffic_speed[1] = 5.;
  // ==================================

  // Build the scenario.
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry =
      std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"dragway"}), num_dragway_lanes,
          kDragwayLength, kLaneWidth, 0. /* shoulder width */,
          5. /* maximum height above road */,
          std::numeric_limits<double>::epsilon() /* linear tolerance */,
          std::numeric_limits<double>::epsilon() /* angular tolerance */);

  // Build an AutomotiveSimulator containing traffic cars and an ego car.
  auto automotive_simulator = sim_setup::SetupAutomotiveSimulator<double>(
      std::move(road_geometry), num_dragway_lanes, ego_desired_lane_index,
      traffic_lane_indices);
  const auto plant = automotive_simulator->GetDiagram().ToAutoDiffXd();
  auto context = plant->CreateDefaultContext();

  // Retrieve a vector of states corresponding to each car subsystem.
  SimpleCarState<AutoDiffXd>* ego_state = nullptr;
  std::vector<TrajectoryCarStruct<AutoDiffXd>> traffic_structs{};
  std::tie(ego_state, traffic_structs) =
      sim_setup::GetAutomotiveSubsystemStates(*plant, traffic_lane_indices,
                                              *context);
  DRAKE_DEMAND(traffic_structs.size() == traffic_lane_indices.size());

  // Set all of the initial states by name.
  //const double initial_lane_y_value =
  //    -kLaneWidth / 2. * (num_dragway_lanes - 1) +
  //    ego_initial_lane_index * kLaneWidth;
  //ego_state->set_x(ego_x);
  //ego_state->set_y(initial_lane_y_value);
  //ego_state->set_heading(ego_heading);
  //ego_state->set_velocity(ego_velocity);
  for (int i{0}; i < static_cast<int>(traffic_structs.size()); ++i) {
    //    traffic_structs[i].states->set_position(traffic_pos[i]);
    //traffic_structs[i].states->set_speed(traffic_speed[i]);
  }

  // Declare the partial derivatives for all of the context members.
  sim_setup::InitializeAutoDiffContext(context.get());

  // Initialize the Drake Simulator.
  std::unique_ptr<systems::Simulator<AutoDiffXd>> simulator =
      sim_setup::SetupSimulator<AutoDiffXd>(*plant, std::move(context),
                                            0.01 /* simulator time step */);

  // Simulate forward up to the specified horizon, and evaluate the loss
  // function at the desired time steps along the way.

  std::tie(ego_state, traffic_structs) =
      EvalStep(std::move(simulator),
               *automotive_simulator->GetRoadGeometry(),
               traffic_lane_indices, 0.1 /* evaluation time step */,
               10. /* time horizon */);

  // TODO(jadecastro): Use a python wrapper or our usual LCM tricks to grab the
  // data ego_state and traffic_structs data.
  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::automotive::DoMain();
}
