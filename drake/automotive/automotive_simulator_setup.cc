#include "drake/automotive/automotive_simulator_setup.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace automotive {
namespace sim_setup {

// These instantiations must match the API documentation in
// automotive_simulator_setup.h.
template std::unique_ptr<AutomotiveSimulator<double>>
SetupAutomotiveSimulator<double>(
    std::unique_ptr<const maliput::api::RoadGeometry> road_geometry,
    int num_dragway_lanes, int ego_desired_lane_index,
    const std::vector<int> traffic_lane_indices);
template std::unique_ptr<AutomotiveSimulator<AutoDiffXd>>
SetupAutomotiveSimulator<AutoDiffXd>(
    std::unique_ptr<const maliput::api::RoadGeometry> road_geometry,
    int num_dragway_lanes, int ego_desired_lane_index,
    const std::vector<int> traffic_lane_indices);

template std::unique_ptr<systems::Simulator<double>> SetupSimulator<double>(
    const systems::System<double>& plant,
    std::unique_ptr<systems::Context<double>> context,
    double simulator_time_step);
template std::unique_ptr<systems::Simulator<AutoDiffXd>>
SetupSimulator<AutoDiffXd>(
    const systems::System<AutoDiffXd>& plant,
    std::unique_ptr<systems::Context<AutoDiffXd>> context,
    double simulator_time_step);

template std::pair<SimpleCarState<double>*,
                   std::vector<TrajectoryCarStruct<double>>>
    GetAutomotiveSubsystemStates<double>(const systems::System<double>& plant,
                                         std::vector<int> traffic_lane_indices,
                                         systems::Context<double>* context);
template std::pair<SimpleCarState<AutoDiffXd>*,
                   std::vector<TrajectoryCarStruct<AutoDiffXd>>>
    GetAutomotiveSubsystemStates<AutoDiffXd>(
        const systems::System<AutoDiffXd>& plant,
        std::vector<int> traffic_lane_indices,
        systems::Context<AutoDiffXd>* context);
}  // namespace sim_setup
}  // namespace automotive
}  // namespace drake
