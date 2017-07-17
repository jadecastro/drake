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
    int num_traffic_cars, int requested_lane_index, int num_dragway_lanes);
template std::unique_ptr<AutomotiveSimulator<AutoDiffXd>>
SetupAutomotiveSimulator<AutoDiffXd>(
    std::unique_ptr<const maliput::api::RoadGeometry> road_geometry,
    int num_traffic_cars, int requested_lane_index, int num_dragway_lanes);

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
                   std::vector<TrajectoryCarState<double>*>>
    GetAutomotiveSubsystemStates<double>(const systems::System<double>& plant,
                                         systems::Context<double>* context);
template std::pair<SimpleCarState<AutoDiffXd>*,
                   std::vector<TrajectoryCarState<AutoDiffXd>*>>
    GetAutomotiveSubsystemStates<AutoDiffXd>(
        const systems::System<AutoDiffXd>& plant,
        systems::Context<AutoDiffXd>* context);
}  // namespace sim_setup
}  // namespace automotive
}  // namespace drake
