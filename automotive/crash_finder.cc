#include <gflags/gflags.h>

#include "drake/automotive/automotive_trajectory_optimization.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/segment.h"

namespace drake {
namespace automotive {
namespace {

using maliput::api::Lane;

static constexpr double kRoadLength = 100.;
static constexpr double kLaneWidth = 3.;
static constexpr int kNumLanes = 3;

static constexpr int kNumTimeSamples = 21;
static constexpr double kInitialGuessDurationSec = 10.;

static constexpr int kX = SimpleCarStateIndices::kX;
static constexpr int kY = SimpleCarStateIndices::kY;

int DoMain(void) {
  auto scenario =
      std::make_unique<Scenario>(kNumLanes, kLaneWidth, kRoadLength);
  const maliput::api::Segment* segment =
      scenario->road().junction(0)->segment(0);

  // Make an ego car.
  const auto ego = scenario->AddSimpleCar("ego_car");

  // Make three ado cars.
  const auto ado0 = scenario->AddIdmSimpleCar("ado_car_0");
  // const auto ado1 = scenario->AddIdmSimpleCar("ado_car_1");
  // const auto ado2 = scenario->AddIdmSimpleCar("ado_car_2");

  // Fix the goal lanes for each of the ado cars.
  scenario->FixGoalLaneDirection(*ado0, LaneDirection(segment->lane(2), true));
  // scenario->FixGoalLaneDirection(*ado1, LaneDirection(segment->lane(0), true));
  // scenario->FixGoalLaneDirection(*ado2, LaneDirection(segment->lane(1), true));

  scenario->Build();

  // Supply initial conditions (used as falsification constraints and for the
  // initial guess trajectory).
  SimpleCarState<double> initial_conditions;
  auto ego_initial_pos = segment->lane(0)->ToGeoPosition({30., 0., 0.});
  initial_conditions.set_x(ego_initial_pos.x());
  initial_conditions.set_y(ego_initial_pos.y());
  initial_conditions.set_heading(0.);
  initial_conditions.set_velocity(7.);
  scenario->SetInitialSubsystemState(*ego, initial_conditions);

  auto ado0_initial_pos = segment->lane(2)->ToGeoPosition({10., 0., 0.});
  initial_conditions.set_x(ado0_initial_pos.x());
  initial_conditions.set_y(ado0_initial_pos.y());
  initial_conditions.set_heading(0.);
  initial_conditions.set_velocity(5.);
  scenario->SetInitialSubsystemState(*ado0, initial_conditions);

  /*
  auto ado1_initial_pos = segment->lane(0)->ToGeoPosition({40., 0., 0.});
  initial_conditions.set_x(ado1_initial_pos.x());
  initial_conditions.set_y(ado1_initial_pos.y());
  initial_conditions.set_heading(0.);
  initial_conditions.set_velocity(5.);
  scenario->SetInitialSubsystemState(*ado1, initial_conditions);
  */
  /*
  auto ado2_initial_pos = segment->lane(1)->ToGeoPosition({60., 0., 0.});
  initial_conditions.set_x(ado2_initial_pos.x());
  initial_conditions.set_y(ado2_initial_pos.y());
  initial_conditions.set_heading(0.);
  initial_conditions.set_velocity(8.);
  scenario->SetInitialSubsystemState(*ado2, initial_conditions);
  */

  // Supply final conditions (only used for the initial guess trajectory).
  SimpleCarState<double> final_conditions;
  auto ego_final_pos = segment->lane(0)->ToGeoPosition({40., 1.2, 0.});
  final_conditions.set_x(ego_final_pos.x());
  final_conditions.set_y(ego_final_pos.y());
  final_conditions.set_heading(0.);
  final_conditions.set_velocity(5.0);
  scenario->SetFinalSubsystemState(*ego, final_conditions);

  auto ado0_final_pos = segment->lane(2)->ToGeoPosition({40., 0., 0.});
  final_conditions.set_x(ado0_final_pos.x());
  final_conditions.set_y(ado0_final_pos.y());
  final_conditions.set_heading(0.);
  final_conditions.set_velocity(6.);
  scenario->SetFinalSubsystemState(*ado0, final_conditions);

  /*
  auto ado1_final_pos = segment->lane(0)->ToGeoPosition({60., 0., 0.});
  final_conditions.set_x(ado1_final_pos.x());
  final_conditions.set_y(ado1_final_pos.y());
  final_conditions.set_heading(0.);
  final_conditions.set_velocity(5.);
  scenario->SetFinalSubsystemState(*ado1, final_conditions);
  */
  /*
  auto ado2_final_pos = segment->lane(1)->ToGeoPosition({80., 0., 0.});
  final_conditions.set_x(ado2_final_pos.x());
  final_conditions.set_y(ado2_final_pos.y());
  final_conditions.set_heading(0.);
  final_conditions.set_velocity(8.);
  scenario->SetFinalSubsystemState(*ado2, final_conditions);
  */

  const double kMinTimeStep =
      0.2 * kInitialGuessDurationSec / (kNumTimeSamples - 1);
  const double kMaxTimeStep =
      3. * kInitialGuessDurationSec / (kNumTimeSamples - 1);
  auto falsifier = std::make_unique<AutomotiveTrajectoryOptimization>(
      std::move(scenario), kNumTimeSamples, kMinTimeStep, kMaxTimeStep,
      kInitialGuessDurationSec);

  // Constraints keeping the cars on the road or in their lanes.
  std::pair<const Lane*, const Lane*> lane_bounds =
      std::make_pair(segment->lane(0), segment->lane(2));
  falsifier->SetDragwayLateralLaneBounds(ego, lane_bounds);
  falsifier->SetDragwayLateralLaneBounds(ado0, lane_bounds);
  // falsifier->SetDragwayLateralLaneBounds(ado1, lane_bounds);
  // falsifier->SetDragwayLateralLaneBounds(ado2, lane_bounds);

  // Solve a collision with one of the cars, in this case, Traffic Car 2 merging
  // into the middle lane.  Assume for now that, irrespective of its
  // orientation, the traffic car occupies a lane-aligned bounding box.
  const auto final_state = falsifier->prog()->final_state();
  const int ego_y_index = falsifier->scenario().GetStateIndices(*ego)[kY];
  const int ado_y_index = falsifier->scenario().GetStateIndices(*ado0)[kY];
  falsifier->prog()->AddLinearConstraint(
      final_state[ego_y_index] <= final_state[ado_y_index] + 0.5 * kLaneWidth);
  falsifier->prog()->AddLinearConstraint(
      final_state[ado_y_index] >= final_state[ego_y_index] - 2.5);
  const int ego_x_index = falsifier->scenario().GetStateIndices(*ego)[kX];
  const int ado_x_index = falsifier->scenario().GetStateIndices(*ado0)[kX];
  falsifier->prog()->AddLinearConstraint(
      final_state[ado_x_index] <= final_state[ego_x_index] + 2.5);

  falsifier->AddLogProbabilityCost();

  falsifier->AddLogProbabilityChanceConstraint();

  falsifier->SetLinearGuessTrajectory();

  falsifier->Solve();

  falsifier->SimulateResult();

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::automotive::DoMain();
}
