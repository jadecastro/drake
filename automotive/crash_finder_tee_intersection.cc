#include <string>

#include <gflags/gflags.h>

#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/automotive/gen/pure_pursuit_params.h"
#include "drake/automotive/gen/simple_car_params.h"
#include "drake/automotive/trajectory_optimization.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/multilane/loader.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace automotive {
namespace {

using maliput::api::Lane;

/*
static constexpr double kRoadLength = 300.;
static constexpr double kLaneWidth = 3.;
static constexpr int kNumLanes = 3;
*/

static constexpr double kCarWidth = 2.;
static constexpr double kCarLength = 4.;

static constexpr int kNumTimeSamples = 21;
static constexpr double kInitialGuessDurationSec = 4.;

static constexpr double kMinTimeStep =
    0.9 * kInitialGuessDurationSec / (kNumTimeSamples - 1);
static constexpr double kMaxTimeStep =
    1.1 * kInitialGuessDurationSec / (kNumTimeSamples - 1);

static constexpr double kBoundingBoxLimit = 300.;

int DoMain(void) {
  // For building Multilane roads.
  //  const auto resource = FindResourceOrThrow(
  //    "drake/automotive/maliput/multilane/tee_intersection.yaml");
  const maliput::multilane::BuilderFactory builder_factory{};
  auto road = maliput::multilane::LoadFile(
      builder_factory, "/home/jon/drake-distro/automotive/maliput/multilane/tee_intersection.yaml");

  auto scenario =
      std::make_unique<Scenario>(std::move(road), kCarWidth, kCarLength);
  const maliput::api::Segment* branch_segment =
      scenario->road().junction(0)->segment(0);
  const maliput::api::Segment* straight_segment_0 =
      scenario->road().junction(3)->segment(0);
  const maliput::api::Segment* straight_segment_1 =
      scenario->road().junction(1)->segment(0);

  // Make an ego car.
  const auto ego = scenario->AddSimpleCar("ego_car", SimpleCarParams<double>());

  // Make three ado cars.
  const auto ado0 = scenario->AddIdmSimpleCar(
      "ado_car_0", LaneDirection(branch_segment->lane(0), true),
      SimpleCarParams<double>(), IdmPlannerParameters<double>(),
      PurePursuitParams<double>());
  /*
  const auto ado1 = scenario->AddIdmSimpleCar(
      "ado_car_1", LaneDirection(straight_segment_0->lane(0), true),
      SimpleCarParams<double>(), IdmPlannerParameters<double>(),
      PurePursuitParams<double>()));
  const auto ado2 = scenario->AddIdmSimpleCar(
      "ado_car_2", LaneDirection(straight_segment_0->lane(1), true),
      SimpleCarParams<double>(), IdmPlannerParameters<double>(),
      PurePursuitParams<double>()));
  */

  scenario->Build();

  auto falsifier = std::make_unique<TrajectoryOptimization>(
      std::move(scenario), kNumTimeSamples, kMinTimeStep, kMaxTimeStep,
      kInitialGuessDurationSec, kBoundingBoxLimit);

  // Supply initial conditions (used as falsification constraints and for the
  // initial guess trajectory).
  SimpleCarState<double> initial_conditions;
  auto ego_initial_pos = branch_segment->lane(0)->ToGeoPosition({3., 0., 0.});
  initial_conditions.set_x(ego_initial_pos.x());
  initial_conditions.set_y(ego_initial_pos.y());
  initial_conditions.set_heading(1.5);
  initial_conditions.set_velocity(7.);
  falsifier->RegisterInitialConstraint(*ego, initial_conditions);

  auto ado0_initial_pos = straight_segment_0->lane(0)->ToGeoPosition({4., 0., 0.});
  initial_conditions.set_x(ado0_initial_pos.x());
  initial_conditions.set_y(ado0_initial_pos.y());
  initial_conditions.set_heading(0.);
  initial_conditions.set_velocity(5.);
  falsifier->RegisterInitialConstraint(*ado0, initial_conditions);

  // Supply final conditions (only used for the initial guess trajectory).
  SimpleCarState<double> final_conditions;
  auto ego_final_pos = straight_segment_1->lane(0)->ToGeoPosition({0., 0., 0.});
  final_conditions.set_x(ego_final_pos.x());
  final_conditions.set_y(ego_final_pos.y());
  final_conditions.set_heading(0.);
  final_conditions.set_velocity(5.0);
  falsifier->RegisterFinalConstraint(*ego, final_conditions);

  auto ado0_final_pos = straight_segment_1->lane(0)->ToGeoPosition({2., 0., 0.});
  final_conditions.set_x(ado0_final_pos.x());
  final_conditions.set_y(ado0_final_pos.y());
  final_conditions.set_heading(0.);
  final_conditions.set_velocity(6.);
  falsifier->RegisterFinalConstraint(*ado0, final_conditions);

  // Set a guess trajectory based on these constraints.
  falsifier->SetLinearGuessTrajectory();

  // Set constraints on the initial states based on these constraints.
  falsifier->AddInitialConstraints();

  // Constraints keeping the cars on the road or in their lanes.
  /*
  std::pair<const Lane*, const Lane*> lane_bounds =
      std::make_pair(segment->lane(0), segment->lane(0));
  falsifier->AddDragwayLaneConstraints(*ego, lane_bounds);
  falsifier->AddDragwayLaneConstraints(*ado0, lane_bounds);
  */

  falsifier->AddFinalCollisionConstraints(*ego, *ado0);

  Eigen::Matrix2d sigma;
  sigma << 0.1, 0.0, // BR
           0.0, 0.1;
  falsifier->AddGaussianCost(*ado0, sigma);

  const solvers::SolutionResult result = falsifier->Solve();
  if (result == solvers::SolutionResult::kSolutionFound) {
    std::cout << "Solution found." << std::endl;
  } else {
    std::cout << "A solution could not be found." << std::endl;
  }
  std::cout << "The log-pdf of the solution is: "
            << falsifier->GetSolutionTotalLogPdf() << std::endl;
  std::cout << "The normalized log-pdf of the solution is: "
            << falsifier->GetSolutionTotalLogNormalizedPdf() << std::endl;

  falsifier->AnimateSolution();

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::automotive::DoMain();
}
