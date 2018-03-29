#include "drake/automotive/autodiff_simulator.h"

#include <gtest/gtest.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace automotive {
namespace {

// TODO(jadecastro) If this code ever becomes serious, we'd better make this
// common to this and automotive_simulator_test.cc.
std::unique_ptr<AutomotiveSimulator<double>> MakeWithIdmCarAndDecoy() {
  auto simulator = std::make_unique<AutomotiveSimulator<double>>(nullptr);
  const maliput::api::RoadGeometry* road{};
  EXPECT_NO_THROW(
      road = simulator->SetRoadGeometry(
          std::make_unique<const maliput::dragway::RoadGeometry>(
              maliput::api::RoadGeometryId("TestDragway"), 2 /* num lanes */,
              100 /* length */, 4 /* lane width */, 1 /* shoulder width */,
              5 /* maximum_height */,
              std::numeric_limits<double>::epsilon() /* linear_tolerance */,
              std::numeric_limits<double>::epsilon() /* angular_tolerance */)));

  // ---------------------------------------------------------------
  // ^  +r, +y
  // |    -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
  // +---->  +s, +x       |  IDM Car  |         |  Decoy  |
  // ---------------------------------------------------------------
  const double start_s_position(2.);
  const double start_speed(10.);

  const int kStartLaneIndex = 0;
  const maliput::api::Lane* start_lane =
      road->junction(0)->segment(0)->lane(kStartLaneIndex);
  const int kGoalLaneIndex = 0;
  const maliput::api::Lane* goal_lane =
      road->junction(0)->segment(0)->lane(kGoalLaneIndex);

  // Set the initial states.
  const maliput::api::GeoPosition start_position =
      start_lane->ToGeoPosition({start_s_position, 0., 0.});

  SimpleCarState<double> initial_state;
  // The following presumes we are on a dragway, in which x -> s, y -> r.
  initial_state.set_x(start_position.x());
  initial_state.set_y(start_position.y());
  initial_state.set_heading(0.);
  initial_state.set_velocity(start_speed);

  int id_idm_car{};
  EXPECT_NO_THROW(id_idm_car = simulator->AddIdmControlledCar(
      "idm_car", true /* with_s */, initial_state, goal_lane,
      ScanStrategy::kPath,
      RoadPositionStrategy::kExhaustiveSearch, 0. /* time period (unused) */));
  EXPECT_EQ(id_idm_car, 0);

  auto dragway = dynamic_cast<const maliput::dragway::RoadGeometry*>(road);
  EXPECT_NE(nullptr, dragway);

  const double traffic_s(6.);
  const double traffic_speed(0.);
  const auto& traffic_params =
      CreateTrajectoryParamsForDragway(*dragway, kStartLaneIndex, traffic_speed,
                                       0. /* start time */);
  const int id_decoy = simulator->AddPriusTrajectoryCar(
      "decoy", std::get<0>(traffic_params), traffic_speed, traffic_s);
  EXPECT_EQ(id_decoy, 1);

  return simulator;
}


GTEST_TEST(AutodiffSimulatorTest, BasicTest) {
  auto simulator = MakeWithIdmCarAndDecoy();
  simulator->Build();
  const systems::System<double>& system = simulator->GetDiagram();

  const auto ad_simulator = std::make_unique<AutodiffSimulator>(system);
  EXPECT_NO_THROW(ad_simulator->StepTo(10.));

  const VectorX<AutoDiffXd> state = ad_simulator->get_state();
  EXPECT_TRUE(CompareMatrices(
      state(0).derivatives(), Eigen::Matrix<double, 7, 1>::Zero()));
}

}  // namespace
}  // namespace automotive
}  // namespace drake
