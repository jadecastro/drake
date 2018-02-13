#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/curve2.h"
#include "drake/automotive/trajectory_car.h"
#include "drake/automotive/gen/maliput_railcar_params.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/lcm/drake_lcm.h"

std::unique_ptr<const drake::maliput::api::RoadGeometry> add_oval_road(
    int num_lanes) {
  const double lane_width = 5.;
  drake::maliput::api::HBounds elevation_bounds(0., 5.);
  drake::maliput::multilane::Builder builder(lane_width, elevation_bounds,
                                             1e-8, 1e-8 * M_PI);
  drake::maliput::multilane::ArcOffset counter_clockwise_arc(50., M_PI);
  drake::maliput::multilane::EndpointZ LayFlat(0., 0., 0., 0.);
  drake::maliput::multilane::Endpoint start{{0., 0., -M_PI / 4.}, LayFlat};

  auto c0 = builder.Connect("0", num_lanes, 0, 0., 0., start, 200, LayFlat);
  auto c1 = builder.Connect("1", num_lanes, 0, 0, 0, c0->end(),
                            counter_clockwise_arc, LayFlat);
  auto c2 = builder.Connect("2", num_lanes, 0, 0, 0, c1->end(), 200, LayFlat);
  builder.Connect("3", num_lanes, 0, 0, 0, c2->end(),
                  counter_clockwise_arc, LayFlat);
  //builder.Connect("4",num_lanes,0,2,2,c3->end(),50,c0->start());

  std::unique_ptr<const drake::maliput::api::RoadGeometry> oval =
      builder.Build(drake::maliput::api::RoadGeometryId{"oval"});
  return oval;
}

std::unique_ptr<const drake::maliput::api::RoadGeometry> add_dragway(
    double length, int num_lanes){
  const double kMaximumHeight = 5.;  // meters
  const double lane_width = 5;
  const double shoulder_width = 2;
  const double kLinearTolerance = std::numeric_limits<double>::epsilon();
  const double kAngularTolerance = std::numeric_limits<double>::epsilon();
  std::unique_ptr<const drake::maliput::api::RoadGeometry> road_geometry
      = std::make_unique<const drake::maliput::dragway::RoadGeometry>(
          drake::maliput::api::RoadGeometryId("Automotive Demo Dragway"),
          num_lanes,
          length,
          lane_width,
          shoulder_width,
          kMaximumHeight, kLinearTolerance, kAngularTolerance);
  return road_geometry;
}

void add_idm_car(std::string car_name, const drake::maliput::api::Lane* lane,
                 double lane_pos, double speed,
                 drake::automotive::AutomotiveSimulator<double>* simulator){
  drake::automotive::MaliputRailcarParams<double> params;
  drake::automotive::MaliputRailcarState<double> state;
  params.set_r(0);
  params.set_h(0);
  state.set_s(lane_pos);
  state.set_speed(speed);
  simulator->AddIdmControlledPriusMaliputRailcar(
      car_name, drake::automotive::LaneDirection(lane),
      drake::automotive::RoadPositionStrategy::kExhaustiveSearch, 0,
      params, state);
}


int main(){
  auto simulator =
      std::make_unique<drake::automotive::AutomotiveSimulator<double>>(
          std::make_unique<drake::lcm::DrakeLcm>());

  auto road_geometry = add_oval_road(3);
  //auto road_geometry = add_dragway(200,3);

  const int idm_1_lane = 0;
  const int idm_2_lane = 1;
  // const int idm_3_lane = 2;

  const drake::maliput::api::Lane* lane1 =
      road_geometry->junction(0)->segment(0)->lane(idm_1_lane);
  //do this BEFORE you move the road geometry to the simulator
  const drake::maliput::api::Lane* lane2 =
      road_geometry->junction(0)->segment(0)->lane(idm_2_lane);
  // const auto * lane3 = road_geometry->junction(0)->segment(0)->lane(idm_3_lane);

  simulator->SetRoadGeometry(std::move(road_geometry));

  add_idm_car("Car1", lane1, 50, 5, simulator.get());
  add_idm_car("Car2", lane2, 0, 100, simulator.get());

  simulator->Start(1.0);
  simulator->StepBy(std::numeric_limits<double>::infinity());


  return 0;
}
