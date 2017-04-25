#include <limits>
#include <tuple>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"

#include "drake/common/drake_assert.h"
#include <iostream>
namespace drake {
namespace maliput {
namespace monolane {

Junction* RoadGeometry::NewJunction(api::JunctionId id) {
  junctions_.push_back(std::make_unique<Junction>(id, this));
  return junctions_.back().get();
}


BranchPoint* RoadGeometry::NewBranchPoint(api::BranchPointId id) {
  branch_points_.push_back(std::make_unique<BranchPoint>(id, this));
  return branch_points_.back().get();
}


const api::Junction* RoadGeometry::do_junction(int index) const {
  return junctions_[index].get();
}


const api::BranchPoint* RoadGeometry::do_branch_point(int index) const {
  return branch_points_[index].get();
}


api::RoadPosition RoadGeometry::DoToRoadPosition(
    const api::GeoPosition& geo_position,
    const api::RoadPosition* hint,
    api::GeoPosition* nearest_position,
    double* distance) const {
  // TODO(jadecastro): Check that the hint contains a valid lane.
  if (hint != nullptr) {
    return {hint->lane,
          hint->lane->ToLanePosition(geo_position, nearest_position, distance)};
  }
  double min_distance{std::numeric_limits<double>::infinity()};
  std::tuple<int, int, int> indices{0, 0, 0};
  std::cerr << " DoToRoadPosition  " << std::endl;
  for (int i = 0; i < num_junctions(); ++i) {
    const api::Junction* junction = this->junction(i);
    for (int j = 0; j < junction->num_segments(); ++j) {
      const api::Segment* segment = junction->segment(j);
      for (int k = 0; k < segment->num_lanes(); ++k) {
        const api::Lane* lane = segment->lane(k);
        double new_distance{};
        const api::LanePosition lane_position =
            lane->ToLanePosition(geo_position, nearest_position, &new_distance);
        if (new_distance >= min_distance) continue;
        if (lane->length() < lane_position.s || lane_position.s < 0.) {
          continue;
        }
        if (lane->lane_bounds(lane_position.s).r_max <= lane_position.r
            || lane_position.r <= lane->lane_bounds(lane_position.s).r_min) {
          continue;
        }
        indices = std::make_tuple(i, j, k);
        min_distance = new_distance;
      }
    }
  }
  const api::Lane* lane =
      junction(std::get<0>(indices))->segment(std::get<1>(indices))
      ->lane(std::get<2>(indices));
  std::cerr << "  lane  " << lane->id().id << std::endl;
  return {lane, lane->ToLanePosition(geo_position, nearest_position, distance)};
}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
