#pragma once

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// RoadOdometry Contains the position of the vehicle with respect to a lane in
/// a road, along with its velocity vector in the world frame.
template <typename T>
struct RoadOdometry {
  /// Default constructor.
  RoadOdometry() = default;
  /// Fully-parameterized constructor.
  RoadOdometry(const maliput::api::RoadPosition& road_position,
               const systems::rendering::FrameVelocity<T>& frame_velocity)
      : lane(road_position.lane),
        pos(road_position.pos.s(), road_position.pos.r(),
            road_position.pos.h()),
        vel(frame_velocity) {}
  RoadOdometry(const maliput::api::Lane* const lane,
               const maliput::api::LanePositionWithAutoDiff<T>& lane_position,
               const systems::rendering::FrameVelocity<T>& frame_velocity)
      : lane(lane), pos(lane_position), vel(frame_velocity) {}

  const maliput::api::Lane* lane{};
  maliput::api::LanePositionWithAutoDiff<T> pos{};
  systems::rendering::FrameVelocity<T> vel{};
};

}  // namespace automotive
}  // namespace drake
