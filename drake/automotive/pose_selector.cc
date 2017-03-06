#include "drake/automotive/pose_selector.h"

#include <cmath>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

template <typename T>
const std::pair<RoadPosition, RoadPosition>
PoseSelector<T>::SelectClosestPositions(const RoadGeometry& road,
                                        const Lane* agent_lane,
                                        const PoseVector<T>& ego_pose,
                                        const PoseBundle<T>& agent_poses) {
  const RoadPosition& ego_position =
      GetRoadPosition(road, ego_pose.get_isometry());
  const Lane* lane = (agent_lane == nullptr) ? ego_position.lane : agent_lane;

  RoadPosition result_ahead = RoadPosition(lane, LanePosition(1e6, 0, 0));
  RoadPosition result_behind = RoadPosition(lane, LanePosition(-1e6, 0, 0));
  for (int i = 0; i < agent_poses.get_num_poses(); ++i) {
    const RoadPosition& agent_position =
        GetRoadPosition(road, agent_poses.get_pose(i));
    const T& s_agent = agent_position.pos.s;

    // Accept this pose if it is not the ego car and it in the correct lane,
    // then plop it into the correct bin.
    if (ego_position.lane->id().id != lane->id().id ||
        s_agent != ego_position.pos.s) {
      if (agent_position.lane->id().id == lane->id().id &&
          result_behind.pos.s < s_agent && s_agent < result_ahead.pos.s) {
        if (s_agent >= ego_position.pos.s)
          result_ahead = agent_position;
        else
          result_behind = agent_position;
      }
    }
  }
  std::cerr << " result_ahead.pos.s: " << result_ahead.pos.s << std::endl;
  return std::make_pair(result_ahead, result_behind);
}

template <typename T>
const RoadPosition PoseSelector<T>::SelectClosestPositionAhead(
    const RoadGeometry& road, const PoseVector<T>& ego_pose,
    const PoseBundle<T>& agent_poses) {
  return SelectClosestPositions(road, nullptr, ego_pose, agent_poses).first;
}

template <typename T>
const RoadPosition PoseSelector<T>::GetRoadPosition(const RoadGeometry& road,
                                                    const Isometry3<T>& pose) {
  const maliput::api::GeoPosition& geo_position = maliput::api::GeoPosition(
      pose.translation().x(), pose.translation().y(), pose.translation().z());
  return road.ToRoadPosition(geo_position, nullptr, nullptr, nullptr);
}

// These instantiations must match the API documentation in pose_selector.h.
template class PoseSelector<double>;

}  // namespace automotive
}  // namespace drake
