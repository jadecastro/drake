#include "drake/automotive/pose_selector.h"

#include <limits>
#include <memory>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/math/saturate.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

template <typename T>
const std::map<AheadOrBehind, const ClosestPose<T>>
PoseSelector<T>::FindClosestPair(const Lane* const lane,
                                 const PoseVector<T>& ego_pose,
                                 const FrameVelocity<T>& ego_velocity,
                                 const PoseBundle<T>& traffic_poses,
                                 const T& scan_distance) {
  const ClosestPose<T> result_leading =
      FindSingleClosestPose(lane, ego_pose, ego_velocity, traffic_poses,
                            scan_distance, AheadOrBehind::kAhead);
  const ClosestPose<T> result_trailing =
      FindSingleClosestPose(lane, ego_pose, ego_velocity, traffic_poses,
                            scan_distance, AheadOrBehind::kBehind);

  std::map<AheadOrBehind, const ClosestPose<T>> result;
  result.insert(std::make_pair(AheadOrBehind::kAhead, result_leading));
  result.insert(std::make_pair(AheadOrBehind::kBehind, result_trailing));
  return result;
}

template <typename T>
const ClosestPose<T> PoseSelector<T>::FindSingleClosestPose(
    const Lane* const lane, const PoseVector<T>& ego_pose,
    const FrameVelocity<T>& ego_velocity, const PoseBundle<T>& traffic_poses,
    const T& scan_distance, const AheadOrBehind ahead_or_behind) {
  DRAKE_DEMAND(lane != nullptr);

  ClosestPose<T> result;
  result.odometry = set_default_odometry(lane, ahead_or_behind);
  result.distance = std::numeric_limits<T>::infinity();
  ClosestPose<T> default_result = result;

  const GeoPosition ego_geo_position{ego_pose.get_translation().x(),
                                     ego_pose.get_translation().y(),
                                     ego_pose.get_translation().z()};
  const LanePosition ego_lane_position =
      lane->ToLanePosition(ego_geo_position, nullptr, nullptr);

  // Get ego car's heading with respect to the trial lane.
  const Eigen::Quaternion<T> ego_rotation = ego_pose.get_rotation();
  const Eigen::Quaternion<T> lane_rotation =
      lane->GetOrientation(ego_lane_position).quat();
  // with_s is interpereted as the direction, in a particular lane, along which
  // we are observing other cars.  initial_with_s is the value of with_s in the
  // initial lane.
  const bool initial_with_s = (ahead_or_behind == AheadOrBehind::kAhead)
                                  ? lane_rotation.dot(ego_rotation) >= 0.0
                                  : lane_rotation.dot(ego_rotation) < 0.0;
  LaneDirection lane_direction(lane, initial_with_s);

  // Compute the s-direction of the ego car and its direction of travel.
  // N.B. `distance_scanned` is a net distance, so will always increase.  Note
  // also that the result will always be positive, as the current lane's length
  // is always added to the result at each loop iteration.
  T distance_scanned = initial_with_s
                           ? T(-ego_lane_position.s())
                           : T(ego_lane_position.s() - lane->length());

  // Traverse forward or backward from the current lane the given scan_distance,
  // looking for traffic cars.
  while (distance_scanned < scan_distance) {
    T distance_increment{};
    for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {
      Isometry3<T> traffic_iso = traffic_poses.get_pose(i);
      const GeoPosition traffic_geo_position(traffic_iso.translation().x(),
                                             traffic_iso.translation().y(),
                                             traffic_iso.translation().z());

      if (ego_geo_position == traffic_geo_position) continue;
      if (!IsWithinLane(traffic_geo_position, lane_direction.lane)) continue;

      const LanePosition traffic_lane_position =
          lane_direction.lane->ToLanePosition(traffic_geo_position, nullptr,
                                              nullptr);
      // Ignore traffic that are not in the desired direction (ahead or behind)
      // of the ego car (with respect to the car's current direction) when the
      // two share the same lane.
      if (ahead_or_behind == AheadOrBehind::kAhead) {
        if (distance_scanned <= T(0.)) {
          if ((lane_direction.with_s &&
               traffic_lane_position.s() <= ego_lane_position.s()) ||
              (!lane_direction.with_s &&
               traffic_lane_position.s() >= ego_lane_position.s())) {
            continue;
          }
        }
      } else if (ahead_or_behind == AheadOrBehind::kBehind) {
        if (distance_scanned <= T(0.)) {
          if ((lane_direction.with_s &&
               traffic_lane_position.s() < ego_lane_position.s()) ||
              (!lane_direction.with_s &&
               traffic_lane_position.s() > ego_lane_position.s())) {
            continue;
          }
        }
      }
      // Ignore positions at the desired direction (ahead or behind) of the ego
      // car that are not closer than any other found so far.
      if (ahead_or_behind == AheadOrBehind::kAhead) {
        if (traffic_lane_position.s() > result.odometry.pos.s()) continue;
      } else if (ahead_or_behind == AheadOrBehind::kBehind) {
        if (traffic_lane_position.s() <= result.odometry.pos.s()) continue;
      }

      // Update the result with the new candidate.
      result.odometry =
          RoadOdometry<T>({lane_direction.lane, traffic_lane_position},
                          traffic_poses.get_velocity(i));
      distance_increment =
          lane_direction.with_s
              ? result.odometry.pos.s()
              : (lane_direction.lane->length() - result.odometry.pos.s());
    }

    if (std::abs(result.odometry.pos.s()) <
        std::numeric_limits<double>::infinity()) {
      // Figure out whether or not the result is within scan_distance.
      if (distance_scanned + distance_increment < scan_distance) {
        result.distance = distance_scanned + distance_increment;
        return result;
      }
    }
    // Increment distance_scanned.
    distance_scanned += T(lane_direction.lane->length());

    // Obtain the next lane_direction in the scanned sequence.
    get_default_ongoing_lane(&lane_direction);
    if (lane_direction.lane == nullptr) return result;  // +/- Infinity.
  }
  return default_result;
}

template <typename T>
const T PoseSelector<T>::GetSigmaVelocity(
    const RoadOdometry<T>& road_odometry) {
  const double kLargeSValue{1e9};
  const LanePosition sat_position{
      math::saturate(road_odometry.pos.s(), -kLargeSValue, kLargeSValue),
      road_odometry.pos.r(), road_odometry.pos.h()};
  const maliput::api::Rotation rot =
      road_odometry.lane->GetOrientation(sat_position);
  const Vector3<T>& vel = road_odometry.vel.get_velocity().translational();
  return vel(0) * std::cos(rot.yaw()) + vel(1) * std::sin(rot.yaw());
}

template <typename T>
bool PoseSelector<T>::IsWithinLane(const GeoPosition& geo_position,
                                   const Lane* const lane) {
  double distance{};
  const LanePosition pos =
      lane->ToLanePosition(geo_position, nullptr, &distance);
  return (distance == 0. && pos.r() >= lane->lane_bounds(pos.s()).r_min &&
          pos.r() <= lane->lane_bounds(pos.s()).r_max);
}

template <typename T>
std::unique_ptr<LaneEnd> PoseSelector<T>::get_default_ongoing_lane(
    LaneDirection* lane_direction) {
  const Lane* lane{lane_direction->lane};
  const bool with_s{lane_direction->with_s};
  std::unique_ptr<LaneEnd> branch =
      (with_s) ? lane->GetDefaultBranch(LaneEnd::kFinish)
               : lane->GetDefaultBranch(LaneEnd::kStart);
  if (branch == nullptr) {
    lane_direction->lane = nullptr;
    lane_direction->with_s = true;
    return branch;
  }
  lane_direction->lane = branch->lane;
  lane_direction->with_s = (branch->end == LaneEnd::kStart) ? true : false;
  return branch;
}

template <typename T>
const RoadOdometry<T> PoseSelector<T>::set_default_odometry(
    const Lane* const lane, const AheadOrBehind ahead_or_behind) {
  const double infinite_distance =
      (ahead_or_behind == AheadOrBehind::kAhead)
          ? std::numeric_limits<double>::infinity()
          : -std::numeric_limits<double>::infinity();
  const RoadPosition default_road_position(lane, {infinite_distance, 0., 0.});
  return {default_road_position, FrameVelocity<T>()};
}

template <typename T>
template <typename T1>
void IdmController<T>::CalcLanePosition(
    const PoseVector<std::enable_if_t<std::is_same<T1, double>::value, T1>>&
    ego_pose) {
  DRAKE_ABORT();
}

template <typename T>
template <typename T1>
void IdmController<T>::CalcLanePosition(
    const PoseVector<std::enable_if_t<std::is_same<T1, AutoDiffXd>::value, T1>>&
    ego_pose) {
  DRAKE_ABORT();
}

// These instantiations must match the API documentation in pose_selector.h.
template class PoseSelector<double>;
template class PoseSelector<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
