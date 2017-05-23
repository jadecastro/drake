#include "drake/automotive/pose_selector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>

#include "drake/common/drake_assert.h"
#include "drake/math/quaternion.h"
#include "drake/math/saturate.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPosition;
using maliput::api::IsoLaneVelocity;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LaneEndSet;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

template <typename T>
const std::pair<const RoadOdometry<double>, const RoadOdometry<double>>
PoseSelector<T>::FindClosestPair(const Lane* const lane,
                                 const PoseVector<double>& ego_pose,
                                 const FrameVelocity<double>& ego_velocity,
                                 const PoseBundle<double>& traffic_poses,
                                 double scan_distance,
                                 std::pair<double, double>* distances) {
  double distance_ahead{};
  double distance_behind{};
  const RoadOdometry<double> result_leading =
      FindSingleClosestPose<double>(lane, ego_pose, ego_velocity, traffic_poses,
                            scan_distance, WhichSide::kAhead, &distance_ahead);
  const RoadOdometry<double> result_trailing =
      FindSingleClosestPose<double>(lane, ego_pose, ego_velocity, traffic_poses,
                            scan_distance, WhichSide::kBehind,
                            &distance_behind);
  if (distances != nullptr) {
    *distances = std::make_pair(distance_ahead, distance_behind);
  }
  return std::make_pair(result_leading, result_trailing);
}

template <typename T>
const std::pair<const RoadOdometry<AutoDiffXd>, const RoadOdometry<AutoDiffXd>>
PoseSelector<T>::FindClosestPair(const Lane* const lane,
                                 const PoseVector<AutoDiffXd>& ego_pose,
                                 const FrameVelocity<AutoDiffXd>& ego_velocity,
                                 const PoseBundle<AutoDiffXd>& traffic_poses,
                                 double scan_distance,
                                 std::pair<AutoDiffXd, AutoDiffXd>* distances) {
  DRAKE_ABORT();
}

template <typename T>
template <typename T1>
//typename std::enable_if<std::is_same<T1, double>::value,
//                        const RoadOdometry<double>>::type
const RoadOdometry<T1>
PoseSelector<T>::FindSingleClosestPose(
        const Lane* const lane, const PoseVector<T1>& ego_pose,
        const FrameVelocity<
        std::enable_if_t<std::is_same<T1, double>::value, T1>>&
        ego_velocity,
        const PoseBundle<T1>& traffic_poses, double scan_distance,
        const WhichSide side, T1* distance) {
  DRAKE_DEMAND(lane != nullptr);
  if (distance != nullptr) {
    *distance = (side == WhichSide::kAhead)
                    ? std::numeric_limits<double>::infinity()
                    : -std::numeric_limits<double>::infinity();
  }
  const RoadOdometry<double> default_result = set_default_odometry(lane, side);
  const GeoPosition ego_geo_position{ego_pose.get_translation().x(),
                                     ego_pose.get_translation().y(),
                                     ego_pose.get_translation().z()};
  const LanePosition ego_lane_position =
      lane->ToLanePosition(ego_geo_position, nullptr, nullptr);

  // Get the ego car's velocity and lane direction in the trial lane.
  const RoadOdometry<double> road_odom({lane, ego_lane_position}, ego_velocity);
  const double ego_sigma_v = PoseSelector<double>::GetSigmaVelocity(road_odom);
  LaneDirection lane_direction(lane, ego_sigma_v >= 0. /* with_s */);

  // Compute the s-direction of the ego car and its direction of travel.
  // N.B. `distance_scanned` is a net distance, so will always increase.
  double distance_scanned{};
  if (side == WhichSide::kAhead) {
    distance_scanned = (lane_direction.with_s)
                           ? -ego_lane_position.s()
                           : ego_lane_position.s() - lane->length();
  } else {
    distance_scanned = (lane_direction.with_s)
                           ? ego_lane_position.s() - lane->length()
                           : -ego_lane_position.s();
  }
  RoadOdometry<double> result = default_result;
  while (distance_scanned < scan_distance) {
    double distance_increment{};
    for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {
      const Isometry3<double> traffic_iso = traffic_poses.get_pose(i);
      const GeoPosition traffic_geo_position(traffic_iso.translation().x(),
                                             traffic_iso.translation().y(),
                                             traffic_iso.translation().z());

      if (ego_geo_position == traffic_geo_position) continue;
      if (!IsWithinLane(traffic_geo_position, lane_direction.lane)) continue;

      const LanePosition traffic_lane_position =
          lane_direction.lane->ToLanePosition(traffic_geo_position, nullptr,
                                              nullptr);
      switch (side) {
        case WhichSide::kAhead: {
          // Ignore traffic behind the ego car when the two share the same lane.
          // For all other lanes, we will be traversing forward from the current
          // one, with respect to the car's direction.
          if (distance_scanned <= 0.) {
            if ((lane_direction.with_s &&
                 traffic_lane_position.s() <= ego_lane_position.s()) ||
                (!lane_direction.with_s &&
                 traffic_lane_position.s() >= ego_lane_position.s())) {
              continue;
            }
          }
          // Keep positions that are closer than any other found so far.
          if ((lane_direction.with_s &&
               traffic_lane_position.s() <= result.pos.s()) ||
              (!lane_direction.with_s &&
               traffic_lane_position.s() >= result.pos.s())) {
            result = RoadOdometry<double>(
              {lane_direction.lane, traffic_lane_position},
              traffic_poses.get_velocity(i));
            distance_increment =
                (lane_direction.with_s)
                ? result.pos.s()
                : (lane_direction.lane->length() - result.pos.s());
          }
        }
        case WhichSide::kBehind: {
          // Ignore traffic ahead of the ego car when the two share the same
          // lane.  For all other lanes, we will be traversing backward from the
          // current one, with respect to the car's direction.
          if (distance_scanned <= 0.) {
            if ((lane_direction.with_s &&
                 traffic_lane_position.s() > ego_lane_position.s()) ||
                (!lane_direction.with_s &&
                 traffic_lane_position.s() < ego_lane_position.s())) {
              continue;
            }
          }
          // Keep positions that are closer than any other found so far.
          if ((lane_direction.with_s &&
               traffic_lane_position.s() > result.pos.s()) ||
              (!lane_direction.with_s &&
               traffic_lane_position.s() < result.pos.s())) {
            result = RoadOdometry<double>(
                {lane_direction.lane, traffic_lane_position},
                traffic_poses.get_velocity(i));
            distance_increment =
                (lane_direction.with_s)
                    ? (lane_direction.lane->length() - result.pos.s())
                    : result.pos.s();
          }
        }
      }
    }
    if (std::abs(result.pos.s()) < std::numeric_limits<double>::infinity()) {
      // Figure out whether or not the result is within scan_distance.
      if (distance_scanned + distance_increment < scan_distance) {
        if (distance != nullptr) {
          *distance = distance_scanned + distance_increment;
        }
        return result;
      }
    }
    // Increment distance_scanned.
    distance_scanned += lane_direction.lane->length();

    // Obtain the next lane_direction in the scanned sequence.
    get_default_ongoing_lane(&lane_direction);
    if (lane_direction.lane == nullptr) return result;  // Infinity.
  }
  return default_result;
}

template <typename T>
template <typename T1>
//typename std::enable_if<!std::is_same<T1, double>::value,
//                        const RoadOdometry<double>>::type
const RoadOdometry<T1>
PoseSelector<T>::FindSingleClosestPose(
    const Lane* const lane, const PoseVector<T1>& ego_pose,
    const FrameVelocity<
    std::enable_if_t<!std::is_same<T1, double>::value, T1>>&
    ego_velocity,
    const PoseBundle<T1>& traffic_poses, double scan_distance,
    const WhichSide side, T1* distance) {
  DRAKE_ABORT();
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
  return (distance == 0. &&
          pos.r() >= lane->lane_bounds(pos.s()).r_min &&
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
    const Lane* const lane, const WhichSide side) {
  const double infinite_distance =
      (side == WhichSide::kAhead) ? std::numeric_limits<double>::infinity()
                                  : -std::numeric_limits<double>::infinity();
  const RoadPosition default_road_position(lane, {infinite_distance, 0., 0.});
  return {default_road_position, FrameVelocity<T>()};
}

// These instantiations must match the API documentation in
// pose_selector.h.
template class PoseSelector<double>;
template class PoseSelector<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
