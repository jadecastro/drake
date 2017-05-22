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

const std::pair<const RoadOdometry<double>, const RoadOdometry<double>>
PoseSelector::FindClosestPair(const Lane* const lane,
                              const PoseVector<double>& ego_pose,
                              const FrameVelocity<double>& ego_velocity,
                              const PoseBundle<double>& traffic_poses,
                              double scan_distance,
                              std::pair<double, double>* distances) {
  double distance_ahead{};
  double distance_behind{};
  const RoadOdometry<double> result_leading =
      FindSingleClosestPose(lane, ego_pose, ego_velocity, traffic_poses,
                            scan_distance, WhichSide::kAhead, &distance_ahead);
  const RoadOdometry<double> result_trailing = FindSingleClosestPose(
      lane, ego_pose, ego_velocity, traffic_poses, scan_distance,
      WhichSide::kBehind, &distance_behind);
  if (distances != nullptr) {
    *distances = std::make_pair(distance_ahead, distance_behind);
  }
  return std::make_pair(result_leading, result_trailing);
}

const RoadOdometry<double> PoseSelector::FindSingleClosestPose(
    const Lane* const lane, const PoseVector<double>& ego_pose,
    const FrameVelocity<double>& ego_velocity,
    const PoseBundle<double>& traffic_poses, double scan_distance,
    const WhichSide side, double* distance) {
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
  std::cerr << " lane " << lane->id().id << std::endl;

  // Get the ego car's velocity and lane direction in the trial lane.
  const IsoLaneVelocity ego_lane_velocity =
      GetIsoLaneVelocity({lane, ego_lane_position}, ego_velocity);
  LaneDirection lane_direction(lane,
                               ego_lane_velocity.sigma_v >= 0. /* with_s */);

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
  while (distance_scanned < scan_distance) {
    RoadOdometry<double> result = default_result;
    double distance_increment{};
    for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {
      const Isometry3<double> traffic_iso = traffic_poses.get_pose(i);
      const GeoPosition traffic_geo_position(traffic_iso.translation().x(),
                                             traffic_iso.translation().y(),
                                             traffic_iso.translation().z());

      if (ego_geo_position == traffic_geo_position) continue;
      std::cerr << "   traffic_geo_position.y() "
                << traffic_geo_position.y() << std::endl;
      std::cerr << "   lane_direction.lane " << lane_direction.lane->id().id
                << std::endl;
      if (!IsWithinLane(traffic_geo_position, lane_direction.lane)) continue;

      const LanePosition traffic_lane_position =
          lane_direction.lane->ToLanePosition(traffic_geo_position, nullptr,
                                              nullptr);
      switch (side) {
        case WhichSide::kAhead: {
          std::cerr << " (query) AHEAD " << std::endl;
          // Ignore traffic behind the ego car when the two share the same lane.
          // For all other lanes, we will be traversing forward from the current
          // one, with respect to the car's direction.
          if (distance_scanned <= 0.) {
            if ((lane_direction.with_s &&
                 traffic_lane_position.s() < ego_lane_position.s()) ||
                (!lane_direction.with_s &&
                 traffic_lane_position.s() > ego_lane_position.s())) {
              continue;
            }
          }
          // Keep positions that are closer than any other found so far.
          std::cerr << " traffic_lane_position.s() " << traffic_lane_position.s() << std::endl;
          if ((lane_direction.with_s &&
               traffic_lane_position.s() < result.pos.s()) ||
              (!lane_direction.with_s &&
               traffic_lane_position.s() > result.pos.s())) {
            result = RoadOdometry<double>(
                {lane_direction.lane, traffic_lane_position},
                traffic_poses.get_velocity(i));
            std::cerr << " result.pos.s() " << result.pos.s() << std::endl;
            distance_increment =
                (lane_direction.with_s)
                    ? result.pos.s()
                    : (lane_direction.lane->length() - result.pos.s());
          }
        }
        case WhichSide::kBehind: {
          std::cerr << " (query) BEHIND " << std::endl;
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
          std::cerr << " traffic_lane_position.s() " << traffic_lane_position.s() << std::endl;
          if ((lane_direction.with_s &&
               traffic_lane_position.s() > result.pos.s()) ||
              (!lane_direction.with_s &&
               traffic_lane_position.s() < result.pos.s())) {
            result = RoadOdometry<double>(
                {lane_direction.lane, traffic_lane_position},
                traffic_poses.get_velocity(i));
            std::cerr << " result.pos.s() " << result.pos.s() << std::endl;
            distance_increment =
                (lane_direction.with_s)
                    ? (lane_direction.lane->length() - result.pos.s())
                    : result.pos.s();
          }
        }
      }
    }
    std::cerr << " result.pos.s() " << result.pos.s() << std::endl;
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

const IsoLaneVelocity PoseSelector::GetIsoLaneVelocity(
    const RoadPosition& road_position, const FrameVelocity<double>& velocity) {
  const double large_s_value{1e9};
  const LanePosition sat_position{
      math::saturate(road_position.pos.s(), -large_s_value, large_s_value),
      road_position.pos.r(), road_position.pos.h()};
  const maliput::api::Rotation rot =
      road_position.lane->GetOrientation(sat_position);
  const Vector3<double>& vel = velocity.get_velocity().translational();
  return {vel(0) * std::cos(rot.yaw()) + vel(1) * std::sin(rot.yaw()),
          -vel(0) * std::sin(rot.yaw()) + vel(1) * std::cos(rot.yaw()), 0.};
}

bool PoseSelector::IsWithinLane(const GeoPosition& geo_position,
                                const Lane* const lane) {
  double distance;
  lane->ToLanePosition(geo_position, nullptr, &distance);
  std::cerr << " IsWithinLane::distance  " << distance << std::endl;
  return distance == 0.;
}

std::unique_ptr<LaneEnd> PoseSelector::get_default_ongoing_lane(
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

const RoadOdometry<double> PoseSelector::set_default_odometry(
    const Lane* const lane, const WhichSide side) {
  const double infinite_distance =
      (side == WhichSide::kAhead) ? std::numeric_limits<double>::infinity()
                                  : -std::numeric_limits<double>::infinity();
  const RoadPosition default_road_position(lane, {infinite_distance, 0., 0.});
  return {default_road_position, FrameVelocity<double>()};
}

}  // namespace automotive
}  // namespace drake
