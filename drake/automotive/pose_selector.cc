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

const RoadOdometry<double> PoseSelector::FindSingleClosestAheadAndInBranches(
    const RoadGeometry& road, const PoseVector<double>& ego_pose,
    const FrameVelocity<double>& ego_velocity,
    const PoseBundle<double>& traffic_poses, double scan_distance,
    double* distance) {
  const GeoPosition geo_position{ego_pose.get_translation().x(),
                                 ego_pose.get_translation().y(),
                                 ego_pose.get_translation().z()};
  const Lane* const lane =
      road.ToRoadPosition(geo_position, nullptr, nullptr, nullptr).lane;

  // Find any leading traffic cars along the same default path as the ego
  // vehicle.
  double headway_distance{};
  const RoadOdometry<double> result_in_path = FindSingleClosestPose(
      lane, ego_pose, ego_velocity, traffic_poses, scan_distance,
      WhichSide::kAhead, &headway_distance);

  const std::vector<LaneEndDistance> branches =
      FindBranches(lane, ego_pose, ego_velocity, scan_distance);

  if (distance != nullptr) *distance = headway_distance;
  if (branches.size() == 0) return result_in_path;

  const RoadPosition ego_position =
      road.ToRoadPosition(geo_position, nullptr, nullptr, nullptr);
  const double ego_sigma_v =
      GetIsoLaneVelocity(ego_position, ego_velocity).sigma_v;

  // Find any leading traffic cars in lanes leading into the ego vehicle's
  // default path.
  double effective_headway{};
  const RoadOdometry<double> result_in_branch = FindSingleClosestAcrossBranches(
      road, traffic_poses, ego_sigma_v, branches, &effective_headway);

  if (distance != nullptr) {
    *distance = std::min(headway_distance, effective_headway);
  }
  if (headway_distance <= effective_headway) {
    return result_in_path;
  } else {
    return result_in_branch;
  }
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
                           ? -ego_lane_position.s
                           : ego_lane_position.s - lane->length();
  } else {
    distance_scanned = (lane_direction.with_s)
                           ? ego_lane_position.s - lane->length()
                           : -ego_lane_position.s;
  }
  while (distance_scanned < scan_distance) {
    RoadOdometry<double> result = default_result;
    double distance_increment{};
    for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {
      const Isometry3<double> traffic_iso = traffic_poses.get_pose(i);
      const GeoPosition traffic_geo_position(traffic_iso.translation().x(),
                                             traffic_iso.translation().y(),
                                             traffic_iso.translation().z());

      if (!IsWithinLane(traffic_geo_position, lane_direction.lane)) continue;
      if (is_equal(ego_geo_position, traffic_geo_position)) continue;

      const LanePosition traffic_lane_position =
          lane_direction.lane->ToLanePosition(traffic_geo_position, nullptr,
                                              nullptr);
      switch (side) {
        case WhichSide::kAhead: {
          // Ignore traffic behind the ego car when the two share the same lane.
          if (distance_scanned <= 0.) {
            if ((lane_direction.with_s &&
                 traffic_lane_position.s < ego_lane_position.s) ||
                (!lane_direction.with_s &&
                 traffic_lane_position.s > ego_lane_position.s)) {
              continue;
            }
          }
          // Keep positions that are closer than any other found so far.
          if ((lane_direction.with_s &&
               traffic_lane_position.s < result.pos.s) ||
              (!lane_direction.with_s &&
               traffic_lane_position.s > result.pos.s)) {
            result = RoadOdometry<double>(
                {lane_direction.lane, traffic_lane_position},
                traffic_poses.get_velocity(i));
            distance_increment =
                (lane_direction.with_s)
                    ? result.pos.s
                    : (lane_direction.lane->length() - result.pos.s);
          }
        }
        case WhichSide::kBehind: {
          // Ignore traffic ahead of the ego car when the two share the same
          // lane.
          if (distance_scanned <= 0.) {
            if ((lane_direction.with_s &&
                 traffic_lane_position.s > ego_lane_position.s) ||
                (!lane_direction.with_s &&
                 traffic_lane_position.s < ego_lane_position.s)) {
              continue;
            }
          }
          // Keep positions that are closer than any other found so far.
          if ((traffic_lane_position.s > result.pos.s) ||
              (!lane_direction.with_s &&
               traffic_lane_position.s < result.pos.s)) {
            result = RoadOdometry<double>(
                {lane_direction.lane, traffic_lane_position},
                traffic_poses.get_velocity(i));
            distance_increment =
                (lane_direction.with_s)
                    ? (lane_direction.lane->length() - result.pos.s)
                    : result.pos.s;
          }
        }
      }
    }
    if (std::abs(result.pos.s) < std::numeric_limits<double>::infinity()) {
      // Figure out whether or not the result is within scan_distance.
      if (distance_scanned + distance_increment < scan_distance) {
        if (distance != nullptr) {
          *distance = distance_scanned + distance_increment;
        }
        return result;
      }
    }
    get_default_ongoing_lane(&lane_direction);
    if (lane_direction.lane == nullptr) return result;  // Infinity.

    // Increment distance_scanned.
    distance_scanned += lane_direction.lane->length();
  }
  return default_result;
}

const RoadOdometry<double> PoseSelector::FindSingleClosestAcrossBranches(
    const RoadGeometry& road, const PoseBundle<double>& traffic_poses,
    double ego_sigma_v, const std::vector<LaneEndDistance>& branches,
    double* headway_distance) {
  DRAKE_DEMAND(headway_distance != nullptr);

  *headway_distance = std::numeric_limits<double>::infinity();
  RoadOdometry<double> result =
      set_default_odometry(nullptr, WhichSide::kAhead);

  for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {
    const Isometry3<double> isometry = traffic_poses.get_pose(i);
    const GeoPosition geo_position(isometry.translation().x(),
                                   isometry.translation().y(),
                                   isometry.translation().z());
    const RoadPosition road_position =
        road.ToRoadPosition(geo_position, nullptr, nullptr, nullptr);

    const Lane* const lane = road_position.lane;
    const LanePosition lane_position = road_position.pos;

    // Get this traffic vehicle's velocity and travel direction in the lane it
    // is occupying.
    const IsoLaneVelocity lane_velocity = GetIsoLaneVelocity(
        {lane, lane_position}, traffic_poses.get_velocity(i));
    LaneDirection lane_direction(lane,
                                 lane_velocity.sigma_v >= 0. /* with_s */);
    double distance_to_this_branch{};
    LaneEnd branch;
    double distance_scanned = (lane_direction.with_s)
                                  ? -lane_position.s
                                  : lane_position.s - lane->length();
    // Determine if any of the traffic cars eventually lead to a branch within a
    // speed- and branch-dependent influence distance horizon.
    for (auto branch_distance : branches) {
      std::tie(distance_to_this_branch, branch) = branch_distance;

      // The distance ahead needed to scan for intersection is assumed equal to
      // the distance scanned in the ego vehicle's lane times the ratio of
      // s-velocity of the traffic car to that of the ego car.  Cars much slower
      // than the ego car are thus phased out closer to the branch-point, while
      // those that are faster remain in scope further away from the
      // branch-point.
      const double distance_to_scan =
          std::abs(lane_velocity.sigma_v / ego_sigma_v) *
          distance_to_this_branch;

      double effective_headway{};
      while (distance_scanned < distance_to_scan) {
        auto lane_direction_pre = lane_direction;
        auto lane_end = get_default_ongoing_lane(&lane_direction);
        if (lane_end == nullptr || lane_direction.lane == nullptr) {
          break;
        }
        if (IsAnyAdjacent(lane_end->lane, branch.lane)) {
          // "Effective headway" is the distance between the traffic vehicle and
          // the ego vehicle, compared relative to their positions with respect
          // to their shared branch point.
          effective_headway =
              distance_to_this_branch -
              (distance_scanned + lane_direction_pre.lane->length());
        }
        if (0. < effective_headway && effective_headway < *headway_distance) {
          *headway_distance = effective_headway;
          result =
              RoadOdometry<double>({lane_direction_pre.lane, lane_position},
                                   traffic_poses.get_velocity(i));
          break;
        }
        // Increment distance_scanned.
        distance_scanned += lane_direction.lane->length();
      }
    }
  }
  return result;
}

const std::vector<PoseSelector::LaneEndDistance> PoseSelector::FindBranches(
    const maliput::api::Lane* const lane, const PoseVector<double>& ego_pose,
    const FrameVelocity<double>& ego_velocity, double scan_distance) {
  DRAKE_DEMAND(lane != nullptr);
  const GeoPosition ego_geo_position{ego_pose.get_translation().x(),
                                     ego_pose.get_translation().y(),
                                     ego_pose.get_translation().z()};
  // Transform the ego car's pose into the lane's frame.
  const LanePosition ego_lane_position =
      lane->ToLanePosition(ego_geo_position, nullptr, nullptr);

  // Get the ego car's velocity and lane direction in the trial lane.
  const IsoLaneVelocity ego_lane_velocity =
      GetIsoLaneVelocity({lane, ego_lane_position}, ego_velocity);
  LaneDirection lane_direction(lane,
                               ego_lane_velocity.sigma_v >= 0. /* with_s */);
  // Obtain any branches starting from the ego vehicle's lane, moving along its
  // direction of travel by an amount equal to scan_distance.
  std::vector<LaneEndDistance> branches;
  double distance_scanned = (lane_direction.with_s)
                                ? -ego_lane_position.s
                                : ego_lane_position.s - lane->length();
  std::unique_ptr<LaneEnd> lane_end;
  while (distance_scanned < scan_distance) {
    // Increment distance_scanned and collect all non-trivial branches as we go.
    distance_scanned += lane_direction.lane->length();
    const LaneEndSet* ends = GetIncomingLaneEnds(lane_direction);
    if (lane_end != nullptr && ends->size() > 1) {
      for (int i = 0; i < ends->size(); ++i) {
        // Store, from the complete list, the LaneEnds that do not belong to the
        // main trunk (lane sequence containing the ego vehicle).
        if (!IsAdjacent(*lane_end, ends->get(i))) {
          branches.emplace_back(std::make_pair(distance_scanned, ends->get(i)));
        }
      }
    }
    lane_end = get_default_ongoing_lane(&lane_direction);
    if (lane_direction.lane == nullptr) break;
  }
  return branches;
}

const IsoLaneVelocity PoseSelector::GetIsoLaneVelocity(
    const RoadPosition& road_position, const FrameVelocity<double>& velocity) {
  const double large_s_value{1e9};
  const LanePosition sat_position{
    math::saturate(-large_s_value, large_s_value, road_position.pos.s),
        road_position.pos.r, road_position.pos.h};
  const maliput::api::Rotation rot =
      road_position.lane->GetOrientation(sat_position);
  const Vector3<double>& vel = velocity.get_velocity().translational();
  return {vel(0) * std::cos(rot.yaw) + vel(1) * std::sin(rot.yaw),
          -vel(0) * std::sin(rot.yaw) + vel(1) * std::cos(rot.yaw), 0.};
}

bool PoseSelector::IsWithinLane(const GeoPosition& geo_position,
                                const Lane* const lane) {
  const LanePosition lane_position =
      lane->ToLanePosition(geo_position, nullptr, nullptr);
  if (lane_position.s < 0. || lane->length() < lane_position.s) {
    return false;
  }
  if (lane_position.r <= lane->lane_bounds(lane_position.s).r_min ||
      lane->lane_bounds(lane_position.s).r_max <= lane_position.r) {
    return false;
  }
  return true;
}

bool PoseSelector::IsAdjacent(const LaneEnd& lane_end0,
                              const LaneEnd& lane_end1) {
  const LanePosition end0 =
      (lane_end0.end == LaneEnd::Which::kStart)
          ? LanePosition(0., 0., 0.)
          : LanePosition(lane_end0.lane->length(), 0., 0.);
  const GeoPosition datum0 = lane_end0.lane->ToGeoPosition(end0);
  const LanePosition end1 =
      (lane_end1.end == LaneEnd::Which::kFinish)
          ? LanePosition(0., 0., 0.)
          : LanePosition(lane_end1.lane->length(), 0., 0.);
  const GeoPosition datum1 = lane_end1.lane->ToGeoPosition(end1);
  return (datum0.x == datum1.x) && (datum0.y == datum1.y) &&
         (datum0.z == datum1.z);
}

bool PoseSelector::IsAnyAdjacent(const Lane* const lane0,
                                 const Lane* const lane1) {
  GeoPosition datum0 = lane0->ToGeoPosition({0., 0., 0.});
  GeoPosition datum1 = lane1->ToGeoPosition({lane1->length(), 0., 0.});
  if ((datum0.x == datum1.x) && (datum0.y == datum1.y) &&
      (datum0.z == datum1.z)) {
    return true;
  }
  datum0 = lane0->ToGeoPosition({lane0->length(), 0., 0.});
  if ((datum0.x == datum1.x) && (datum0.y == datum1.y) &&
      (datum0.z == datum1.z)) {
    return true;
  }
  datum1 = lane1->ToGeoPosition({0., 0., 0.});
  if ((datum0.x == datum1.x) && (datum0.y == datum1.y) &&
      (datum0.z == datum1.z)) {
    return true;
  }
  datum0 = lane0->ToGeoPosition({0., 0., 0.});
  return (datum0.x == datum1.x) && (datum0.y == datum1.y) &&
         (datum0.z == datum1.z);
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

const LaneEndSet* PoseSelector::GetIncomingLaneEnds(
    const LaneDirection& lane_direction) {
  const Lane* lane{lane_direction.lane};
  const bool with_s{lane_direction.with_s};
  return (with_s) ? lane->GetConfluentBranches(LaneEnd::kFinish)
                  : lane->GetConfluentBranches(LaneEnd::kStart);
}

const RoadOdometry<double> PoseSelector::set_default_odometry(
    const Lane* const lane, const WhichSide side) {
  const double infinite_distance =
      (side == WhichSide::kAhead) ? std::numeric_limits<double>::infinity()
                                  : -std::numeric_limits<double>::infinity();
  const RoadPosition default_road_position(lane, {infinite_distance, 0., 0.});
  return {default_road_position, FrameVelocity<double>()};
}

bool PoseSelector::is_equal(const GeoPosition& geo_position0,
                            const GeoPosition& geo_position1) {
  return geo_position0.x == geo_position1.x &&
         geo_position0.y == geo_position1.y &&
         geo_position0.z == geo_position1.z;
}

}  // namespace automotive
}  // namespace drake
