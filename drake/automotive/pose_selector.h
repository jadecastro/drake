#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// Contains the position of the vehicle with respect to a lane in a road, along
/// with its velocity vector in the world frame.
template <typename T>
struct RoadOdometry {
  /// Default constructor.
  RoadOdometry() = default;
  /// Fully-parameterized constructor.
  RoadOdometry(const maliput::api::RoadPosition& road_position,
               const systems::rendering::FrameVelocity<T>& frame_velocity)
      : lane(road_position.lane), pos(road_position.pos), vel(frame_velocity) {}

  const maliput::api::Lane* lane{};
  maliput::api::LanePosition pos{};
  systems::rendering::FrameVelocity<T> vel{};
};

/// Specifies whether to assess the cars ahead or behind the ego car at its
/// current orientation with respect to its lane.
enum class WhichSide { kAhead = 0, kBehind = 1 };

///
// TODO(jadecastro): There is some repetitive computations here, making this
// computation O(m^2 * n), where m is the total number of lanes in the road and
// n is the number of vehicles in the road.
class PoseSelector {
 public:
  typedef typename std::pair<const double, const maliput::api::LaneEnd>
      LaneEndDistance;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseSelector)
  PoseSelector() = delete;

  /// Returns the leading and trailing vehicles that have closest
  /// `s`-coordinates in a given @p traffic_lane to an ego vehicle as if the ego
  /// vehicle, at its current `s`-position, is considered to be in @p lane.
  /// This function is used, for instance, as logic for lane-change planners
  /// (e.g. MOBIL).  The ego car's pose @ego_pose, its geo-space velocity @p
  /// ego_velocity and the poses of the traffic cars (@p traffic_poses) are
  /// provided.  The parameter @p scan_distance determines the distance along
  /// the sequence of lanes to scan before declaring the car is at infinite
  /// distance ahead (resp. behind).  If no leading/trailing vehicles are seen
  /// within @p traffic_lane, `s`-positions are taken to be at infinite
  /// distances away from the ego car.  If a non-null @p distances is given,
  /// then it is populated with a pair of distances (ahead and behind) that are
  /// closest.  Infinite distances are returned if no traffic cars are found.
  ///
  /// @return A pair of leading/trailing RoadOdometries. Note that when no
  /// vehicle is detected in front of (resp. behind) the ego vehicle, the
  /// respective RoadPosition will contain an `s`-value of positive
  /// (resp. negative) infinity (`std::numeric_limits<double>::infinity()`).
  /// Any traffic poses that are redunant with `ego_pose` (i.e. have the same
  /// RoadPosition as the ego car) are discarded.
  ///
  /// The RoadGeometry from which @p lane is drawn is required to have default
  /// branches set for all branches in the road network.
  static const std::pair<const RoadOdometry<double>, const RoadOdometry<double>>
  FindClosestPair(const maliput::api::Lane* const lane,
                  const systems::rendering::PoseVector<double>& ego_pose,
                  const systems::rendering::FrameVelocity<double>& ego_velocity,
                  const systems::rendering::PoseBundle<double>& traffic_poses,
                  double scan_distance,
                  std::pair<double, double>* distances = nullptr);

  /// Returns the leading traffic vehicle that has the closest `s`-coordinates
  /// in a given @p traffic_lane to an ego vehicle as if the ego vehicle, at its
  /// current `s`-position, is considered to be in @p road.  This function is
  /// used, for instance, as logic for merge planners.  The traffic vehicle can
  /// either be in the same lane as the ego vehicle or in a lane that eventually
  /// leads, within a distance horizon, into the same lane as the ego.  This
  /// horizon is the `scan_distance` of the ego car, multiplied by the
  /// ratio between the along-lane speed of the traffic vehicle to that of the
  /// ego vehicle.  Hence, at a certain distance away from a merge-point, we
  /// ignore slower traffic that will approach a merge-point much later than the
  /// ego car, yet retain traffic vehicles that are faster but further away from
  /// the merge point.
  ///
  /// The RoadGeometry from which @p lane is drawn is required to have default
  /// branches set for all branches in the road network.  The ego car's pose
  /// @ego_pose, its geo-space velocity @p ego_velocity and the poses of the
  /// traffic cars (@p traffic_poses) are provided.  The parameter @p
  /// scan_distance determines the distance along the sequence of lanes to
  /// scan.  If no leading vehicle is seen within @p traffic_lane, `s`-positions
  /// are taken to be at infinite distances away from the ego car.  If a
  /// non-null @p distance is given, then it is populated with the distance to
  /// the car distance that are closest.  Infinite distances are returned if no
  /// leading traffic car is found.
  ///
  /// @return The RoadOdometry for the lead vehicle. Note that when no leading
  /// vehicle is detected, the RoadPosition will contain an `s`-value of
  /// positive infinity (`std::numeric_limits<double>::infinity()`).  Any
  /// traffic poses that are redunant with `ego_pose` (i.e. have the same
  /// RoadPosition as the ego car) are discarded.
  static const RoadOdometry<double> FindSingleClosestAheadAndInBranches(
      const maliput::api::RoadGeometry& road,
      const systems::rendering::PoseVector<double>& ego_pose,
      const systems::rendering::FrameVelocity<double>& ego_velocity,
      const systems::rendering::PoseBundle<double>& traffic_poses,
      double scan_distance, double* distance = nullptr);

  /// Same as PoseSelector::FindClosestPair() except that: (1) it only considers
  /// the ego car's lane and (2) it returns a single the RoadOdometry of either
  /// the vehicle ahead (WhichSide::kAhead) or behind (WhichSide::kBehind).
  ///
  /// Note that when no car is detected in front of the ego car, the returned
  /// RoadOdometry will contain an `s`-value of
  /// `std::numeric_limits<double>::infinity()`.
  static const RoadOdometry<double> FindSingleClosestPose(
      const maliput::api::Lane* const lane,
      const systems::rendering::PoseVector<double>& ego_pose,
      const systems::rendering::FrameVelocity<double>& ego_velocity,
      const systems::rendering::PoseBundle<double>& traffic_poses,
      double scan_distance, const WhichSide side,
      double* distance = nullptr);

  /// Extracts the vehicle's `s`-direction velocity based on its RoadOdometry @p
  /// road_odom in the Lane coordinate frame.  Assumes the road has zero
  /// elevation and superelevation.
  //
  // TODO(jadecastro): Generalize to three-dimensional rotations.
  static const maliput::api::IsoLaneVelocity GetIsoLaneVelocity(
      const maliput::api::RoadPosition& road_position,
      const systems::rendering::FrameVelocity<double>& velocity);

  // Returns `true` if within the given lane, and `false` otherwise.  @p
  // geo_position is the geo-space coordinates of the vehicle, and @p lane is
  // the lane within which `geo_position` will be checked for membership.
  static bool IsWithinLane(const maliput::api::GeoPosition& geo_position,
                           const maliput::api::Lane* lane);

 private:
  // Returns the closest pose to the ego car given a `road`, a PoseBundle of
  // `traffic_poses`, the along-lane velocity of the ego vehicle `ego_sigma_v`
  // and a set of `branches` to be checked.  If `headway_distance` is given,
  // then it compares this value to the result obtained across branches, and
  // returns whichever value is smaller.  The return value is the same as
  // PoseSelector::FindSingleClosestPose().
  static const RoadOdometry<double> FindSingleClosestAcrossBranches(
      const maliput::api::RoadGeometry& road,
      const systems::rendering::PoseBundle<double>& traffic_poses,
      double ego_sigma_v, const std::vector<LaneEndDistance>& branches,
      double* headway_distance = nullptr);

  // Returns the vector of branches along the sequence of default road segments
  // in a `road`, up to a given `scan_distance` in the ego vehicle's current
  // lane, given its PoseVector `ego_pose` and FrameVelocity `ego_velocity`. A
  // vector of LaneEndDistance is returned, whose elements are pairs where the
  // first entry is the distance along the s-coordinate from the ego vehicle to
  // the branch and second entry is the LaneEnd describing the branch.
  static const std::vector<LaneEndDistance> FindBranches(
      const maliput::api::RoadGeometry& road,
      const systems::rendering::PoseVector<double>& ego_pose,
      const systems::rendering::FrameVelocity<double>& ego_velocity,
      double scan_distance);

  // Retrieves the lane and direction of travel based on the vehicle's current
  // velocity `iso_velocity` within a given `lane`.
  static const LaneDirection get_lane_direction(
      const maliput::api::IsoLaneVelocity& iso_velocity,
      const maliput::api::Lane* lane);

  // Returns true if `lane_end0` is adjacent with `lane_end1`, and false
  // otherwise.  Two lane ends are said to be adjacent if they share the same
  // coordinates at the ends (i.e. kStart, kFinish) under consideration.
  static bool IsAdjacent(const maliput::api::LaneEnd& lane_end0,
                         const maliput::api::LaneEnd& lane_end1);

  // Returns true if `lane0` has the same endpoints as `lane1`, and false
  // otherwise.  Two lane ends are said to be adjacent if they share the same
  // coordinates for any two ends.
  static bool IsEqual(const maliput::api::Lane* const lane0,
                      const maliput::api::Lane* const lane1);

  // Mutates `lane_direction` according to the next default lane based on the
  // current lane and travel direction contained within `lane_direction`.
  // `lane_direction` contains a null pointer in the `lane` field if no default
  // branch is found.
  static std::unique_ptr<maliput::api::LaneEnd> get_default_ongoing_lane(
      LaneDirection* lane_direction);

  // Returns a LaneEndSet consisting of all LaneEnds attached to the provided
  // lane (specified in `lane_direction`) corresponding to all branches
  // connected to the end of the lane is reached when traveling in the `with_s`
  // direction specified within `lane_direction`.  The return value contains a
  // null pointer if no default branch is found.
  static const maliput::api::LaneEndSet* GetIncomingLaneEnds(
      const LaneDirection& lane_direction);

  // Assign the with default positions extending to, respectively, positive and
  // negative infinity and with zero velocities.
  static const RoadOdometry<double> set_default_odometry(
      const maliput::api::Lane* const Lane, const WhichSide side);
};


}  // namespace automotive
}  // namespace drake
