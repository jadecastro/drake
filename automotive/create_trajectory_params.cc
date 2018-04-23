#include "drake/automotive/create_trajectory_params.h"

#include <algorithm>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/segment.h"

namespace drake {
namespace automotive {

enum class SpeedSetting { kConstant, kIncreasing };

namespace {
// A figure-eight.  One loop has a radius of @p radius - @p inset,
// the other loop has a radius of @p radius + @p inset.
std::pair<Curve2<double>, std::vector<double>> MakeCurve(
    double radius, double inset) {
  // TODO(jwnimmer-tri) This function will be rewritten once we have
  // proper splines.  Don't try too hard to understand it.  Run the
  // demo to see it first, and only then try to understand the code.

  typedef Curve2<double>::Point2 Point2d;
  std::vector<Point2d> waypoints;
  std::vector<double> waypoint_headings;

  // Start (0, +i).
  // Straight right to (+r, +i).
  // Loop around (+i, +r).
  // Straight back to (+i, 0).
  waypoints.push_back({0.0, inset});
  waypoint_headings.push_back(0.);
  for (int theta_deg = -90; theta_deg <= 180; theta_deg += 30.) {
    std::cout << " theta_deg " << theta_deg << std::endl;
    const Point2d center{radius, radius};
    const double theta = theta_deg * M_PI / 180.0;
    waypoint_headings.push_back(theta + M_PI_2);
    const Point2d direction{std::cos(theta), std::sin(theta)};
    waypoints.push_back(center + (direction * (radius - inset)));
  }
  waypoints.push_back({inset, 0.0});
  waypoint_headings.push_back(-M_PI_2);

  // Start (+i, 0).
  // Straight down to (+i, -r).
  // Loop around (-r, +i).
  // Straight back to start (implicitly via segment to waypoints[0]).
  for (int theta_deg = 0; theta_deg >= -270; theta_deg -= 30.) {
    const Point2d center{-radius, -radius};
    const double theta = theta_deg * M_PI / 180.0;
    waypoint_headings.push_back(theta - M_PI_2);
    const Point2d direction{std::cos(theta), std::sin(theta)};
    waypoints.push_back(center + (direction * (radius + inset)));
  }

  // Many copies.
  const int kNumCopies = 100;
  std::vector<Point2d> looped_waypoints;
  std::vector<double> looped_waypoint_headings;
  for (int copies = 0; copies < kNumCopies; ++copies) {
    std::copy(waypoints.begin(), waypoints.end(),
              std::back_inserter(looped_waypoints));
    std::copy(waypoint_headings.begin(), waypoint_headings.end(),
              std::back_inserter(looped_waypoint_headings));
  }
  looped_waypoints.push_back(waypoints.front());
  looped_waypoint_headings.push_back(waypoint_headings.front());

  return std::make_pair(Curve2<double>(looped_waypoints),
                        looped_waypoint_headings);
}
}  // anonymous namespace

std::tuple<Curve2<double>, double, double, std::vector<double>>
CreateTrajectoryParams(int index) {
  // The possible curves to trace (lanes).
  static const std::vector<std::pair<Curve2<double>, std::vector<double>>> curves{
    MakeCurve(40.0, 0.0),  // BR
    MakeCurve(40.0, 4.0),  // BR
    MakeCurve(40.0, 8.0),
  };

  // Magic car placement to make a good visual demo.
  const auto& curve = curves[index % curves.size()].first;
  const auto& headings = curves[index % curves.size()].second;
  const double start_time = (index / curves.size()) * 0.8;
  const double kSpeed = 20.0;
  return std::make_tuple(curve, kSpeed, start_time, headings);
}

AgentTrajectory CreateAgentTrajectoryParams(int index) {
  const SpeedSetting speed_setting = SpeedSetting::kIncreasing;
  const std::tuple<Curve2<double>, double, double, std::vector<double>> curve =
      CreateTrajectoryParams(index);
  const std::vector<Curve2<double>::Point2> points = std::get<0>(curve).waypoints();
  const std::vector<double> headings = std::get<3>(curve);
  DRAKE_DEMAND(points.size() == headings.size());

  const double speed = std::get<1>(curve);
  std::vector<Eigen::Isometry3d> waypoints{};
  for (int i{0}; i < static_cast<int>(points.size()); i++) {
    const Eigen::Translation<double, 3> translation(
        Eigen::Vector3d{points[i].x(), points[i].y(), 0.});
    Eigen::Isometry3d pose(translation);
    const Eigen::Vector3d rpy{0., 0., headings[i]};
    const Eigen::Quaternion<double> rotation =
        math::RollPitchYaw<double>(rpy).ToQuaternion();
    pose.rotate(rotation);
    waypoints.push_back(pose);
  }

  if (speed_setting == SpeedSetting::kConstant) {
    return AgentTrajectory::MakeCubicFromWaypoints(waypoints, speed);
  } else if (speed_setting == SpeedSetting::kIncreasing) {
    std::vector<double> speeds(waypoints.size());
    double new_speed = speed / 5.;
    for (int i{0}; i < static_cast<int>(waypoints.size()); i++) {
      speeds[i] = new_speed;
      new_speed += 1.;
    }
    return AgentTrajectory::MakeCubicFromWaypoints(waypoints, speeds);
  }
}

std::tuple<Curve2<double>, double, double> CreateTrajectoryParamsForDragway(
    const maliput::dragway::RoadGeometry& road_geometry, int index,
    double speed, double start_position) {
  const maliput::api::Segment* segment = road_geometry.junction(0)->segment(0);
  DRAKE_DEMAND(index < segment->num_lanes());
  const maliput::api::Lane* lane = segment->lane(index);
  const maliput::api::GeoPosition start_geo_position =
      lane->ToGeoPosition(maliput::api::LanePosition(
          0 /* s */, 0 /* r */, 0 /* h */));
  const maliput::api::GeoPosition end_geo_position =
      lane->ToGeoPosition(maliput::api::LanePosition(
          lane->length() /* s */, 0 /* r */, 0 /* h */));
  std::vector<Curve2<double>::Point2> waypoints;
  waypoints.push_back({start_geo_position.x(), start_geo_position.y()});
  waypoints.push_back({end_geo_position.x(), end_geo_position.y()});
  Curve2<double> curve(waypoints);
  return std::make_tuple(curve, speed, start_position);
}

AgentTrajectory CreateAgentTrajectoryForDragway(
    const maliput::dragway::RoadGeometry& road_geometry, int index,
    double speed, double start_position) {
  const std::tuple<Curve2<double>, double, double> curve =
      CreateTrajectoryParamsForDragway(road_geometry, index, speed, start_position);
  const std::vector<Curve2<double>::Point2> points = std::get<0>(curve).waypoints();
  std::vector<Eigen::Isometry3d> waypoints{};
  waypoints.push_back(Eigen::Isometry3d{Eigen::Translation<double, 3>(
      Eigen::Vector3d{points[0].x(), points[0].y(), 0.})});
  waypoints.push_back(Eigen::Isometry3d{Eigen::Translation<double, 3>(
      Eigen::Vector3d{points[1].x(), points[1].y(), 0.})});
  return AgentTrajectory::MakeCubicFromWaypoints(waypoints, speed);
}

}  // namespace automotive
}  // namespace drake
