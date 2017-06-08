#include "drake/automotive/maliput/api/lane_data.h"

#include <iostream>

namespace drake {
namespace maliput {
namespace api {

std::ostream& operator<<(std::ostream& out, const LaneEnd::Which& which_end) {
  return out << (which_end == LaneEnd::kStart ? "start" : "finish");
}

std::ostream& operator<<(std::ostream& out, const Rotation& rotation) {
  return out << "(roll = " << rotation.roll()
             << ", pitch = " << rotation.pitch()
             << ", yaw = " << rotation.yaw() << ")";
}

std::ostream& operator<<(std::ostream& out, const GeoPosition& geo_position) {
  return out << "(x = " << geo_position.x() << ", y = " << geo_position.y()
      << ", z = " << geo_position.z() << ")";
}

std::ostream& operator<<(std::ostream& out, const LanePosition& lane_position) {
  return out << "(s = " << lane_position.s() << ", r = " << lane_position.r()
      << ", h = " << lane_position.h() << ")";
}

// These instantiations must match the API documentation in lane_data.h.
template class GeoPositionWithAutoDiff<double>;
template class GeoPositionWithAutoDiff<AutoDiffXd>;

template class LanePositionWithAutoDiff<double>;
template class LanePositionWithAutoDiff<AutoDiffXd>;

}  // namespace api
}  // namespace maliput
}  // namespace drake
