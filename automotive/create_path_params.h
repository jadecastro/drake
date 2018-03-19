#pragma once

// TODO(jwnimmer-tri) This file provides trajectories in support of demos.
// This data should come from files loaded at runtime, instead.

#include <memory>
#include <tuple>

#include "drake/automotive/curve2.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/path_following_car.h"

namespace drake {
namespace automotive {

/**
 * Creates PathFollowingCar constructor demo arguments.  The details of the
 * trajectory are not documented / promised by this API.
 *
 * @param index Selects which pre-programmed trajectory to use.
 *
 * @return tuple of curve, speed, start_time
 */
std::tuple<Curve2<double>, double, double> CreatePathParams(int index);

/**
 * Creates PathFollowingCar constructor demo arguments for a vehicle on a dragway.
 * The details of the trajectory are not documented / promised by this API.
 *
 * @param road_geometry The dragway upon which the PathFollowingCar will travel.
 *
 * @param index The lane index within the provided `road_geometry`.
 *
 * @param speed The speed of the vehicle.
 *
 * @param start_time The time when the vehicle should start driving.
 *
 * @return tuple of curve, speed, start_time
 */
std::tuple<Curve2<double>, double, double> CreatePathParamsForDragway(
    const maliput::dragway::RoadGeometry& road_geometry, int index,
    double speed, double start_time);

}  // namespace automotive
}  // namespace drake
