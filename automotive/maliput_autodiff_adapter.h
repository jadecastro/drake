#pragma once

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/autodiff.h"

namespace drake {
namespace automotive {
namespace autodiff {

maliput::api::LanePositionT<AutoDiffXd> ToLanePositionT(
    const maliput::api::Lane* lane,
    const maliput::api::GeoPositionT<AutoDiffXd>& geo_pos,
    maliput::api::GeoPositionT<AutoDiffXd>* nearest_point,
    AutoDiffXd* distance);

maliput::api::LanePositionT<double> ToLanePositionT(
    const maliput::api::Lane* lane,
    const maliput::api::GeoPositionT<double>& geo_pos,
    maliput::api::GeoPositionT<double>* nearest_point,
    double* distance);

maliput::api::GeoPositionT<AutoDiffXd> ToGeoPositionT(
    const maliput::api::Lane* lane,
    const maliput::api::LanePositionT<AutoDiffXd>& lane_pos);

maliput::api::GeoPositionT<double> ToGeoPositionT(
    const maliput::api::Lane* lane,
    const maliput::api::LanePositionT<double>& lane_pos);

}  // namespace autodiff
}  // namespace automotive
}  // namespace drake
