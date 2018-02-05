#include "drake/automotive/maliput_autodiff_adapter.h"

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/autodiff.h"
#include "drake/common/autodiffxd_make_coherent.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace automotive {
namespace autodiff {

using maliput::api::GeoPosition;
using maliput::api::GeoPositionT;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::LanePositionT;
using maliput::api::Rotation;

namespace {

static constexpr double kDelta = 1e-9;

// Populate the partial derivatives of LanePosition with respect to x, y, z from
// a GeoPosition using the method of finite differences.
LanePositionT<AutoDiffXd> ToLanePositionFiniteDifferences(
    const Lane* lane, const GeoPosition& gp, const GeoPositionT<AutoDiffXd>& gp_ad) {
  Eigen::Matrix3d derivs;
  GeoPosition gp_p = gp;
  GeoPosition gp_m = gp;
  gp_p.set_x(gp.x() + 0.5 * kDelta);
  gp_m.set_x(gp.x() - 0.5 * kDelta);
  const LanePosition lp_x_p = lane->ToLanePosition(gp_p, nullptr, nullptr);
  const LanePosition lp_x_m = lane->ToLanePosition(gp_m, nullptr, nullptr);
  derivs.col(0) = Vector3<double>{(lp_x_p.s() - lp_x_m.s()) / kDelta,
                                  (lp_x_p.r() - lp_x_m.r()) / kDelta,
                                  (lp_x_p.h() - lp_x_m.h()) / kDelta};

  gp_p = gp;
  gp_m = gp;
  gp_p.set_y(gp.y() + 0.5 * kDelta);
  gp_m.set_y(gp.y() - 0.5 * kDelta);
  const LanePosition lp_y_p = lane->ToLanePosition(gp_p, nullptr, nullptr);
  const LanePosition lp_y_m = lane->ToLanePosition(gp_m, nullptr, nullptr);
  derivs.col(1) = Vector3<double>{(lp_y_p.s() - lp_y_m.s()) / kDelta,
                                  (lp_y_p.r() - lp_y_m.r()) / kDelta,
                                  (lp_y_p.h() - lp_y_m.h()) / kDelta};

  // Do only a positive perturbation in the z-axis because the minimum HBound is
  // zero.
  gp_p = gp;
  gp_m = gp;
  gp_p.set_z(gp.z() + 0.5 * kDelta);
  gp_m.set_z(gp.z() - 0.5 * kDelta);
  const LanePosition lp_z_p = lane->ToLanePosition(gp_p, nullptr, nullptr);
  const LanePosition lp_z_m = lane->ToLanePosition(gp_m, nullptr, nullptr);
  derivs.col(2) = Vector3<double>{(lp_z_p.s() - lp_z_m.s()) / kDelta,
                                  (lp_z_p.r() - lp_z_m.r()) / kDelta,
                                  (lp_z_p.h() - lp_z_m.h()) / kDelta};

  Eigen::MatrixXd gp_derivs(3, gp_ad.x().derivatives().size());
  gp_derivs.row(0) = gp_ad.x().derivatives();
  gp_derivs.row(1) = gp_ad.y().derivatives();
  gp_derivs.row(2) = gp_ad.z().derivatives();
  LanePosition lp = lane->ToLanePosition(gp, nullptr, nullptr);
  LanePositionT<AutoDiffXd> lp_result;
  lp_result.set_s(AutoDiffXd(lp.s(), derivs.row(0) * gp_derivs));
  lp_result.set_r(AutoDiffXd(lp.r(), derivs.row(1) * gp_derivs));
  lp_result.set_h(AutoDiffXd(lp.h(), derivs.row(2) * gp_derivs));

  return lp_result;
}

// Populate the partial derivatives of GeoPosition with respect to s, r, h from
// a LanePosition using the method of finite differences.
GeoPositionT<AutoDiffXd> ToGeoPositionFiniteDifferences(
    const Lane* lane, const LanePosition& lp, const LanePositionT<AutoDiffXd>& lp_ad) {
  Eigen::Matrix3d derivs;
  LanePosition lp_p = lp;
  LanePosition lp_m = lp;
  lp_p.set_s(lp.s() + 0.5 * kDelta);
  lp_m.set_s(lp.s() - 0.5 * kDelta);
  const GeoPosition gp_s_p = lane->ToGeoPosition(lp_p);
  const GeoPosition gp_s_m = lane->ToGeoPosition(lp_m);
  derivs.col(0) = Vector3<double>{(gp_s_p.x() - gp_s_m.x()) / kDelta,
                                  (gp_s_p.y() - gp_s_m.y()) / kDelta,
                                  (gp_s_p.z() - gp_s_m.z()) / kDelta};

  lp_p = lp;
  lp_m = lp;
  lp_p.set_r(lp.r() + 0.5 * kDelta);
  lp_m.set_r(lp.r() - 0.5 * kDelta);
  const GeoPosition gp_r_p = lane->ToGeoPosition(lp_p);
  const GeoPosition gp_r_m = lane->ToGeoPosition(lp_m);
  derivs.col(1) = Vector3<double>{(gp_r_p.x() - gp_r_m.x()) / kDelta,
                                  (gp_r_p.y() - gp_r_m.y()) / kDelta,
                                  (gp_r_p.z() - gp_r_m.z()) / kDelta};

  // Do only a positive perturbation in the z-axis because the minimum HBound is
  // zero.
  lp_p = lp;
  lp_m = lp;
  lp_p.set_h(lp.h() + 0.5 * kDelta);
  lp_m.set_h(lp.h() - 0.5 * kDelta);
  const GeoPosition gp_h_p = lane->ToGeoPosition(lp_p);
  const GeoPosition gp_h_m = lane->ToGeoPosition(lp_m);
  derivs.col(2) = Vector3<double>{(gp_h_p.x() - gp_h_m.x()) / kDelta,
                                  (gp_h_p.y() - gp_h_m.y()) / kDelta,
                                  (gp_h_p.z() - gp_h_m.z()) / kDelta};

  Eigen::MatrixXd lp_derivs(3, lp_ad.s().derivatives().size());
  lp_derivs.row(0) = lp_ad.s().derivatives();
  lp_derivs.row(1) = lp_ad.r().derivatives();
  lp_derivs.row(2) = lp_ad.h().derivatives();
  GeoPosition gp = lane->ToGeoPosition(lp);
  GeoPositionT<AutoDiffXd> gp_result;
  gp_result.set_x(AutoDiffXd(gp.x(), derivs.row(0) * lp_derivs));
  gp_result.set_y(AutoDiffXd(gp.y(), derivs.row(1) * lp_derivs));
  gp_result.set_z(AutoDiffXd(gp.z(), derivs.row(2) * lp_derivs));
  return gp_result;
}

}  // namespace

LanePositionT<AutoDiffXd> ToLanePositionT(
    const Lane* lane,
    const GeoPositionT<AutoDiffXd>& geo_pos,
    GeoPositionT<AutoDiffXd>* nearest_point,
    AutoDiffXd* distance) {
  DRAKE_DEMAND(lane != nullptr);
  const int deriv_size = geo_pos.x().derivatives().size();
  DRAKE_DEMAND(geo_pos.y().derivatives().size() == deriv_size);
  DRAKE_DEMAND(geo_pos.z().derivatives().size() == deriv_size);

  const GeoPosition gp_double = geo_pos.MakeDouble();

  // For computing nearest position and distance.
  GeoPosition np_double;
  double distance_double{};
  lane->ToLanePosition(gp_double, &np_double, &distance_double);

  const double tol =
      lane->segment()->junction()->road_geometry()->linear_tolerance();
  DRAKE_DEMAND(distance_double < tol);  // geo_pos must be in the lane's
                                        // driveable bounds.
  if (!!nearest_point) {
    nearest_point->set_xyz(geo_pos.xyz());
  }
  if (!!distance) {
    distance->value() = distance_double;
    autodiffxd_make_coherent(geo_pos.x(), distance);
  }
  return ToLanePositionFiniteDifferences(lane, gp_double, geo_pos);
}

LanePositionT<double> ToLanePositionT(const Lane* lane,
                                      const GeoPositionT<double>& geo_pos,
                                      GeoPositionT<double>* nearest_point,
                                      double* distance) {
  return lane->ToLanePosition(geo_pos, nearest_point, distance);
}

GeoPositionT<AutoDiffXd> ToGeoPositionT(
    const Lane* lane,
    const LanePositionT<AutoDiffXd>& lane_pos) {
  const int deriv_size = lane_pos.s().derivatives().size();
  DRAKE_DEMAND(lane_pos.r().derivatives().size() == deriv_size);
  DRAKE_DEMAND(lane_pos.h().derivatives().size() == deriv_size);

  const LanePosition lp_double = lane_pos.MakeDouble();
  return ToGeoPositionFiniteDifferences(lane, lp_double, lane_pos);
}

GeoPositionT<double> ToGeoPositionT(const Lane* lane,
                                    const LanePositionT<double>& lane_pos) {
  return lane->ToGeoPosition(lane_pos);
}

}  // namespace autodiff
}  // namespace automotive
}  // namespace drake
