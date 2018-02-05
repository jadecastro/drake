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

static constexpr double kEpsilon = 1e-9;

// Compute the inverse rotation matrix:
//    R_inv : (δs, δr, δh) -> (δx, δy, δz)
// with signed scalar multipliers
// introduced for certain elements at each stage in the chain of rotation
// operations:
//    (x, y, z) -> (s', r', z) -> (s, r', h') -> (s, r, h)
// In this code, the following multipliers are introduced:
//    (x, y, z)   -> (w_rs * s', r', z)
//    (s', r', z) -> (w_hs * s, r', h)
//    (s, r', h') -> (s, w_rh * r, h)
// The inverse of the modified rotation matrix is returned.
Eigen::Matrix3d CalcWeightedDerivatives(const Eigen::Vector3d& rpy,
                                        double w_rs,
                                        double w_hs,
                                        double w_rh) {
  Eigen::Matrix3d Rx;
  Rx = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());
  Rx.transposeInPlace();
  Eigen::Matrix3d Ry;
  Ry = Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY());
  Ry.transposeInPlace();
  Eigen::Matrix3d Rz;
  Rz = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
  Rz.transposeInPlace();
  Rx.row(0) << w_rs * Rx.row(0);
  Ry.row(0) << w_hs * Ry.row(0);
  Rz.row(1) << w_rh * Rz.row(1);
  // TODO: decide if it's more sensible to multiply by diagonal matrices.
  return Rx * Ry * Rz;
}

// Given a lane position and lane, estimate the signed curvature projections to
// the x-y, y-z, x-z axes via the three-point method.
Eigen::Vector3d EstimateCurvature(const Lane* lane,
                                  const LanePosition& lane_pos) {
  const LanePosition lane_pos_plus(
      lane_pos.s() + kEpsilon, lane_pos.r(), lane_pos.h());
  const LanePosition lane_pos_minus(
      lane_pos.s() - kEpsilon, lane_pos.r(), lane_pos.h());

  const GeoPosition geo_pos = lane->ToGeoPosition(lane_pos);
  const GeoPosition geo_pos_plus = lane->ToGeoPosition(lane_pos_plus);
  const GeoPosition geo_pos_minus = lane->ToGeoPosition(lane_pos_minus);

  const Eigen::Vector3d xyz = geo_pos.xyz();
  const Eigen::Vector3d xyz_plus = geo_pos_plus.xyz();
  const Eigen::Vector3d xyz_minus = geo_pos_minus.xyz();

  const Eigen::Vector3d d1 = (xyz_plus - xyz_minus) / 2. / kEpsilon;
  const Eigen::Vector3d d2 = (xyz_plus + xyz_minus) / 2. / kEpsilon - xyz;
  const double kx =
      (d1[0] * d2[1] - d2[0] * d1[1]) / pow(d1[0] * d1[0] + d1[1] * d1[1], 3/2);
  const double ky =
      (d1[1] * d2[2] - d2[1] * d1[2]) / pow(d1[1] * d1[1] + d1[2] * d1[2], 3/2);
  const double kz =
      (d1[0] * d2[2] - d2[0] * d1[2]) / pow(d1[0] * d1[0] + d1[2] * d1[2], 3/2);
  return {kx, ky, kz};
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
  GeoPosition np_double;
  double distance_double{};
  const LanePosition lp_double =
      lane->ToLanePosition(gp_double, &np_double, &distance_double);

  const double tol =
      lane->segment()->junction()->road_geometry()->linear_tolerance();
  DRAKE_DEMAND(distance_double < tol);  // geo_pos must be in the lane's
                                        // driveable bounds.
  // Assuming ∂(rot)/∂v = 0:
  const Rotation rot = lane->GetOrientation(lp_double);
  // N.B. Pitch is still wrong here. Need to negate.

  // ∂S/∂X = [∂S/∂x, ∂S/∂y, ∂S/∂z].
  // const Eigen::Matrix3d rotmat = rot.matrix().transpose();
  const double k_yaw = 25. / (25. - lp_double.r());
  std::cout << " k_yaw " << k_yaw << std::endl;
  Eigen::Matrix3d dSdX;
  if (std::abs(lp_double.r()) < tol || std::abs(lp_double.h()) < tol) {
    dSdX = math::rpy2rotmat(rot.rpy()).transpose();
  } else {
    const Eigen::Vector3d k = EstimateCurvature(lane, lp_double);
    std::cout << " k estimated " << k << std::endl;

    // TODO: below is incorrect for k[1], k[2]. Let's hold off till the
    // debugging stage to fix.
    dSdX = CalcWeightedDerivatives(rot.rpy(), k[0], k[1], k[2]);
  }
  // dX/dV = [∂X/∂v₀, …, ∂X/∂v_n] (a row vector of 3-vectors).
  Eigen::MatrixXd dXdV(3, deriv_size);
  dXdV.row(0) = geo_pos.x().derivatives();
  dXdV.row(1) = geo_pos.y().derivatives();
  dXdV.row(2) = geo_pos.z().derivatives();
  // Note that, in general dS/dV = ∂S/∂X * dX/dV + ∂S/∂Θ * dΘ/dV
  //                             = R(Θ) * dX/∂V  + 0
  const Eigen::MatrixXd dSdV = dSdX * dXdV;

  // TODO(jadecastro) Unit test should include derivs wrt to non-spatial
  // quantities (e.g. time) and take those at the lane's s-axis as well as away
  // from it.

  if (!!nearest_point) {
    nearest_point->set_xyz(geo_pos.xyz());
  }
  if (!!distance) {
    distance->value() = distance_double;
    autodiffxd_make_coherent(geo_pos.x(), distance);
  }
  const AutoDiffXd s(lp_double.s(), dSdV.row(0));
  const AutoDiffXd r(lp_double.r(), dSdV.row(1));
  const AutoDiffXd h(lp_double.h(), dSdV.row(2));
  return {s, r, h};
}

LanePositionT<double> ToLanePositionT(const Lane* lane,
                                      const GeoPositionT<double>& geo_pos,
                                      GeoPositionT<double>* nearest_point,
                                      double* distance) {
  return lane->ToLanePosition(geo_pos, nearest_point, distance);
}

/*
GeoPositionT<AutoDiffXd> ToGeoPositionT(
    const Lane* lane,
    const LanePositionT<AutoDiffXd>& lane_pos) {
  const int deriv_size = lane_pos.s().derivatives().size();
  DRAKE_DEMAND(lane_pos.r().derivatives().size() == deriv_size);
  DRAKE_DEMAND(lane_pos.h().derivatives().size() == deriv_size);

  const LanePosition lp_double = lane_pos.MakeDouble();
  const GeoPosition gp_double = lane->ToGeoPosition(lp_double);
  // Assuming ∂(rot)/∂v = 0:
  const Rotation rot = lane->GetOrientation(lp_double);

  // ∂X/∂S = [∂X/∂s, ∂X/∂r, ∂X/∂h].
  Eigen::Matrix3d dXdS;
  if (std::abs(lane_pos.r()) < tol || std::abs(lane_pos.h()) < tol) {
    dXdS = math::rpy2rotmat(rot.rpy())();
  } else {
    dXdS = ...
  }
  // dS/dV = [∂S/∂v₀, …, ∂S/∂v_n] (a row vector of 3-vectors).
  Eigen::MatrixXd dSdV(3, deriv_size);
  dSdV.row(0) = lane_pos.s().derivatives();
  dSdV.row(1) = lane_pos.r().derivatives();
  dSdV.row(2) = lane_pos.h().derivatives();
  // Note that, in general dX/dV = ∂X/∂S * dS/dV  + ∂X/∂Θ * dΘ/dV
  //                             = R(Θ)⁻¹ * dS/∂V + 0
  const Eigen::MatrixXd dXdV = dXdS * dSdV;

  // TODO(jadecastro) Unit test should include derivs wrt to non-spatial
  // quantities (e.g. time) and take those at the lane's s-axis as well as away
  // from it.

  const AutoDiffXd x(gp_double.x(), dXdV.row(0));
  const AutoDiffXd y(gp_double.y(), dXdV.row(1));
  const AutoDiffXd z(gp_double.z(), dXdV.row(2));
  return {x, y, z};
}

GeoPositionT<double> ToGeoPositionT(Lane* lane,
                                    LanePositionT<double>& lane_pos) {
  return lane->ToGeoPosition(lane_pos);
}
*/

}  // namespace autodiff
}  // namespace automotive
}  // namespace drake
