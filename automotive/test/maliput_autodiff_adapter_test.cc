#include "drake/automotive/maliput_autodiff_adapter.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/multilane_onramp_merge.h"
#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace automotive {
namespace autodiff {
namespace {

using maliput::api::GeoPosition;
using maliput::api::GeoPositionT;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::LanePositionT;
using maliput::api::Rotation;

constexpr double kDelta = 1e-9;
constexpr double kBuffer = 1e-3;
const LanePosition kSomeLanePosition{20., 0., 0.};

enum class LanePolarity { kWithS, kAgainstS };

const Lane* GetLaneFromId(const maliput::api::RoadGeometry& road,
                          const std::string& lane_id) {
  for (int i = 0; i < road.num_junctions(); ++i) {
    if (road.junction(i)->segment(0)->lane(0)->id().string() == lane_id) {
      return road.junction(i)->segment(0)->lane(0);
    }
  }
  throw std::runtime_error("No matching lane name in the road network");
}

// Populate the partial derivatives of LanePosition with respect to x, y, z from
// a GeoPosition using the method of finite differences.
LanePositionT<AutoDiffXd> ToLanePositionFiniteDifferences(
    const Lane* lane, const GeoPosition& gp) {
  LanePosition lp = lane->ToLanePosition(gp, nullptr, nullptr);
  LanePositionT<AutoDiffXd> lp_result;
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

  lp_result.set_s(AutoDiffXd(lp.s(), derivs.row(0)));
  lp_result.set_r(AutoDiffXd(lp.r(), derivs.row(1)));
  lp_result.set_h(AutoDiffXd(lp.h(), derivs.row(2)));
  return lp_result;
}

// Check idempotency wrt. Lane::ToLanePosition for double types.
GTEST_TEST(MaliputAutodiffUtils, TestDoubleMultilane) {
  // N.B. In this road, `post0` branches into `pre0` and `onramp1`.
  std::unique_ptr<MultilaneOnrampMerge> merge_road(new MultilaneOnrampMerge);
  std::unique_ptr<const maliput::api::RoadGeometry> rg =
      merge_road->BuildOnramp();

  const Lane* lane = GetLaneFromId(*rg, "l:pre0_0");
  const GeoPosition gp = lane->ToGeoPosition(kSomeLanePosition);

  const LanePosition lp_dut = ToLanePositionT(lane, gp, nullptr, nullptr);

  EXPECT_TRUE(CompareMatrices(kSomeLanePosition.srh(), lp_dut.srh(), 1e-10));
}

// Test derivatives at various locations in a monolane road network having zero
// elevation/superelevation.
GTEST_TEST(MaliputAutodiffUtils, TestAutoDiffMultilane) {
  // N.B. In this road, `post0` branches into `pre0` and `onramp1`.
  std::unique_ptr<MultilaneOnrampMerge> merge_road(new MultilaneOnrampMerge);
  std::unique_ptr<const maliput::api::RoadGeometry> rg =
      merge_road->BuildOnramp();

  for (int i{0}; i < 1; ++i) {
    const Lane* lane = GetLaneFromId(*rg, "l:post0_0"); // rg->junction(i)->segment(0)->lane(0);
    std::cout << " Testing @ lane " << lane->id().string() << std::endl;

    const double s_min = 0.;
    const double s_max = lane->length();
    for (double s{s_min + kBuffer}; s < s_max - kBuffer;
         s += 0.5 * (s_max - s_min - 4 * kBuffer)) {
      const double r_min = lane->driveable_bounds(s).min();
      const double r_max = lane->driveable_bounds(s).max();
      for (double r{r_min + kBuffer}; r < r_max - kBuffer;
           r += 0.5 * (r_max - r_min - 4 * kBuffer)) {
        const double h_min = lane->elevation_bounds(s, r).min();
        const double h_max = lane->elevation_bounds(s, r).max();
        for (double h{h_min + kBuffer}; h < h_max - kBuffer;
             h += 0.5 * (h_max - h_min - 4 * kBuffer)) {
          const LanePosition lp{s, r, h};
          std::cout << "    Testing @ lp " << lp << std::endl;
          const GeoPosition gp = lane->ToGeoPosition(lp);
          std::cout << "    Testing @ gp " << gp << std::endl;

          // Check also lp_ad and gp_dut.
          GeoPositionT<AutoDiffXd> gp_ad;
          gp_ad.set_x(AutoDiffXd(gp.x(), Vector3<double>{1., 0., 0.}));
          gp_ad.set_y(AutoDiffXd(gp.y(), Vector3<double>{0., 1., 0.}));
          gp_ad.set_z(AutoDiffXd(gp.z(), Vector3<double>{0., 0., 1.}));

          const LanePositionT<AutoDiffXd> lp_dut =
              ToLanePositionT(lane, gp_ad, nullptr, nullptr);

          EXPECT_TRUE(CompareMatrices(
              lp.srh(), lp_dut.MakeDouble().srh(), 1e-10));

          const LanePositionT<AutoDiffXd> lp_finite_diff =
              ToLanePositionFiniteDifferences(lane, gp);
          EXPECT_TRUE(CompareMatrices(
              lp.srh(), lp_finite_diff.MakeDouble().srh(), 1e-10));

          EXPECT_TRUE(CompareMatrices(lp_finite_diff.s().derivatives(),
                                      lp_dut.s().derivatives(), 1e-4));
          EXPECT_TRUE(CompareMatrices(lp_finite_diff.r().derivatives(),
                                      lp_dut.r().derivatives(), 1e-4));
          EXPECT_TRUE(CompareMatrices(lp_finite_diff.h().derivatives(),
                                      lp_dut.h().derivatives(), 1e-4));
        }
      }
    }
  }
}

GTEST_TEST(MaliputAutodiffUtils, TestAutoDiffDragway) {
  maliput::dragway::RoadGeometry rg(
      maliput::api::RoadGeometryId("1-lane dragway"), 1 /* num_lanes */,
      100. /* length */, 2. /* lane_width */, 0. /* shoulder_width */,
      5. /* maximum_height */,
      std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance */);

  // AutoDiffXd only appear at the inputs; only check that computation succeeds.
  const AutoDiffXd speed = 10.;

  const Lane* lane = rg.junction(0)->segment(0)->lane(0);
  unused(lane);
}

// TODO:
//
// - At lane-lane interfaces, multiple solutions are possible, due to changes in
//   curvature, thus we get subderivatives.  Check that our code picks the right
//   one.
// - Run tests that ensure that the assumptions hold.
// - Test for curvatures in positive-r and negative-r directions.

}  // namespace
}  // namespace autodiff
}  // namespace automotive
}  // namespace drake
