#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/mobil_planner.h"

namespace drake {

namespace dma = maliput::api;
namespace dmm = maliput::multilane;

namespace automotive {

class Simulator : public AutomotiveSimulator<double> {
 public:
  using AutomotiveSimulator::AutomotiveSimulator;

  const dma::RoadGeometry* make_oval_track(const size_t& num_lanes,
                                           const double& lane_width) {
    using dma::LaneEnd;
    using dmm::ArcOffset;
    using dmm::Builder;
    using dmm::Direction;
    using dmm::Endpoint;
    using dmm::EndpointZ;
    using dmm::EndReference;
    using dmm::LaneLayout;
    using dmm::LineOffset;
    using dmm::StartReference;

    const double rotation = 0.;
    const double x_start = 0.;
    const double y_start = -20.;
    const double straight_length = 50.;  // length of straightaways
    const double arc_radius = 25.;       // radius of arcs
    const double arc_angle = M_PI;

    const double kLeftShoulder = 2.;
    const double kRightShoulder = 2.;
    const int kRefLane = 0;
    const double kRefR0 = 0.;

    const LaneLayout kLaneLayout(kLeftShoulder, kRightShoulder, num_lanes,
                                 kRefLane, kRefR0);

    const dma::HBounds elevation_bounds(0., 5.);
    const ArcOffset kCounterClockwiseArc(arc_radius, arc_angle);
    const LineOffset kLineOffset(straight_length);

    const EndpointZ lay_flat(0., 0., 0., 0.);
    const EndpointZ elevated(10., 0., 0., 0.);
    const Endpoint kStartEndpoint{{x_start, y_start, rotation}, lay_flat};

    Builder builder(lane_width, elevation_bounds, 0.01, 0.01 * M_PI);
    const auto c0 = builder.Connect(
        "0", kLaneLayout,
        StartReference().at(kStartEndpoint, Direction::kForward), kLineOffset,
        EndReference().z_at(lay_flat, Direction::kForward));
    const auto c1 = builder.Connect(
        "1", kLaneLayout,
        StartReference().at(*c0, LaneEnd::Which::kFinish, Direction::kForward),
        kCounterClockwiseArc,
        EndReference().z_at(lay_flat, Direction::kForward));
    const auto c2 = builder.Connect(
        "2", kLaneLayout,
        StartReference().at(*c1, LaneEnd::Which::kFinish, Direction::kForward),
        kLineOffset, EndReference().z_at(lay_flat, Direction::kForward));
    builder.Connect(
        "3", kLaneLayout,
        StartReference().at(*c2, LaneEnd::Which::kFinish, Direction::kForward),
        kCounterClockwiseArc,
        EndReference().z_at(lay_flat, Direction::kForward));

    std::unique_ptr<const dma::RoadGeometry> oval =
        builder.Build(dma::RoadGeometryId{"oval"});

    return SetRoadGeometry(std::move(oval));
  }
};

int main() {
  Simulator simulator;

  const dma::RoadGeometry* rg =
      simulator.make_oval_track(2 /*num_lanes*/, 5 /*lane_width*/);
  SimpleCarState<double> state;
  MaliputRailcarParams<double> params;
  MaliputRailcarState<double> mali_state;

  auto lane = rg->junction(0)->segment(0)->lane(0);
  auto start = lane->ToGeoPosition(dma::LanePosition(0, 0, 0));

  state.set_x(start.x() + 20.);
  state.set_y(start.y() + 1.);
  state.set_velocity(10.);
  state.set_heading(0.);

  mali_state.set_s(40.);
  mali_state.set_speed(40.);

  simulator.AddIdmControlledPriusMaliputRailcar(
      "RailCar", LaneDirection(lane), ScanStrategy::kBranches,
      RoadPositionStrategy::kExhaustiveSearch, 0.2, params, mali_state);

  simulator.AddMobilControlledSimpleCar(
      "MOBILCar", true, ScanStrategy::kBranches,
      RoadPositionStrategy::kExhaustiveSearch, 0.1, state);

  simulator.Start(1);
  while (true) {
    simulator.StepBy(0.1);
  }
  return 0;
}

}  // namespace automotive
}  // namespace drake

int main() { return drake::automotive::main(); }
