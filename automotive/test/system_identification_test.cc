// TODO(jon.decastro) Make this a binary file, rather than a unit test.

#include <random>  // Used only with deterministic seeds!

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/automotive/gen/pure_pursuit_params.h"
#include "drake/automotive/gen/simple_car_params.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/scenario.h"
#include "drake/common/polynomial.h"
#include "drake/common/trig_poly.h"
#include "drake/solvers/system_identification.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace automotive {
namespace {

static constexpr double kRoadLength = 100.;
static constexpr double kLaneWidth = 3.;
static constexpr int kNumLanes = 3;
static constexpr double kCarWidth = 2.;
static constexpr double kCarLength = 4.;

typedef solvers::SystemIdentification<double> SID;

using maliput::api::Lane;
using systems::BasicVector;
using systems::Diagram;

const Diagram<double>& MakeScenario() {
  // For building Dragway roads.
  auto road = std::make_unique<maliput::dragway::RoadGeometry>(
      maliput::api::RoadGeometryId("three_lane_road"), kNumLanes, kRoadLength,
      kLaneWidth, 0., 5., 1e-6, 1e-6);
  auto scenario =
      std::make_unique<Scenario>(std::move(road), kCarWidth, kCarLength);
  const maliput::api::Segment* segment =
      scenario->road().junction(0)->segment(0);
  // Make an ego car.
  scenario->AddSimpleCar("ego_car", SimpleCarParams<double>());
  // Make three ado cars.
  scenario->AddIdmSimpleCar(
      "ado_car_0", LaneDirection(segment->lane(0), true),
      SimpleCarParams<double>(), IdmPlannerParameters<double>(),
      PurePursuitParams<double>());
  scenario->Build();
  return scenario->diagram();
}

/*
std::vector<BasicVector<double>> MakeSimulationData(
    const Eigen::VectorXd& t, const Eigen::MatrixXd& states,
    double final_time) {
  using Type = Trajectory::InterpolationType;

  const double kRealTimeRate = 1.;

  // Build another simulator with LCM capability and run in play-back mode.
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  simulator->SetRoadGeometry(scenario_->get_road());
  auto* lcm = dynamic_cast<drake::lcm::DrakeLcm*>(simulator->get_lcm());
  DRAKE_DEMAND(lcm != nullptr);

  std::vector<double> times(t.rows());
  for (size_t i{0}; i < times.size(); i++) {
    times[i] = t(i);
  }
  int index{0};
  for (const auto& subsystem : scenario_->aliases()) {
    std::vector<Eigen::Vector3d> translations{};
    std::vector<Quaternion<double>> rotations{};
    for (size_t i{0}; i < times.size(); i++) {
      translations.push_back(
          {states(0 + index * SimpleCarStateIndices::kNumCoordinates, i),
           states(1 + index * SimpleCarStateIndices::kNumCoordinates, i), 0.});
      const math::RollPitchYaw<double> rpy(
          Eigen::Vector3d(0., 0., states(2 + index * SimpleCarStateIndices::kNumCoordinates, i)));
      rotations.push_back(rpy.ToQuaternion());
      rotations.back().normalize();  // TODO: Need?
    }
    const Trajectory trajectory =
        Trajectory::Make(times, rotations, translations, Type::kPchip);
    simulator->AddPriusTrajectoryFollower(subsystem->get_name(), trajectory);
    index++;
  }
  simulator->Start(kRealTimeRate);
  simulator->StepBy(final_time);

  return;
}

std::vector<BasicVector<double>> MakeMeasurements(
    const std::BasicVector<double>& trajectory) {
  std::default_random_engine noise_generator;
  noise_generator.seed(kNoiseSeed);
  std::uniform_real_distribution<double> noise_distribution(-kNoise, kNoise);
  auto noise = std::bind(noise_distribution, noise_generator);

  std::vector<SID::PartialEvalType> measurements;
  for (const BasicVector<double>& state : trajectory) {
    SID::PartialEvalType measurement;
    measurement[pos_var] = state.position + noise();
    measurement[velocity_var] = state.velocity + noise();
    measurement[acceleration_var] = state.acceleration + noise();
    measurement[input_force_var] = state.force + noise();
    measurements.push_back(measurement);
  }
  return measurements;
}
*/

GTEST_TEST(SystemIdentificationTest, IdmSystemIdentification) {
  /*
  Polynomiald pos = Polynomiald("pos");
  auto pos_var = pos.GetSimpleVariable();
  Polynomiald velocity = Polynomiald("vel");
  auto velocity_var = velocity.GetSimpleVariable();
  Polynomiald acceleration = Polynomiald("acc");
  auto acceleration_var = acceleration.GetSimpleVariable();
  Polynomiald input_force = Polynomiald("f_in");
  auto input_force_var = input_force.GetSimpleVariable();
  Polynomiald mass = Polynomiald("m");
  auto mass_var = mass.GetSimpleVariable();
  Polynomiald damping = Polynomiald("b");
  auto damping_var = damping.GetSimpleVariable();
  Polynomiald spring = Polynomiald("k");
  auto spring_var = spring.GetSimpleVariable();

  VectorXPoly v(1); v << velocity;
  VectorXPoly vdot(1); vdot << acceleration;
  VectorXPoly H(1); H << mass;
  VectorXPoly C(1); C << 0;
  VectorXPoly g(1); g << (spring * pos);
  VectorXPoly f(1); f << (velocity * damping);
  VectorXPoly B(1); B << 1;
  VectorXPoly u(1); u << input_force;

  const VectorXPoly manipulator_left = (H * vdot) + (C * v) + g;
  const VectorXPoly manipulator_right = (B * u) + f;
  */

  const Diagram<double>& diagram = MakeScenario();

  // const std::vector<BasicVector<double>> trajectory = MakeSimulationData();
  // const std::vector<BasicVector<double>> data = MakeMeasurements(trajectory);

  /*
  SID::PartialEvalType estimated_params;
  double error;
  std::tie(estimated_params, error) =
      SID::EstimateParameters(, measurements);
  EXPECT_LT(error, 2e-2);
  */
}

}  // anonymous namespace
}  // namespace automotive
}  // namespace drake
