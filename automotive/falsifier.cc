#include "drake/automotive/falsifier.h"

#include <memory>

#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/pure_pursuit.h"
#include "drake/automotive/simple_car.h"
#include "drake/common/proto/call_python.h"
#include "drake/common/symbolic.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace automotive {

using common::CallPython;
using systems::System;
using systems::trajectory_optimization::DirectCollocation;
using trajectories::PiecewisePolynomial;

static constexpr int kX = SimpleCarStateIndices::kX;
static constexpr int kY = SimpleCarStateIndices::kY;
// static constexpr int kHeading = SimpleCarStateIndices::kHeading;
// static constexpr int kVelocity = SimpleCarStateIndices::kVelocity;

static constexpr int kNumAdoCars = 3;
static constexpr double kLaneWidth = 3.;
static constexpr double kDragwayLength = 150.;
static const std::string kEgoCarName = "ego_car";
static const std::string kLaneChangerName = "lane_changer";

static constexpr int kNumTimeSamples = 21;

namespace {

std::unique_ptr<maliput::api::RoadGeometry> MakeRoad(int num_dragway_lanes) {
  return std::make_unique<maliput::dragway::RoadGeometry>(
      maliput::api::RoadGeometryId(
          maliput::api::TypeSpecificIdentifier<
          class maliput::api::RoadGeometry>("Dragway")),
      num_dragway_lanes, kDragwayLength, kLaneWidth,
      0. /* shoulder width */, 5. /* maximum_height */,
      std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance */);
}

std::pair<std::unique_ptr<AutomotiveSimulator<double>>, std::vector<int>>
SetupSimulator(bool is_playback_mode, int num_dragway_lanes,
               std::unique_ptr<maliput::api::RoadGeometry> road,
               const std::vector<const maliput::api::Lane*>& goal_lanes) {
  DRAKE_DEMAND(num_dragway_lanes > 0);
  std::vector<int> ids{};

  const int num_cars = goal_lanes.size() + 1 /* ego + ado cars */;
  auto simulator = (is_playback_mode)
      ? std::make_unique<AutomotiveSimulator<double>>(
          std::make_unique<lcm::DrakeLcm>(), num_cars)
      : std::make_unique<AutomotiveSimulator<double>>(nullptr, num_cars);

  auto simulator_road = simulator->SetRoadGeometry(std::move(road));
  auto dragway_road_geometry =
      dynamic_cast<const maliput::dragway::RoadGeometry*>(simulator_road);

  // TODO: We can just delete these altogether.
  // Set the initial states.
  const int kEgoStartLaneIndex = 0;
  const int kEgoGoalLaneIndex = 0;

  const maliput::api::Lane* ego_start_lane =
      dragway_road_geometry->junction(0)->segment(0)->lane(kEgoStartLaneIndex);
  const maliput::api::Lane* ego_goal_lane =
      dragway_road_geometry->junction(0)->segment(0)->lane(kEgoGoalLaneIndex);

  // Provide a valid instantiation to be fixed in the optimization.
  const double start_s_ego = 5.;
  const double start_speed_ego = 20.;
  const maliput::api::GeoPosition start_position_ego =
      ego_start_lane->ToGeoPosition({start_s_ego, 0., 0.});

  SimpleCarState<double> ego_initial_state;
  ego_initial_state.set_x(start_position_ego.x());
  ego_initial_state.set_y(start_position_ego.y());
  ego_initial_state.set_heading(0.);
  ego_initial_state.set_velocity(start_speed_ego);

  const int ego_id = simulator->AddIdmControlledCar(
      kEgoCarName, true /* move along the "s"-direction */,
      ego_initial_state, ego_goal_lane, ScanStrategy::kPath,
      RoadPositionStrategy::kExhaustiveSearch, 0. /* (unused) */);
  ids.emplace_back(ego_id);

  int i{0};
  for (const auto goal_lane : goal_lanes) {
    // Provide a valid instantiation for the lane changer (note these will be
    // overwritten).
    const double start_s_lc = 5.;
    const double start_speed_lc = 10.;
    const maliput::api::GeoPosition start_position_lc =
        ego_start_lane->ToGeoPosition({start_s_lc, 0., 0.});

    SimpleCarState<double> lc_initial_state;
    lc_initial_state.set_x(start_position_lc.x());
    lc_initial_state.set_y(start_position_lc.y());
    lc_initial_state.set_heading(0.);
    lc_initial_state.set_velocity(start_speed_lc);

    // All lane changers move into the ego car's lane.
    const int ado_id = simulator->AddIdmControlledCar(
        kLaneChangerName + "_" + std::to_string(i++),
        true /* move along the "s"-direction */, lc_initial_state, goal_lane,
        ScanStrategy::kPath, RoadPositionStrategy::kExhaustiveSearch,
        0. /* (unused) */);
    ids.emplace_back(ado_id);
  }
  return std::make_pair(
      std::unique_ptr<AutomotiveSimulator<double>>(simulator.release()), ids);
}

template <typename T>
void SetSimpleCarState(const systems::Diagram<T>& diagram,
                       const systems::System<T>& car,
                       const SimpleCarState<T>& value,
                       systems::Context<T>* context) {
  systems::VectorBase<T>& context_state =
      diagram.GetMutableSubsystemContext(car, context)
      .get_mutable_continuous_state_vector();
  SimpleCarState<T>* const state =
      dynamic_cast<SimpleCarState<T>*>(&context_state);
  DRAKE_ASSERT(state);
  state->set_value(value.get_value());
}

// TODO(jadecastro) Diagram context yearns for a better way at mapping
// subcontext to subsystem.
void PopulateContext(
    const systems::Diagram<double>& diagram,
    const SimpleCarState<double>& ego_initial_conditions,
    const std::vector<SimpleCarState<double>*>& lc_initial_conditions,
    std::vector<const systems::System<double>*>& systems,
    systems::Context<double>* context) {
  int i{0};
  for (const auto& system : systems) {
    std::string name = system->get_name();
    //    std::cout << " NAME " << name << std::endl;
    if (name.find(kEgoCarName + "_simple_car") != std::string::npos) {
      SetSimpleCarState(diagram, *system, ego_initial_conditions, context);
    } else if ((name.find(kLaneChangerName) != std::string::npos) &&
               (name.find("simple_car") != std::string::npos)) {
      SetSimpleCarState(diagram, *system, *lc_initial_conditions[i++], context);
      // std::cout << "  Ado ic "
      //          << context->get_continuous_state_vector().CopyToVector()
      //          << std::endl;
    }
  }
}


std::vector<int> GetEgoIndices(std::vector<const System<double>*> systems) {
  int index{0};
  auto result = std::vector<int>(SimpleCarStateIndices::kNumCoordinates);
  for (const auto& system : systems) {
    std::string name = system->get_name();
    if (name.find(kEgoCarName + "_simple_car") != std::string::npos) {
      std::iota(std::begin(result), std::end(result), index);
      break;
    }
    if (name.find("simple_car") != std::string::npos) {
      index += SimpleCarStateIndices::kNumCoordinates;
    }
  }
  return result;
}

std::vector<std::vector<int>> GetAdoIndices(
    std::vector<const System<double>*> systems) {
  std::vector<std::vector<int>> result{};
  for (int i{0}; i < kNumAdoCars; i++) {
    int index{0};
    auto car_result = std::vector<int>(SimpleCarStateIndices::kNumCoordinates);
    for (const auto& system : systems) {
      std::string name = system->get_name();
      if ((name.find(kLaneChangerName) != std::string::npos) &&
          (name.find("_" + std::to_string(i) + "_") !=
           std::string::npos) &&
          (name.find("simple_car") != std::string::npos)) {
        std::iota(std::begin(car_result), std::end(car_result), index);
        break;
      }
      if (name.find("simple_car") != std::string::npos) {
        index += SimpleCarStateIndices::kNumCoordinates;
      }
    }
    result.push_back(car_result);
  }
  return result;
}

}  // namespace

Falsifier::Falsifier() {
  using std::log;
  using std::sqrt;

  const int kNumLanes = 3;  // Number of lanes in the scenario.

  std::unique_ptr<maliput::api::RoadGeometry> road = MakeRoad(kNumLanes);
  const maliput::api::Segment* segment = road->junction(0)->segment(0);

  const maliput::api::Lane* goal_lane_car0 = segment->lane(0);
  // const maliput::api::Lane* goal_lane_car1 = segment->lane(1);
  const maliput::api::Lane* goal_lane_car2 = segment->lane(2);
  //  const maliput::api::Lane* goal_lane_car3 = segment->lane(0);

  std::vector<const maliput::api::Lane*> goal_lanes;
  goal_lanes.push_back(goal_lane_car2);
  // goal_lanes.push_back(goal_lane_car1);
  // goal_lanes.push_back(goal_lane_car2);
  //  goal_lanes.push_back(goal_lane_car3);

  std::vector<int> ids;
  std::tie(simulator_, ids) = SetupSimulator(
      false /* is_playback_mode */, kNumLanes, std::move(road), goal_lanes);

  simulator_->BuildAndInitialize();

  const systems::System<double>& plant = simulator_->GetDiagram();
  auto context = plant.CreateDefaultContext();

  // Set up a direct-collocation feasibility problem.
  const double guess_duration = 10.;  // seconds
  const double kMinTimeStep = 1. * guess_duration / (kNumTimeSamples - 1);
  const double kMaxTimeStep = 1. * guess_duration / (kNumTimeSamples - 1);
  min_time_step_ = kMinTimeStep;
  max_time_step_ = kMaxTimeStep;

  prog_.reset(new DirectCollocation(
      &plant, *context, kNumTimeSamples, kMinTimeStep, kMaxTimeStep));

  // Ensure that time intervals are evenly spaced.
  prog_->AddEqualTimeIntervalsConstraints();

  // VectorX<double> vect0(14);
  // VectorX<double> vectf(14);

  const auto& diagram = dynamic_cast<const systems::Diagram<double>&>(plant);
  std::vector<const systems::System<double>*> systems = diagram.GetSystems();

  // Getters for the subsystem (SimpleCar) states.
  ego_indices_ = GetEgoIndices(systems);
  ado_indices_ = GetAdoIndices(systems);

  // Supply ICs externally to main().
  SimpleCarState<double> ego_initial_conditions;
  ego_initial_conditions.set_x(30.);
  ego_initial_conditions.set_y(goal_lane_car0->ToGeoPosition({0., 0., 0.}).y());
  ego_initial_conditions.set_heading(0.);
  ego_initial_conditions.set_velocity(7.);

  std::vector<SimpleCarState<double>*> lc_initial_conditions;
  SimpleCarState<double> car0;
  car0.set_x(10.);
  car0.set_y(goal_lane_car2->ToGeoPosition({0., 0., 0.}).y());
  car0.set_heading(0.);
  car0.set_velocity(5.);
  lc_initial_conditions.push_back(&car0);

  /*
  SimpleCarState<double> car1;
  car1.set_x(60.);
  car1.set_y(goal_lane_car1->ToGeoPosition({0., 0., 0.}).y());
  car1.set_heading(0.);
  car1.set_velocity(8.);
  lc_initial_conditions.push_back(&car1);

  SimpleCarState<double> car2;
  car2.set_x(10.);
  car2.set_y(goal_lane_car2->ToGeoPosition({0., 0., 0.}).y());
  car2.set_heading(0.);
  car2.set_velocity(5.);
  lc_initial_conditions.push_back(&car2);

  SimpleCarState<double> car3;
  car3.set_x(20.);
  car3.set_y(1. * kLaneWidth + delta_y);
  car3.set_heading(0.);
  car3.set_velocity(10.);
  lc_initial_conditions.push_back(&car3);
  */

  auto initial_context = diagram.CreateDefaultContext();
  PopulateContext(diagram, ego_initial_conditions, lc_initial_conditions,
                  systems, initial_context.get());

  // Supply final conditions for the guessed trajectory.
  SimpleCarState<double> ego_final_conditions;
  ego_final_conditions.set_x(40.);
  ego_final_conditions.set_y(goal_lane_car0->ToGeoPosition({0., 0., 0.}).y());
  ego_final_conditions.set_heading(0.);
  ego_final_conditions.set_velocity(5.0);

  std::vector<SimpleCarState<double>*> lc_final_conditions;
  car0.set_x(40.);
  car0.set_y(goal_lane_car2->ToGeoPosition({0., 0., 0.}).y());
  car0.set_heading(0.);
  car0.set_velocity(6.);
  lc_final_conditions.push_back(&car0);

  /*
  car1.set_x(80.);
  car1.set_y(goal_lane_car1->ToGeoPosition({0., 0., 0.}).y());
  car1.set_heading(0.);
  car1.set_velocity(8.);
  lc_final_conditions.push_back(&car1);

  car2.set_x(40.);
  car2.set_y(goal_lane_car2->ToGeoPosition({0., 0., 0.}).y());
  car2.set_heading(0.);
  car2.set_velocity(6.);
  lc_final_conditions.push_back(&car2);

  car3.set_x(30.);
  car3.set_y(0.4 * kLaneWidth + delta_y);
  car3.set_heading(0.);
  car3.set_velocity(10.);
  lc_final_conditions.push_back(&car3);
  */

  auto final_context = diagram.CreateDefaultContext();
  PopulateContext(diagram, ego_final_conditions, lc_final_conditions,
                  systems, final_context.get());

  std::cout << " initial states \n "
            << initial_context->get_continuous_state_vector().CopyToVector()
            << std::endl;
  std::cout << " final states \n"
            << final_context->get_continuous_state_vector().CopyToVector()
            << std::endl;

  // Generous bounding box on all decision variables.
  prog_->AddBoundingBoxConstraint(-100, 100, prog_->decision_variables());

  // Parse the initial conditions.
  const systems::VectorBase<double>& initial_states =
      initial_context->get_continuous_state_vector();
  prog_->AddLinearConstraint(prog_->initial_state() ==
                             initial_states.CopyToVector());

  // Constraints keeping the cars on the road.
  prog_->AddConstraintToAllKnotPoints(
      prog_->state()(ego_indices_.at(kY)) >= -1.5 * kLaneWidth);
  prog_->AddConstraintToAllKnotPoints(
      prog_->state()(ego_indices_.at(kY)) <= 0.5 * kLaneWidth);

  /*
  prog_->AddConstraintToAllKnotPoints(
      prog_->state()(ado_indices_[0].at(kY)) >= -1.5 * kLaneWidth);
  prog_->AddConstraintToAllKnotPoints(
      prog_->state()(ado_indices_[0].at(kY)) <= 0.5 * kLaneWidth);

  prog_->AddConstraintToAllKnotPoints(
      prog_->state()(ado_indices_[1].at(kY)) >= -1.5 * kLaneWidth);
  prog_->AddConstraintToAllKnotPoints(
      prog_->state()(ado_indices_[1].at(kY)) <= 0.5 * kLaneWidth);
  */

  // TODO(jadecastro) Infeasible with these active?
  // prog_->AddConstraintToAllKnotPoints(
  //    prog_->prog.state()(ado_indices_[2].at(kY)) >= -1.5 * kLaneWidth);
  // prog_->AddConstraintToAllKnotPoints(
  //    prog_->state()(ado_indices_[2].at(kY)) <= 0.5 * kLaneWidth);

  // Solve a collision with one of the cars, in this case, Traffic Car 2 merging
  // into the middle lane.  Assume for now that, irrespective of its
  // orientation, the traffic car occupies a lane-aligned bounding box.

  // TODO(jadecastro) As above, this assumes the ordering of the context. Update
  // Dircol to make this less fragile.
  prog_->AddLinearConstraint(prog_->final_state()(ado_indices_[0].at(kY)) <=
                             prog_->final_state()(ego_indices_.at(kY))
                             + 0.5 * kLaneWidth);
  prog_->AddLinearConstraint(prog_->final_state()(ado_indices_[0].at(kX)) >=
                             prog_->final_state()(ego_indices_.at(kX)) - 2.5);
  prog_->AddLinearConstraint(prog_->final_state()(ado_indices_[0].at(kX)) <=
                             prog_->final_state()(ego_indices_.at(kX)) + 2.5);

  const Eigen::VectorXd x0 =
      initial_context->get_continuous_state_vector().CopyToVector();
  const Eigen::VectorXd xf =
      final_context->get_continuous_state_vector().CopyToVector();
  auto guess_state_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      {0, guess_duration}, {x0, xf});
  prog_->SetInitialTrajectory(PiecewisePolynomial<double>(),
                              guess_state_trajectory);

  // Add a log-probability cost representing the deviation of the commands from
  // a deterministic evolution of IDM/PurePursuit (nominal policy).  We assume
  // that the policy has zero-mean Gaussian distribution.
  // TODO: Separate out Sigmas for steering and acceleration.
  const double kSigma = 50.;
  const double kCoeff = 1. / (2. * kSigma * kSigma);
  symbolic::Expression running_cost;
  for (int i{0}; i < prog_->input().size(); ++i) {
    std::cout << " prog_->input()(i) " << prog_->input()(i) << std::endl;
    running_cost += kCoeff * prog_->input()(i) * prog_->input()(i);
  }
  prog_->AddRunningCost(running_cost);

  // Additionally, provide a chance constraint on drawing the sample path of the
  // disturbance.
  const double kSamplePathProb = 0.5;
  const double kP = 2. * kNumTimeSamples * kSigma * kSigma *
      log(1. / (sqrt(2. * M_PI) * kSigma)) -
      2. * kSigma * kSigma * log(kSamplePathProb);
  drake::unused(kP);
  for (int i{0}; i < prog_->input().size(); ++i) {
    std::cout << " prog_->input()(i) " << prog_->input()(i) << std::endl;
    // prog_->AddConstraint(prog_->input()(i) * prog_->input()(i) >= kP);
  }
}

void Falsifier::SetEgoLinearConstraint(
    const Eigen::Ref<const Eigen::MatrixXd> A,
    const Eigen::Ref<const Eigen::VectorXd> b, double t) {
  DRAKE_DEMAND(min_time_step_ == max_time_step_);
  // ** TODO ** Relax this constraint?

  // Assume a zero-order hold on the constraints application.
  // ** TODO ** Replace with std::find().
  int index{0};
  while (t > index++ * min_time_step_) {}
  auto x_ego = prog_->state(index).segment(
      ego_indices_[0], SimpleCarStateIndices::kNumCoordinates);
  prog_->AddLinearConstraint(
      A, -b * std::numeric_limits<double>::infinity(), b, x_ego);
  // prog_->AddLinearConstraint(A * x_ego <= b);
}

void Falsifier::Run() {
  const auto result = prog_->Solve();

  // Extract the initial context from prog and plot the solution using MATLAB.
  // To view, type `bazel run //common/proto:call_python_client_cli`.

  trajectory_.inputs = prog_->GetInputSamples();
  trajectory_.states = prog_->GetStateSamples();
  trajectory_.times = prog_->GetSampleTimes();
  Eigen::VectorXd ego_x = trajectory_.states.row(ego_indices_.at(kX));
  Eigen::VectorXd ego_y = trajectory_.states.row(ego_indices_.at(kY));
  std::cout << " Sample times: " << trajectory_.times << std::endl;
  std::cout << " Inputs for ego car: " << trajectory_.inputs.row(0) << std::endl;
  std::cout << "                     " << trajectory_.inputs.row(1) << std::endl;
  // TODO(jadecastro) Transform this cost into a probability distribution.
  std::cout << " Optimal Cost: " << prog_->GetOptimalCost() << std::endl;
  CallPython("figure", 1);
  CallPython("clf");
  CallPython("plot", ego_x, ego_y);
  CallPython("setvars", "ego_x", ego_x, "ego_y", ego_y);
  CallPython("plt.xlabel", "ego car x (m)");
  CallPython("plt.ylabel", "ego car y (m)");
  CallPython("plt.show");
  for (int i{0}; i < kNumAdoCars; ++i) {
    Eigen::VectorXd ado_x = trajectory_.states.row(ado_indices_[i].at(kX));
    Eigen::VectorXd ado_y = trajectory_.states.row(ado_indices_[i].at(kY));
    CallPython("figure", i+2);
    CallPython("clf");
    CallPython("plot", ado_x, ado_y);
    CallPython("setvars", "ado_x_" + std::to_string(i), ado_x,
               "ado_y_" + std::to_string(i), ado_y);
    CallPython("plt.xlabel", "ado car "+ std::to_string(i) +" x (m)");
    CallPython("plt.ylabel", "ado car "+ std::to_string(i) +" y (m)");
    CallPython("plt.show");
  }

  // Dump the entire solution result to the screen.
  // for (int i{0}; i < trajectory_.states.size(); ++i) {
  //   std::cout << " states(" << i << ") " << trajectory_.states(i) << std::endl;
  // }

  std::cout << " SOLUTION RESULT: " << result << std::endl;

  //if (result == solvers::SolutionResult::kSolutionFound) {
  /*
  if (true) {
    // Build another simulator with LCM capability and run in play-back mode.
    auto simulator_lcm = SetupSimulator(true, kNumLanes,
                                        kNumLaneChangingCars);
    simulator_lcm->Build();

    // Pipe the offending initial condition into AutomotiveSimulator.
    // TODO(jadecastro): Set up a trajectory-playback Diagram.
    const auto& plant_lcm = simulator_lcm->GetDiagram();
    auto context_lcm = plant_lcm.CreateDefaultContext();
    context_lcm->get_mutable_continuous_state().SetFromVector(trajectory_.states.col(0));

    simulator_lcm->Start(1., std::move(context_lcm));
    simulator_lcm->StepBy(10.);
  }
  */
}

}  // namespace automotive
}  // namespace drake
