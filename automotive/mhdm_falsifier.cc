#include <memory>

#include <gflags/gflags.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/simple_car.h"
#include "drake/common/proto/call_python.h"
#include "drake/common/symbolic.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
"Number of seconds to simulate.");

namespace drake {
namespace automotive {
namespace {

using common::CallPython;
using systems::trajectory_optimization::DirectCollocation;

// Road parameters -- Assume fixed for now.
static constexpr double kLaneWidth = 3.;
static constexpr double kDragwayLength = 150.;
static const std::string kEgoCarName = "ego_car";
static const std::string kLaneChangerName = "lane_changer";

static std::unique_ptr<AutomotiveSimulator<double>>
SetupSimulator(bool is_playback_mode, int num_dragway_lanes,
               int num_lane_changers) {
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry =
      std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"Dircol Test Dragway"}),
          num_dragway_lanes , kDragwayLength, kLaneWidth,
          0. /* shoulder width */, 5. /* maximum_height */,
          std::numeric_limits<double>::epsilon() /* linear_tolerance */,
          std::numeric_limits<double>::epsilon() /* angular_tolerance */);
  DRAKE_DEMAND(num_dragway_lanes > 0);

  auto simulator = (is_playback_mode)
      ? std::make_unique<AutomotiveSimulator<double>>(
          std::make_unique<lcm::DrakeLcm>())
      : std::make_unique<AutomotiveSimulator<double>>(nullptr);

  auto simulator_road = simulator->SetRoadGeometry(std::move(road_geometry));
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

  simulator->AddIdmControlledCar(kEgoCarName,
                                 true /* move along the "s"-direction */,
                                 ego_initial_state, ego_goal_lane,
                                 ScanStrategy::kPath,
                                 RoadPositionStrategy::kExhaustiveSearch,
                                 0. /* (unused) */);

  for (int i{0}; i < num_lane_changers; ++i) {
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
    simulator->AddIdmControlledCar(kLaneChangerName + "_" + std::to_string(i),
                                   true /* move along the "s"-direction */,
                                   lc_initial_state, ego_goal_lane,
                                   ScanStrategy::kPath,
                                   RoadPositionStrategy::kExhaustiveSearch,
                                   0. /* (unused) */);
  }
  return std::unique_ptr<AutomotiveSimulator<double>>(simulator.release());
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
void PopulateInitialConditions(
    const systems::Diagram<double>& diagram,
    const SimpleCarState<double>& ego_initial_conditions,
    const std::vector<SimpleCarState<double>*>& lc_initial_conditions,
    std::vector<const systems::System<double>*>& systems,
    systems::Context<double>* context) {
  int i{0};
  for (const auto& system : systems) {
    std::string name = system->get_name();
    std::cout << " NAME " << name << std::endl;
    if (name.find(kEgoCarName + "_simple_car") != std::string::npos) {
      SetSimpleCarState(diagram, *system, ego_initial_conditions, context);
    } else if ((name.find(kLaneChangerName) != std::string::npos) &&
               (name.find("simple_car") != std::string::npos)) {
      SetSimpleCarState(diagram, *system, *lc_initial_conditions[i++], context);
      std::cout << "  Ado ic "
                << context->get_continuous_state_vector().CopyToVector()
                << std::endl;
    }
  }
}

// TODO(jadecastro) Diagram context yearns for a better way at mapping
// subcontext to subsystem.  In particular, we could take better advantage of
// named vectors.
void PopulateCoeffContexts(
    int num_lane_changers, const systems::Diagram<double>& diagram,
    const std::vector<const systems::System<double>*>& systems,
    std::vector<std::unique_ptr<systems::Context<double>>>* coeff_contexts) {
  DRAKE_DEMAND(coeff_contexts->size() == 0);

  // Ado - Ego.
  SimpleCarState<double> ego_coefficients;
  ego_coefficients.set_x(-1.);
  SimpleCarState<double> all_lc_coefficients;
  all_lc_coefficients.set_x(1.);

  for (int i{0}; i < num_lane_changers; ++i) {
    coeff_contexts->emplace_back(diagram.CreateDefaultContext());
    for (const auto& system : systems) {
      std::string name = system->get_name();
      std::cout << " NAME " << name << std::endl;
      if (name.find(kEgoCarName + "_simple_car") != std::string::npos) {
        SetSimpleCarState(diagram, *system, ego_coefficients,
                          (*coeff_contexts)[i].get());
        break;
      }
    }
  }
  int i{0};
  for (const auto& system : systems) {
    std::string name = system->get_name();
    if ((name.find(kLaneChangerName) != std::string::npos) &&
        (name.find("simple_car") != std::string::npos)) {
      SetSimpleCarState(diagram, *system, all_lc_coefficients,
                        (*coeff_contexts)[i++].get());
      std::cout << "  coeff "
                << (*coeff_contexts)[i-1]->get_continuous_state_vector().
          CopyToVector()
                << std::endl;
    }
  }
}

int DoMain(void) {
  const int kNumLanes = 3;  // Number of lanes in the scenario.
  const int kNumLaneChangingCars = 4;  // Number of ado cars in this scenario.

  auto simulator = SetupSimulator(false /* is_playback_mode */, kNumLanes,
                                  kNumLaneChangingCars);

  simulator->BuildAndInitialize();

  const systems::System<double>& plant = simulator->GetDiagram();
  auto context = plant.CreateDefaultContext();

  // Set up a direct-collocation feasibility problem.
  const double guess_duration = 2.;  // seconds
  const int kNumTimeSamples = 100;
  const double kTrajectoryTimeLowerBound = 0.8 * guess_duration;
  const double kTrajectoryTimeUpperBound = 1.2 * guess_duration;

  DirectCollocation prog(&plant, *context, kNumTimeSamples,
                         kTrajectoryTimeLowerBound, kTrajectoryTimeUpperBound);

  // Ensure that time intervals are evenly spaced.
  prog.AddTimeIntervalBounds(kTrajectoryTimeLowerBound / (kNumTimeSamples - 1),
                             kTrajectoryTimeUpperBound / (kNumTimeSamples - 1));

  const double delta_y =  0.;  // 0.5 * kLaneWidth;
  VectorX<double> vect0(14);
  VectorX<double> vectf(14);

  const auto& diagram = dynamic_cast<const systems::Diagram<double>&>(plant);

  // Supply ICs externally to main().
  SimpleCarState<double> ego_initial_conditions;
  ego_initial_conditions.set_x(30.);
  ego_initial_conditions.set_y(-1. * kLaneWidth + delta_y);
  ego_initial_conditions.set_heading(0.);
  ego_initial_conditions.set_velocity(5.5);

  std::vector<SimpleCarState<double>*> lc_initial_conditions;
  SimpleCarState<double> car0;
  car0.set_x(40.);
  car0.set_y(-1. * kLaneWidth + delta_y);
  car0.set_heading(0.);
  car0.set_velocity(5.);
  lc_initial_conditions.push_back(&car0);

  SimpleCarState<double> car1;
  car1.set_x(60.);
  car1.set_y(0. * kLaneWidth + delta_y);
  car1.set_heading(0.);
  car1.set_velocity(8.);
  lc_initial_conditions.push_back(&car1);

  SimpleCarState<double> car2;
  car2.set_x(30.);
  car2.set_y(1. * kLaneWidth + delta_y);
  car2.set_heading(0.);
  car2.set_velocity(5.);
  lc_initial_conditions.push_back(&car2);

  SimpleCarState<double> car3;
  car3.set_x(20.);
  car3.set_y(1. * kLaneWidth + delta_y);
  car3.set_heading(0.);
  car3.set_velocity(10.);
  lc_initial_conditions.push_back(&car3);

  std::vector<const systems::System<double>*> systems = diagram.GetSystems();
  auto initial_context = diagram.CreateDefaultContext();
  PopulateInitialConditions(diagram, ego_initial_conditions,
                            lc_initial_conditions, systems,
                            initial_context.get());

  // Assumes we have two types of cars: one ego car and several ado cars.
  // TODO(jadecastro) This is fragile to kNumLaneChangingCars.
  std::vector<std::unique_ptr<systems::Context<double>>> coeff_contexts;
  PopulateCoeffContexts(kNumLaneChangingCars, diagram, systems,
                        &coeff_contexts);

  // Create an inequality constraint of the form Ax <= b.
  // coeff_context->get_continuous_state_vector().get

  // Parse the initial conditions.

  // TODO: Reverse the ordering to match Dircol's decision vars?
  // TODO(jadecastro) Dircol strips the context, so the ordering here is a
  // extremely fragile.  Perhaps Dircol should keep a mapping of DVs to
  // continuous states?
  const systems::VectorBase<double>& initial_states =
      initial_context->get_continuous_state_vector();
  for (int i{0}; i < initial_states.size(); ++i) {
    prog.AddLinearConstraint(prog.initial_state()(i) ==
                             initial_states.GetAtIndex(i));
  }

  // Solve a collision with one of the cars, in this case, Traffic Car 2 merging
  // into the middle lane.  Assume for now that, irrespective of its
  // orientation, the traffic car occupies a lane-aligned bounding box.

  // TODO: Package these loops into function calls or use a lambdas?
  // TODO(jadecastro) As above, this assumes the ordering of the context. Update
  // Dircol to make this less fragile.
  const int x_index = SimpleCarStateIndices::kX;
  const int y_index = SimpleCarStateIndices::kY;
  int ego_index{0};
  int ado_index{0};
  for (const auto& system : systems) {
    std::string name = system->get_name();
    if (name.find(kEgoCarName + "_simple_car") != std::string::npos) {
      break;
    }
    if (name.find("simple_car") != std::string::npos) {
      ego_index += SimpleCarStateIndices::kNumCoordinates;
    }
  }

  for (const auto& system : systems) {
    std::string name = system->get_name();
    if ((name.find(kLaneChangerName) != std::string::npos) &&
        (name.find("_2_") != std::string::npos) &&
        (name.find("simple_car") != std::string::npos)) {
      std::cout << " ego index " << ego_index << std::endl;
      std::cout << " ado index " << ado_index << std::endl;
      // Assumes the footprint is a full lane-width wide by 5-m long.
      prog.AddLinearConstraint(prog.final_state()(ado_index + y_index) >=
                               prog.final_state()(ego_index + y_index)
                               - 0.5 * kLaneWidth);
      prog.AddLinearConstraint(prog.final_state()(ado_index + x_index) >=
                               prog.final_state()(ego_index + x_index) - 2.5);
      prog.AddLinearConstraint(prog.final_state()(ado_index + x_index) <=
                               prog.final_state()(ego_index + x_index) + 2.5);
    }
    if (name.find("simple_car") != std::string::npos) {
      ado_index += SimpleCarStateIndices::kNumCoordinates;
    }
  }

  // Add a cost to encourage solutions that minimize the distance to a crash.
  //prog.AddRunningCost((prog.state()(4) - prog.state()(0)) *
  //                    (prog.state()(4) - prog.state()(0)) +
  //                    (prog.state()(5) - (-0.5 * kLaneWidth + delta_y)) *
  //                    (prog.state()(5) - (-0.5 * kLaneWidth + delta_y)));

  // systems::BasicVector<double> x0 = ;
  // systems::BasicVector<double> xf(vectf);
  // auto guess_state_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
  //     {0, guess_duration}, {x0.get_value(), xf.get_value()});

  // prog.SetInitialTrajectory(PiecewisePolynomial<double>(),
  //                           guess_state_trajectory);

  const auto result = prog.Solve();

  // Extract the initial context from prog and plot the solution using MATLAB.
  // To view, type `bazel run //common/proto:call_python_client_cli`.
  Eigen::MatrixXd inputs = prog.GetInputSamples();
  Eigen::MatrixXd states = prog.GetStateSamples();
  Eigen::VectorXd times_out = prog.GetSampleTimes();
  const Eigen::VectorXd s0 = states.row(0);
  const Eigen::VectorXd s1 = states.row(1);
  const Eigen::VectorXd s10 = states.row(10);
  const Eigen::VectorXd s11 = states.row(11);
  std::cout << " states.row(0) " << states.row(0) << std::endl;
  std::cout << " states.row(1) " << states.row(1) << std::endl;
  if (kNumLanes == 3) {
    CallPython("figure", 1);
    CallPython("clf");
    CallPython("plot", s10, s11);
    CallPython("setvars", "s10", s10, "s11", s11);
    CallPython("plt.xlabel", "x ego (m)");
    CallPython("plt.ylabel", "y ego (m)");
    CallPython("figure", 2);
    CallPython("clf");
    CallPython("plot", s0, s1);
    CallPython("setvars", "s0", s0, "s1", s1);
    CallPython("plt.xlabel", "x traffic2 (m)");
    CallPython("plt.ylabel", "y traffic2 (m)");
  } else if (kNumLanes == 2) {
    CallPython("figure", 1);
    CallPython("clf");
    CallPython("plot", states.row(4), states.row(5));
    CallPython("plt.xlabel", "x ego (m)");
    CallPython("plt.ylabel", "y ego (m)");
    CallPython("figure", 2);
    CallPython("clf");
    CallPython("plot", states.row(0), states.row(2));
    CallPython("plt.xlabel", "x traffic1 (m)");
    CallPython("plt.ylabel", "x traffic0 (m)");
  } else if (kNumLanes == 1) {
    CallPython("figure", 1);
    CallPython("clf");
    CallPython("plot", states.row(0), states.row(0) - states.row(2));
    CallPython("plt.xlabel", "s lead (m)");
    CallPython("plt.ylabel", "s lead - s ego (m)");
  }

  // Dump the entire solution result to the screen.
  // for (int i{0}; i < states.size(); ++i) {
  //   std::cout << " states(" << i << ") " << states(i) << std::endl;
  // }

  std::cout << " SOLUTION RESULT: " << result << std::endl;

  //if (result == solvers::SolutionResult::kSolutionFound) {
  if (true) {
    // Build another simulator with LCM capability and run in play-back mode.
    auto simulator_lcm = SetupSimulator(true /* is_playback_mode */, kNumLanes,
                                        kNumLaneChangingCars);
    simulator_lcm->Build();

    // Pipe the offending initial condition into AutomotiveSimulator.
    // TODO(jadecastro): Set up a trajectory-playback Diagram.
    const auto& plant_lcm = simulator_lcm->GetDiagram();
    auto context_lcm = plant_lcm.CreateDefaultContext();
    context_lcm->get_mutable_continuous_state().SetFromVector(states.col(0));

    simulator_lcm->Start(1., std::move(context_lcm));
    simulator_lcm->StepBy(10.);
  }

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::automotive::DoMain();
}
