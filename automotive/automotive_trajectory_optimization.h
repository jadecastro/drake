#pragma once

#include <cmath>
#include <memory>
#include <vector>

#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/simple_car.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace automotive {

// ** TODO ** Can we remove this if we have
//            SimpleCarState<symbolic::Expression>?
struct SimpleCarSymbolicState {
  symbolic::Expression x;
  symbolic::Expression y;
  symbolic::Expression heading;
  symbolic::Expression velocity;
};

class Scenario final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Scenario)

  Scenario(int num_lanes, double lane_width, double road_length,
           double car_width, double car_length);

  // ** TODO ** These can only be called *before* Build().
  // ** TODO ** Put these into a subclass?
  // @{
  const systems::System<double>* AddIdmSimpleCar(const std::string& name);

  const systems::System<double>* AddSimpleCar(const std::string& name);

  void FixGoalLaneDirection(const systems::System<double>& subsystem,
                            const LaneDirection& lane_direction);
  // @}

  void Build();

  /// @name Road and geometry accessors.
  /// @{
  const maliput::api::RoadGeometry& road() const { return *road_; }
  double car_width() const { return car_width_; }
  double car_length() const { return car_length_; }
  /// @}

  // ** TODO ** The remainder of these public methods can only be called *after* Build().
  void SetInitialSubsystemStateBounds(
      const systems::System<double>& subsystem,
      const SimpleCarState<double>& lb_value,
      const SimpleCarState<double>& ub_value);

  void SetFinalSubsystemState(const systems::System<double>& subsystem,
                              const SimpleCarState<double>& value);

  /// Returns a vector of indices corresponding to a particular `subsystem` in
  /// the scenario.
  std::vector<int> GetStateIndices(
      const systems::System<double>& subsystem) const;

  /// @name System accessors.
  /// @{
  const systems::Diagram<double>& diagram() const { return *scenario_diagram_; }
  const systems::Context<double>& initial_context_lb() const {
    return *initial_context_lb_;
  }
  const systems::Context<double>& initial_context_ub() const {
    return *initial_context_ub_;
  }
  const systems::Context<double>& final_context() const {
    return *final_context_;
  }
  std::vector<const systems::System<double>*> aliases() const {
    return aliases_;
  }
  const systems::System<double>* ego_alias() const { return ego_alias_; }
  /// @}

 private:
  std::unique_ptr<maliput::api::RoadGeometry> road_;
  const double car_width_{0.};
  const double car_length_{0.};

  // Temporaries that move to DiagramBuilder once BuildScenario is called.
  std::vector<std::unique_ptr<systems::Diagram<double>>> ado_cars_{};
  std::unique_ptr<SimpleCar<double>> ego_car_;

  const systems::System<double>* ego_alias_{nullptr};
  std::vector<const systems::System<double>*>
      aliases_{};  // Double check if even needed!

  std::map<const systems::System<double>*, LaneDirection> goal_lane_map_;

  // Indices used for accessing the decision variables.
  std::vector<int> ego_indices_{};
  std::vector<std::vector<int>> ado_indices_{};

  std::unique_ptr<systems::Diagram<double>> scenario_diagram_;
  std::unique_ptr<systems::Context<double>> initial_context_lb_;
  std::unique_ptr<systems::Context<double>> initial_context_ub_;
  std::unique_ptr<systems::Context<double>> final_context_;

  int noise_inport_{};
  int traffic_inport_{};
  int lane_inport_{};
  int pose_outport_{};
  int velocity_outport_{};
};

// ** TODO ** Rename to AutomotiveTrajectoryOptimization.
class AutomotiveTrajectoryOptimization final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AutomotiveTrajectoryOptimization)

  AutomotiveTrajectoryOptimization(std::unique_ptr<Scenario> scenario,
                                   int num_time_samples, double min_time_step,
                                   double max_time_step,
                                   double initial_guess_duration_sec);

  /// Container with the trajectory solver result.
  struct InputStateTrajectoryData {
    Eigen::MatrixXd inputs;
    Eigen::MatrixXd states;
    Eigen::VectorXd times;
  };

  /// Initializes the guess trajectory as a linear trajectory connecting the
  /// provided initial conditions and final conditions, as defined in
  /// SetInitialSubsystemStateBounds() and SetFinalSubsystemState().  The
  /// initial point taken at the centroid of the provided initial bounding box.
  void SetLinearGuessTrajectory();

  // ** TODO **
  // void SetGuessTrajectory(const Eigen::VectorX& times,
  //                         const Eigen::MatrixX& states);

  /// Adds lane bounds on the system, requiring a particular car to stay within
  /// the prescribed set of `lane_bounds`, for a dragway::RoadGeometry.  If the
  /// given lanes are not contiguous, the car is required to stay within the
  /// hull formed from the specified pair of lanes.
  void SetDragwayLaneBounds(
      const systems::System<double>& subsystem,
      std::pair<const maliput::api::Lane*, const maliput::api::Lane*>
          lane_bounds);

  /// Adds a constraint at the final time step for collision between the car in
  /// `subsystem` with the ego car.  We prevent a disjunctive expression in the
  /// exact formulation by approximating the constraint as finding the
  /// hull-containment of a bisecting point between the centers of both cars.
  void AddFinalCollisionConstraints(const systems::System<double>& subsystem);

  /// Add cost representing the noise perturbation from the inputs of a nominal
  /// controller.  We assume that the noise has a state-independent, zero-mean
  /// Gaussian distribution.
  //
  // ** TODO ** Separate out Sigmas for steering and acceleration, and allow
  //            different values for each car.
  void AddGaussianCost();

  /// Sets the affine constraint:
  ///     A * x_ego(t) ≤ b
  /// at time `t` on the ego car state (Note: does NOT apply the constraint on
  /// the car's convex hull; only its center).
  ///
  /// Since DirectCollocation adds constraints only at requested _indices_, we
  /// require that `min_time_step` == `max_time_step`.  Throws if `t` is less
  /// than zero or greater than the trajectory time, or if `A` and `b` are
  /// ill-dimensioned.
  //
  // TODO(jadecastro) Relax the invariant time vector, e.g. by imposing `time()
  // == t` when adding this constraint.
  void SetEgoLinearConstraint(const Eigen::Ref<const Eigen::MatrixXd> A,
                              const Eigen::Ref<const Eigen::VectorXd> b,
                              double t);

  /// Attempts to solve the falsification problem.
  void Solve();

  /// @name Convenience getters for the underlying states.
  // ** TODO ** Decide if these actually work for us!!
  /// @{
  SimpleCarSymbolicState get_state(
      const systems::System<double>* subsystem) const;

  SimpleCarSymbolicState get_initial_state(
      const systems::System<double>* subsystem) const;

  SimpleCarSymbolicState get_final_state(
      const systems::System<double>* subsystem) const;
  /// @}

  /// Retuns the total probability of the solution under the cost function
  /// specified under AddGaussianCost(), as a zero-mean Gaussian probability
  /// density function representing the deviation from the inputs from the
  /// nominal controller.
  //
  // ** TODO **
  double GetSolutionTotalProbability() const;

  /// Extracts the initial context from prog and plots the solution using
  /// python.  To view, type `bazel run //common/proto:call_python_client_cli`.
  void PlotSolution();

  // ** TODO **
  void AnimateSolution() const;

  const Scenario& scenario() const { return *scenario_; }

  systems::trajectory_optimization::DirectCollocation* prog() const {
    return prog_.get();
  }

  const InputStateTrajectoryData& get_trajectory() const {
    // ** TODO ** check for something like is_solved?
    return trajectory_;
  }

 private:
  void FixInitialConditions();

  const int num_time_samples_{};
  const double min_time_step_{};
  const double max_time_step_{};
  const double initial_guess_duration_sec_{};
  std::unique_ptr<Scenario> scenario_;

  std::unique_ptr<systems::trajectory_optimization::DirectCollocation> prog_;
  solvers::SolutionResult result_;
  InputStateTrajectoryData trajectory_;
};

}  // namespace automotive
}  // namespace drake
