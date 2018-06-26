#pragma once

#include "drake/automotive/scenario.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace automotive {

class TrajectoryOptimization final {
 public:
  using SubVectorXDecisionVariable =
      Eigen::VectorBlock<Eigen::Block<const solvers::VectorXDecisionVariable, -1, 1>>;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryOptimization)

  TrajectoryOptimization(std::unique_ptr<Scenario> scenario,
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
  // TODO(jadecastro) Relax the need for an fixed time vector, e.g. by imposing
  // `time() == t` when adding this constraint.
  void SetEgoLinearConstraint(const Eigen::Ref<const Eigen::MatrixXd> A,
                              const Eigen::Ref<const Eigen::VectorXd> b,
                              double t);

  /// Attempts to solve the falsification problem.
  void Solve();

  /// @name Convenience getters for the underlying states.
  /// @{
  solvers::VectorXDecisionVariable get_state(
      const systems::System<double>* subsystem) const;

  SubVectorXDecisionVariable get_state(
      int index, const systems::System<double>* subsystem) const;

  SubVectorXDecisionVariable get_initial_state(
      const systems::System<double>* subsystem) const;

  SubVectorXDecisionVariable get_final_state(
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
