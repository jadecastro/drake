#pragma once

#include "drake/automotive/scenario.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace automotive {

enum class WhichLimit { kHigh = 0, kLow = 1, kBoth = 2 };

class TrajectoryOptimization final {
 public:
  using SubVectorXDecisionVariable = Eigen::VectorBlock<
      Eigen::Block<const solvers::VectorXDecisionVariable, -1, 1>>;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryOptimization)

  /// @param bounding_box_limit If greater than 0, applies a bounding box of
  /// this value to all states.
  TrajectoryOptimization(std::unique_ptr<Scenario> scenario,
                         int num_time_samples, double min_time_step,
                         double max_time_step,
                         double initial_guess_duration_sec,
                         double bounding_box_limit);

  /// Container with the trajectory solver result.
  struct InputStateTrajectoryData {
    Eigen::VectorXd times;
    Eigen::MatrixXd inputs;
    Eigen::MatrixXd states;

    std::map<const systems::System<double>*, Eigen::VectorXd> x;
    std::map<const systems::System<double>*, Eigen::VectorXd> y;
    std::map<const systems::System<double>*, Eigen::VectorXd> heading;
  };

  template <typename T>
  struct Cartesian {
    T x;
    T y;
    T heading;
  };

  /// Registers the initial set for a `subsystem` in the scenario Diagram as a
  /// bounding box with the given `initial_state_lb` (lower-bound) and
  /// `initial_state_ub` (upper-bound).  Does nothing without the additional
  /// (optional) calls to SetLinearGuessTrajectory(), AddInitialConstraints(),
  /// and AddFinalConstraints().
  void RegisterInitialBoxConstraint(
      const systems::System<double>& subsystem,
      const systems::BasicVector<double>& initial_state_lb,
      const systems::BasicVector<double>& initial_state_ub);

  /// Registers the intitial condition for a `subsystem in the scenario Diagram
  /// at the provided `initial_state`.  Does nothing without the additional
  /// (optional) calls to SetLinearGuessTrajectory(), AddInitialConstraints(),
  /// and AddFinalConstraints().
  void RegisterInitialConstraint(
      const systems::System<double>& subsystem,
      const systems::BasicVector<double>& initial_state);

  /// Registers the final set for a `subsystem` in the scenario Diagram as a
  /// bounding box with the given `initial_state_lb` (lower-bound) and
  /// `initial_state_ub` (upper-bound).  Does nothing without the additional
  /// (optional) calls to SetLinearGuessTrajectory() and AddFinalConstraints().
  void RegisterFinalBoxConstraint(
      const systems::System<double>& subsystem,
      const systems::BasicVector<double>& final_state_lb,
      const systems::BasicVector<double>& final_state_ub);

  /// Registers the final condition for a `subsystem in the scenario Diagram
  /// at the provided `initial_state`.  Does nothing without the additional
  /// (optional) calls to SetLinearGuessTrajectory(), AddInitialConstraints(),
  /// and AddFinalConstraints().
  void RegisterFinalConstraint(const systems::System<double>& subsystem,
                               const systems::BasicVector<double>& final_state);

  /// Adds the initial constraints given in
  /// RegisterSubsystemInitial{Box}Constraint() to the DirectCollocation
  /// program.  Uses the default Context for subsystems with un-registered
  /// constraints.
  void AddInitialConstraints();

  /// Adds the final constraints given in
  /// RegisterSubsystemFinal{Box}Constraint() to the DirectCollocation
  /// program.  Uses the default Context for subsystems with un-registered
  /// constraints.
  void AddFinalConstraints();

  /// Initializes the guess trajectory to one that linearly connects the
  /// provided initial conditions and final conditions, as defined in
  /// RegisterSubsystemInitial{Box}Constraint() and
  /// RegisterSubsystemInitial{Box}Constraint().  Points are taken at the
  /// centroids of the provided bounding boxes.
  void SetLinearGuessTrajectory();

  // ** TODO **
  void SetGuessTrajectory(
    const trajectories::PiecewisePolynomial<double>& traj_u);

  /// Adds lane bounds on the system, requiring a particular car to stay within
  /// the prescribed set of `lane_bounds`, for a dragway::RoadGeometry.  If the
  /// given lanes are not contiguous, the car is required to stay within the
  /// hull formed from the specified pair of lanes.
  void AddLaneConstraints(
      const systems::System<double>& subsystem,
      std::pair<const maliput::api::Lane*, const maliput::api::Lane*>
         lane_bounds,
      WhichLimit which_limit = WhichLimit::kBoth);

  /// Adds a constraint at the final time step for collision between two cars,
  /// `subsystem1` and `subsystem2`.  We prevent a disjunctive expression in the
  /// exact formulation by approximating the constraint as finding the
  /// hull-containment of a bisecting point between the centers of both cars.
  void AddFinalCollisionConstraintsOld(
      const systems::System<double>& subsystem1,
      const systems::System<double>& subsystem2);

  /// Adds a constraint at the final time step for collision between two cars,
  /// `subsystem1` and `subsystem2`.  We use an inscribing-ellipse approach.
  void AddFinalCollisionConstraints(const systems::System<double>& subsystem1,
                                    const systems::System<double>& subsystem2);

  /// Add a cost representing state-independent, zero-mean Gaussian
  /// distribution, with covariance `sigma`, to the inputs of the provided
  /// `subsystem`.
  void AddGaussianCost(const systems::System<double>& subsystem,
                       const Eigen::MatrixXd& sigma);

  /// Adds a chance constraint under the cost function specified under
  /// AddGaussianCost(), as a zero-mean Gaussian probability density function
  /// representing the deviation from the inputs from the nominal controller.
  //
  // ** TODO **
  void AddGaussianTotalProbabilityConstraint(
      const systems::System<double>& subsystem);

  /// Sets the affine constraint:
  ///     A * x_ego(t) ≤ b
  /// at time `t` on the provided `subsystem` state (Note: does NOT apply the
  /// constraint on the car's convex hull; only its center).
  ///
  /// Since DirectCollocation adds constraints only at requested _indices_, we
  /// require that `min_time_step` == `max_time_step`.  Throws if `t` is less
  /// than zero or greater than the trajectory time, or if `A` and `b` are
  /// ill-dimensioned.
  //
  // TODO(jadecastro) Relax the need for an fixed time vector, e.g. by imposing
  // `time() == t` when adding this constraint.
  void AddLinearConstraint(const systems::System<double>& subsystem,
                           const Eigen::Ref<const Eigen::MatrixXd> A,
                           const Eigen::Ref<const Eigen::VectorXd> b, double t);

  /// Attempts to solve the trajectory optimization problem.
  solvers::SolutionResult Solve();

  /// @name Convenience getters for decision variables associated with a
  /// particular subsystem.
  /// @{

  /// Accessor for the input vector at all times for `subsystem`.
  solvers::VectorXDecisionVariable get_input(
      const systems::System<double>& subsystem) const;

  /// Accessor for the input vector at time `t` for `subsystem`.
  SubVectorXDecisionVariable get_input(
      double t, const systems::System<double>& subsystem) const;

  /// Accessor for the state vector at all times for `subsystem`.
  solvers::VectorXDecisionVariable get_state(
      const systems::System<double>& subsystem) const;

  /// Accessor for the state vector at time `t` for `subsystem`.
  SubVectorXDecisionVariable get_state(
      double t, const systems::System<double>& subsystem) const;

  /// Accessor for the initial state for `subsystem`.
  SubVectorXDecisionVariable get_initial_state(
      const systems::System<double>& subsystem) const;

  /// Accessor for the final state for `subsystem`.
  SubVectorXDecisionVariable get_final_state(
      const systems::System<double>& subsystem) const;
  /// @}

  /// Accessor for the cartesian components for `subsystem` evaluated at its
  /// `substate`.
  Cartesian<symbolic::Expression> get_cartesian(
      const systems::System<double>& subsystem,
      const VectorX<symbolic::Expression>& substate) const;

  /// Accessor for the initial context of the scenario diagram.  Currently, we
  /// just return one of the vertices of the bounding box specified under
  /// RegisterInitialConstraint.
  const systems::Context<double>& get_initial_context() const;

  /// Retuns the value of the total log-probability density function of the
  /// solution under the cost function specified under AddGaussianCost(), as a
  /// zero-mean Gaussian probability density function representing the deviation
  /// from the inputs from the nominal controller.
  double GetSolutionTotalLogPdf() const;

  /// Retuns the value of the _normalized_ total log-probability density function
  /// of the solution under the cost function specified under AddGaussianCost(),
  /// as a zero-mean Gaussian probability density function representing the
  /// deviation from the inputs from the nominal controller.
  double GetSolutionTotalLogNormalizedPdf() const;

  /// Extracts the initial context from prog and plots the solution using
  /// python.  To view, type `bazel run //common/proto:call_python_client_cli`.
  void PlotSolution();

  // Animates the current solution.
  void AnimateSolution() const;

  // Animates an externally-provided solution.
  void AnimateSolutionFrom(const InputStateTrajectoryData& trajectory) const;
  void AnimateSolutionFrom(const Eigen::VectorXd& t,
                           const Eigen::MatrixXd& states,
                           double final_time) const;

  /// Computes the utility of the solution returned after calling Solve() with
  /// respect to a given "test" hyperplane.
  double ComputeUtility(const Eigen::Ref<const Eigen::VectorXd> a,
                        double b, double t) const;

  /// Accessor to the registered scenario.
  const Scenario& scenario() const { return *scenario_; }

  /// Accessor to the underlying DirectCollocation program.
  systems::trajectory_optimization::DirectCollocation* prog() const {
    return prog_.get();
  }

  /// Accessor to the solution trajectory, once the DirectCollocation problem
  /// has been solved.
  const InputStateTrajectoryData& get_trajectory() const;

  /// Makes a state trajectory from a simulation based on a set of registered
  /// initial states and an input trajectory.
  const trajectories::PiecewisePolynomial<double> MakeStateTrajectory(
      const trajectories::PiecewisePolynomial<double>& traj_u) const;

 private:
  // Convenience utility to set the `context` for the `subsystem` with the
  // desired `state_vector` for the subsystem.
  // ** TODO ** Check that anything similar doesn't already exist.
  void SetSubcontext(const systems::System<double>& subsystem,
                     const systems::BasicVector<double>& state_vector,
                     systems::Context<double>* context);

  const int num_time_samples_{};
  const double min_time_step_{};
  const double max_time_step_{};
  const double initial_guess_duration_sec_{};
  std::unique_ptr<Scenario> scenario_;
  std::unique_ptr<systems::Context<double>> initial_context_ub_;
  std::unique_ptr<systems::Context<double>> initial_context_lb_;
  std::unique_ptr<systems::Context<double>> final_context_ub_;
  std::unique_ptr<systems::Context<double>> final_context_lb_;
  std::map<const systems::System<double>*, Eigen::MatrixXd> sigma_map_{};
  bool is_solved_{false};

  std::unique_ptr<systems::trajectory_optimization::DirectCollocation> prog_;
  solvers::SolutionResult result_;
  InputStateTrajectoryData trajectory_;
};

}  // namespace automotive
}  // namespace drake
