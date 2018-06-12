#pragma once

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

class Scenario final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Scenario)

  Scenario(int num_lanes, double lane_width, double road_length);

  // ** TODO ** These can only be called *before* Build().
  // @{
  const systems::System<double>* AddIdmSimpleCar(const std::string& name);

  const systems::System<double>* AddSimpleCar(const std::string& name);

  void FixGoalLaneDirection(const systems::System<double>& subsystem,
                            const LaneDirection& lane_direction);
  // @}

  void Build();

  // ** TODO ** These can only be called *after* Build().
  // @{
  void SetInitialSubsystemState(const systems::System<double>& subsystem,
                                const SimpleCarState<double>& value);

  void SetFinalSubsystemState(const systems::System<double>& subsystem,
                              const SimpleCarState<double>& value);

  /// Returns a vector of indices corresponding to a particular `subsystem` in
  /// the scenario.
  std::vector<int> GetStateIndices(
      const systems::System<double>& subsystem) const;

  const maliput::api::RoadGeometry& road() const { return *road_; }
  const systems::Diagram<double>& diagram() const { return *scenario_diagram_; }
  const systems::Context<double>& initial_context() const {
    return *initial_context_;
  }
  const systems::Context<double>& final_context() const {
    return *final_context_;
  }
  std::vector<const systems::System<double>*> aliases() const {
    return aliases_;
  }
  const systems::System<double>* ego_alias() const { return ego_car_.get(); }
  // @}

 private:
  std::unique_ptr<maliput::api::RoadGeometry> road_;

  // Temporaries that move to DiagramBuilder once BuildScenario is called.
  std::vector<std::unique_ptr<systems::Diagram<double>>> ado_cars_{};
  std::unique_ptr<SimpleCar<double>> ego_car_;

  std::vector<const systems::System<double>*>
      aliases_{};  // Double check if even needed!

  std::map<const systems::System<double>*, LaneDirection> goal_lane_map_;

  // Indices used for accessing the decision variables.
  std::vector<int> ego_indices_{};
  std::vector<std::vector<int>> ado_indices_{};

  std::unique_ptr<systems::Diagram<double>> scenario_diagram_;
  std::unique_ptr<systems::Context<double>> initial_context_;
  std::unique_ptr<systems::Context<double>> final_context_;
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

  void SetLinearGuessTrajectory();

  // ** TODO **
  // void SetGuessTrajectory(const Eigen::VectorX& times,
  //                         const Eigen::MatrixX& states);

  // N.B. Assumes Dragway.
  void SetLateralLaneBounds(
      const systems::System<double>* subsystem,
      std::pair<const maliput::api::Lane*, const maliput::api::Lane*>
          lane_bounds);

  // Add cost representing the noise perturbation from a deterministic evolution
  // of IDM/PurePursuit (nominal policy).  We assume that the policy has a
  // state-independent, zero-mean Gaussian distribution.
  // ** TODO ** Separate out Sigmas for steering and acceleration.
  void AddLogProbabilityCost();

  // Add a log-probability contraints representing the deviation of the commands
  // from a deterministic evolution of IDM/PurePursuit (nominal policy).  We
  // assume that the policy has zero-mean Gaussian distribution.  ** TODO **
  // Separate out Sigmas for steering and acceleration.
  void AddLogProbabilityChanceConstraint();

  /// Sets an affine constraint at time t.
  void SetEgoLinearConstraint(const Eigen::Ref<const Eigen::MatrixXd> A,
                              const Eigen::Ref<const Eigen::VectorXd> b,
                              double t);

  void Solve();

  /// Extracts the initial context from prog and plot the solution using python.
  /// To view, type `bazel run //common/proto:call_python_client_cli`.
  void PlotResult();

  void SimulateResult() const;

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
