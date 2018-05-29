#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/automotive/automotive_simulator.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace automotive {

class Falsifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Falsifier)

  Falsifier();

  struct InputStateTrajectory {
    Eigen::MatrixXd inputs;
    Eigen::MatrixXd states;
    Eigen::VectorXd times;
  };

  /// Sets an affine constraint at time t.
  void SetEgoLinearConstraint(const Eigen::Ref<const Eigen::MatrixXd> A,
                              const Eigen::Ref<const Eigen::VectorXd> b,
                              double t);

  /// Executes the falsifier.
  void Run();

  /// 
  const InputStateTrajectory& get_trajectory() const { return trajectory_; }

 private:
  std::unique_ptr<AutomotiveSimulator<double>> simulator_;
  std::vector<int> ego_indices_{};
  std::vector<std::vector<int>> ado_indices_{};
  int min_time_step_{};
  int max_time_step_{};
  std::unique_ptr<systems::trajectory_optimization::DirectCollocation> prog_;
  InputStateTrajectory trajectory_;
};

}  // namespace automotive
}  // namespace drake
