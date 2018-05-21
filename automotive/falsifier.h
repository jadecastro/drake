#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace automotive {

class Falsifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Falsifier)

  Falsifier() = default;

  struct InputStateTrajectory {
    Eigen::MatrixXd inputs;
    Eigen::MatrixXd states;
    Eigen::VectorXd times;
  };

  void Run();

  const InputStateTrajectory& get_trajectory() const { return trajectory_; }

 private:
  InputStateTrajectory trajectory_;
};

}  // namespace automotive
}  // namespace drake
