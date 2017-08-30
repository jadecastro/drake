#pragma once


#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {

/// Implements a basic Model Predictive Controller based on a linearized model.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// @ingroup control_systems
template <typename T>
class LinearModelPredictiveController : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearModelPredictiveController)

  /// Constructor for the most basic, unconstrained MPC formulation with
  /// linearization occurring about the provided context, noting that the
  /// context must be an equilbrium point of the plant model.
  LinearModelPredictiveController(
      std::unique_ptr<systems::System<double>> model,
      std::unique_ptr<systems::Context<double>> base_context,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      double time_period,
      double time_horizon);

  LinearModelPredictiveController(
      std::unique_ptr<systems::System<double>> model,
      std::unique_ptr<PiecewisePolynomialTrajectory> x0,
      std::unique_ptr<PiecewisePolynomialTrajectory> u0,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      double time_period,
      double time_horizon);

  const InputPortDescriptor<T>& get_state_port() const {
    return this->get_input_port(state_input_index_);
  }
  const OutputPort<T>& get_control_port() const {
    return this->get_output_port(control_output_index_);
  }

 protected:
  void CalcControl(const Context<T>& context, BasicVector<T>* control) const;

 private:
  const int state_input_index_{-1};
  const int control_output_index_{-1};

  std::unique_ptr<systems::System<double>> model_;
  std::unique_ptr<systems::Context<double>> base_context_;

  std::unique_ptr<PiecewisePolynomialTrajectory> x0_;
  std::unique_ptr<PiecewisePolynomialTrajectory> u0_;

  const Eigen::MatrixXd Q_;
  const Eigen::MatrixXd R_;

  const double time_period_{};
  const double time_horizon_{};

  std::unique_ptr<LinearSystem<double>> linear_model_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake
