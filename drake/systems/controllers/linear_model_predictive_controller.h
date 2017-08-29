#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

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
      std::unique_ptr<systems::Context<double>> context,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      double sampling_time,
      double time_horizon);

  LinearModelPredictiveController(
      std::unique_ptr<systems::System<double>> model,
      const PiecewisePolynomialTrajectory& x0,
      const PiecewisePolynomialTrajectory& u0,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      double sampling_time,
      double time_horizon);

  const InputPortDescriptor<T>& get_state() const {
    return this->get_input_port(input_index_state_);
  }
  const OutputPort<T>& get_control() const {
    return this->get_output_port(output_index_control_);
  }

 protected:
  void CalcControl(const Context<T>& context, BasicVector<T>* control) const;

 private:
  const std::unique_ptr<systems::System<double>> model_;
  const std::unique_ptr<systems::Context<double>> context_;

  const PiecewisePolynomialTrajectory x0_;
  const PiecewisePolynomialTrajectory u0_;

  const int input_index_state_{-1};
  const int output_index_control_{-1};

  const double sampling_time_{};
  const double time_horizon_{};

  std::unique_ptr<systems::LinearSystem<double>> linear_model_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake
