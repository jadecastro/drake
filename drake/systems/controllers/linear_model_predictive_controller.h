#pragma once


#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {

/// A system containing a time-varying :
///
/// @f[ xdot(t) = A(t)x(t) + B(t)u(t)  @f]
/// @f[ y(t) = A(t)x(t) + B(t)u(t)  @f]
///
template <typename T>
class TimeScheduledAffineSystem : public TimeVaryingAffineSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimeScheduledAffineSystem)

  TimeScheduledAffineSystem(const systems::System<double>& model,
                            std::unique_ptr<PiecewisePolynomialTrajectory> x0,
                            std::unique_ptr<PiecewisePolynomialTrajectory> u0,
                            double time_period)
  : TimeVaryingAffineSystem<T>(x0->cols(), u0->cols(),
                               model.get_output_port(0).size(), time_period) {
    // Check x0, u0 against the model.

    x0_ = std::move(x0);
    u0_ = std::move(u0);
    std::unique_ptr<Context<T>> base_context = model.CreateDefaultContext();
    auto state_vector =
        base_context->get_mutable_discrete_state()->get_mutable_vector();
    auto input_vector = std::make_unique<BasicVector<T>>(u0_->cols());

    std::vector<MatrixX<double>> A_vector{};
    std::vector<MatrixX<double>> B_vector{};
    std::vector<MatrixX<double>> C_vector{};
    std::vector<MatrixX<double>> D_vector{};
    const std::vector<double> times =
        x0_->get_piecewise_polynomial().getSegmentTimes();
    for (auto t : times) {
      const auto state_ref = x0_->value(t);
      state_vector->SetFromVector(state_ref);
      const auto input_ref = u0_->value(t);
      input_vector->SetFromVector(input_ref);
      base_context->SetInputPortValue(
          0,
          std::make_unique<FreestandingInputPortValue>(
              std::move(input_vector)));

      // Also perhaps need to set parameters to obtain a linearization about a
      // non-equilibrium condition.

      const std::unique_ptr<LinearSystem<T>> linear_model =
          Linearize(model, *base_context);

      A_vector.emplace_back(linear_model->A());
      B_vector.emplace_back(linear_model->B());
      C_vector.emplace_back(linear_model->C());
      D_vector.emplace_back(linear_model->D());
    }
    A_.reset(new PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::ZeroOrderHold(times, A_vector)));
    B_.reset(new PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::ZeroOrderHold(times, B_vector)));
    C_.reset(new PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::ZeroOrderHold(times, C_vector)));
    D_.reset(new PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::ZeroOrderHold(times, D_vector)));
  }

  ~TimeScheduledAffineSystem() override {}

  VectorX<T> x0(const T& t) const {
    return x0_->value(t);
  }
  VectorX<T> u0(const T& t) const {
    return u0_->value(t);
  }

  MatrixX<T> A(const T& t) const override {
    return A_->value(t);
  }
  MatrixX<T> B(const T& t) const override {
    return B_->value(t);
  }
  VectorX<T> f0(const T& t) const override {
    return Eigen::MatrixXd::Zero(A_->cols(), 0);
  }
  MatrixX<T> C(const T& t) const override {
    return C_->value(t);
  }
  MatrixX<T> D(const T& t) const override {
    return D_->value(t);
  }
  VectorX<T> y0(const T& t) const override {
    return Eigen::MatrixXd::Zero(C_->cols(), 0);
  }

 private:
  // Nominal (reference) trajectories.
  std::unique_ptr<PiecewisePolynomialTrajectory> x0_;
  std::unique_ptr<PiecewisePolynomialTrajectory> u0_;

  std::unique_ptr<PiecewisePolynomialTrajectory> A_;
  std::unique_ptr<PiecewisePolynomialTrajectory> B_;
  std::unique_ptr<PiecewisePolynomialTrajectory> C_;
  std::unique_ptr<PiecewisePolynomialTrajectory> D_;
};


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

  /// Constructor for an unconstrained MPC formulation with linearization
  /// occurring about the provided context, noting that the context must be an
  /// equilbrium point of the plant model.
  ///
  /// @param model The plant model of the System to be controlled.
  /// @param base_context The fixed base point about which to linearize of the
  /// system and regulate the system.
  /// @param Q A symmetric positive semi-definite state cost matrix of size
  /// (num_states x num_states).
  /// @param R A symmetric positive definite control effort cost matrix of size
  /// (num_inputs x num_inputs).
  /// @param time_period The discrete time period (in seconds) at which
  /// controller updates occur.
  /// @param time_horizon The prediction time horizon (seconds).
  ///
  /// @pre model must have discrete states of dimension num_states and inputs
  /// of dimension num_inputs.
  /// @pre base_context must have discrete states set as appropriate for the
  /// given @p model.  The input must also be initialized via
  /// `base_context->FixInputPort(0, u0)`, or otherwise intialized via Diagram.

  // TODO(jadecastro) Get time_period directly from the plant model.
  LinearModelPredictiveController(
      std::unique_ptr<systems::System<double>> model,
      std::unique_ptr<systems::Context<double>> base_context,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      double time_period,
      double time_horizon);

  /// Constructor an unconstrained, trajectory-regulating MPC formulation with
  /// linearization occurring about the provided trajectory.
  ///
  /// @param model The plant model of the System to be controlled.
  /// @param x0 The state trajectory or size num_states.
  /// @param u0 The input trajectory or size num_inputs.
  /// @param Q A symmetric positive semi-definite state cost matrix of size
  /// (num_states x num_states).
  /// @param R A symmetric positive definite control effort cost matrix of size
  /// (num_inputs x num_inputs).
  /// @param time_period The discrete time period (in seconds) at which
  /// controller updates occur.
  /// @param time_horizon The prediction time horizon (seconds).
  ///
  /// @pre model must have discrete states of dimension num_states and inputs
  /// of dimension num_inputs.
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

 private:
  void CalcControl(const Context<T>& context, BasicVector<T>* control) const;

  // Sets up a DirectTranscription problem and solves for the current control
  // input.
  Eigen::VectorXd SetupAndSolveQp(const Eigen::VectorXd& current_state,
                                  double t) const;

  Eigen::VectorXd SetupAndSolveQp(const Context<T>& base_context,
                                  const Eigen::VectorXd& current_state) const;

  const int state_input_index_{-1};
  const int control_output_index_{-1};

  const int num_states_{};
  const int num_inputs_{};

  const std::unique_ptr<systems::System<double>> model_;
  // The base context is the reference point to regulate.
  const std::unique_ptr<systems::Context<double>> base_context_;

  const Eigen::MatrixXd Q_;
  const Eigen::MatrixXd R_;

  const double time_period_{};
  const double time_horizon_{};

  // Descrption of the linearized plant model; non-null iff base_context_ is
  // non-null.
  std::unique_ptr<LinearSystem<double>> linear_model_;
  // Descrption of the linearized scheduled, time-varying plant model; non-null
  // iff base_context_ is null.
  std::unique_ptr<TimeScheduledAffineSystem<double>> scheduled_model_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake
