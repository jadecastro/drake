#include "drake/systems/controllers/linear_model_predictive_controller.h"

#include <string>

#include "drake/systems/trajectory_optimization/direct_transcription.h"

namespace drake {
namespace systems {
namespace controllers {

template <typename T>
LinearModelPredictiveController<T>::LinearModelPredictiveController(
    std::unique_ptr<systems::System<double>> model,
    std::unique_ptr<systems::Context<double>> context,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    double sampling_time,
    double time_horizon)
    : state_input_index_(
          this->DeclareInputPort(kVectorValued, Q.cols()).get_index()),
      control_output_index_(this->DeclareVectorOutputPort(
          BasicVector<T>(R.cols()),
          &LinearModelPredictiveController<T>::CalcControl).get_index()),
      model_(std::move(model)), context_(std::move(context)),
      sampling_time_(sampling_time), time_horizon_(time_horizon) {
  DRAKE_DEMAND(sampling_time > 0.);
  DRAKE_DEMAND(time_horizon > 0.);

  // Check that the model is SISO and has states that belong only to one
  // discrete state group.
  const auto context = model.CreateDefaultContext();
  DRAKE_DEMAND(context.get_num_discrete_state_groups() == 1);
  DRAKE_DEMAND(context.get_num_continuous_state() == 0);
  DRAKE_DEMAND(context.get_num_abstract_state_groups() == 0);
  DRAKE_DEMAND(system.get_num_input_ports() == 1);
  DRAKE_DEMAND(system.get_num_output_ports() == 1);

  // Check that the provided  x0, u0, Q, R are consistent with the model.
  const double n = context.get_discrete_state()->get_vector().size();
  const double m = model.get_input_port().size();
  DRAKE_DEMAND(n > 0 && m > 0);
  DRAKE_DEMAND(Q.rows() == n && Q.cols() == n);
  DRAKE_DEMAND(R.rows() == m && R.cols() == m);

  this->DeclarePeriodicDiscreteUpdate(sampling_time_);

  if (context_ != nullptr) {
    // Attempt to linearize the system at that context.
    linear_model_ = LinearSystem<double>::Linearize(*model, *context);
  }
}

// TODO: Adopt a constructor hierarchy.
template <typename T>
LinearModelPredictiveController<T>::LinearModelPredictiveController(
   const systems::System<double>& model,
   const PiecewisePolynomialTrajectory& x0,
   const PiecewisePolynomialTrajectory& u0,
   const Eigen::MatrixXd& Q,
   const Eigen::MatrixXd& R,
   double sampling_time,
   double time_horizon)
    : state_input_index_(
          this->DeclareInputPort(kVectorValued, x0.cols()).get_index()),
      control_output_index_(this->DeclareVectorOutputPort(
          BasicVector<T>(u0.cols()),
          &LinearModelPredictiveController<T>::CalcControl).get_index()),
  model_(std::move(model)), x0_(x0), u0_(u0), sampling_time_(sampling_time),
  time_horizon_(time_horizon) {
  DRAKE_DEMAND(sampling_time > 0.);
  DRAKE_DEMAND(time_horizon > 0.);

  // Check that the model is SISO and has states that belong only to one
  // discrete state group.
  const auto context = model.CreateDefaultContext();
  DRAKE_DEMAND(context.get_num_discrete_state_groups() == 1);
  DRAKE_DEMAND(context.get_num_continuous_state() == 0);
  DRAKE_DEMAND(context.get_num_abstract_state_groups() == 0);
  DRAKE_DEMAND(system.get_num_input_ports() == 1);
  DRAKE_DEMAND(system.get_num_output_ports() == 1);

  // Check that the provided  x0, u0, Q, R are consistent with the model.
  const double n = context.get_discrete_state()->get_vector().size();
  const double m = model.get_input_port().size();
  DRAKE_DEMAND(n > 0 && m > 0);
  DRAKE_DEMAND(x0.rows() == n && x0.cols() == n);
  DRAKE_DEMAND(u0.rows() == 1 && u0.cols() == m);
  DRAKE_DEMAND(Q.rows() == n && Q.cols() == n);
  DRAKE_DEMAND(R.rows() == m && R.cols() == m);

  this->DeclarePeriodicDiscreteUpdate(sampling_time_);
}

template <typename T>
void LinearModelPredictiveController<T>::CalcControl(
    const Context<T>& context, BasicVector<T>* control) const {
  // Set up a DirectTranscription problem.
  DirectTranscription prog(linear_model_.get(), context_, kNumSampleTimes);

  // Apply the cost
  //
  if (context_ != nullptr) {
    const Eigen::VectorBlock<const VectorX<T>> current_state =
        this->EvalEigenVectorInput(context, input_index_state_);
    
    const VectorX<T> state_error = state - state_ref;
    const VectorX<T> input_error = input - input_ref;

    prog.AddRunningCost(state_error.transpose() * Q * state_error +
                        input_error.transpose() * R * input_error);
  } else {
    const double t = context.get_time();
    const Eigen::VectorBlock<const VectorX<T>> current_state =
        this->EvalEigenVectorInput(context, 0);
    if (x0_ != )
      const Eigen::VectorBlock<const VectorX<T>> state_ref = x0_(t);
    const Eigen::VectorBlock<const VectorX<T>> input_ref = u0_(t);
    // TODO(jadecastro) Generalize access of arbitrary scheduler
    // signal.

    // Create decision variables for an array of state profiles and input
    // profiles.

    // Create trajectories that span between the current time and a fixed
    // horizon or the end of the trajectory (whichever is smaller).

    const VectorX<T> state_error = state - state_ref;
    const VectorX<T> input_error = input - input_ref;

    // Do AddCost() at each timestep.
    prog.AddRunningCost(state_error.transpose() * Q * state_error +
                        input_error.transpose() * R * input_error);

  }

  DRAKE_DEMAND(prog.Solve(), solvers::SolutionResult::kSolutionFound);

  control->SetFromVector(prog.ReconstructInputTrajectory()(0));
}

template class LinearModelPredictiveController<double>;

}  // namespace controllers
}  // namespace systems
}  // namespace drake
