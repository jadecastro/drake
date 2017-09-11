#include "drake/systems/controllers/linear_model_predictive_controller.h"

#include <string>

#include "drake/common/eigen_types.h"
#include "drake/common/proto/call_matlab.h"
#include "drake/systems/trajectory_optimization/direct_transcription.h"

namespace drake {
namespace systems {
namespace controllers {

template class EquilibriumSystem<double>;
template class EquilibriumSystem<AutoDiffXd>;

template class TimeScheduledAffineSystem<double>;
template class TimeScheduledAffineSystem<symbolic::Expression>;

using solvers::VectorXDecisionVariable;
using trajectory_optimization::DirectTranscription;

template <typename T>
LinearModelPredictiveController<T>::LinearModelPredictiveController(
    std::unique_ptr<System<double>> model,
    std::unique_ptr<Context<double>> base_context, const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R, double time_period, double time_horizon)
    : state_input_index_(
          this->DeclareInputPort(kVectorValued, Q.cols()).get_index()),
      control_output_index_(
          this->DeclareVectorOutputPort(
                  BasicVector<T>(R.cols()),
                  &LinearModelPredictiveController<T>::CalcControl)
              .get_index()),
      model_(std::move(model)),
      base_context_(std::move(base_context)),
      num_states_(
          model_->CreateDefaultContext()->get_discrete_state(0)->size()),
      num_inputs_(model_->get_input_port(0).size()),
      Q_(Q),
      R_(R),
      time_period_(time_period),
      time_horizon_(time_horizon) {
  DRAKE_DEMAND(time_period_ > 0.);
  DRAKE_DEMAND(time_horizon_ > 0.);

  // Check that the model is SISO and has discrete states belonging to a single
  // group.
  const auto model_context = model_->CreateDefaultContext();
  DRAKE_DEMAND(model_context->get_num_discrete_state_groups() == 1);
  DRAKE_DEMAND(model_context->get_continuous_state()->size() == 0);
  DRAKE_DEMAND(model_context->get_num_abstract_state_groups() == 0);
  DRAKE_DEMAND(model_->get_num_input_ports() == 1);
  DRAKE_DEMAND(model_->get_num_output_ports() == 1);

  // Check that the provided  x0, u0, Q, R are consistent with the model.
  DRAKE_DEMAND(num_states_ > 0 && num_inputs_ > 0);
  DRAKE_DEMAND(Q.rows() == num_states_ && Q.cols() == num_states_);
  DRAKE_DEMAND(R.rows() == num_inputs_ && R.cols() == num_inputs_);

  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
  if (R_cholesky.info() != Eigen::Success) {
    throw std::runtime_error("R must be positive definite");
  }

  this->DeclarePeriodicDiscreteUpdate(time_period_);

  if (base_context_ != nullptr) {
    linear_model_ = Linearize(*model_, *base_context_);
    const auto symbolic_linear_model = linear_model_->ToSymbolic();
    DRAKE_DEMAND(symbolic_linear_model != nullptr);
  }
}

template <typename T>
LinearModelPredictiveController<T>::LinearModelPredictiveController(
    std::unique_ptr<System<double>> model,
    std::unique_ptr<PiecewisePolynomialTrajectory> x0,
    std::unique_ptr<PiecewisePolynomialTrajectory> u0, const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R, double time_period, double time_horizon)
    : LinearModelPredictiveController(std::move(model), nullptr, Q, R,
                                      time_period, time_horizon) {
  const auto eq_model =
      std::make_unique<EquilibriumSystem<double>>(std::move(model_),
                                                  time_period_);
  scheduled_model_.reset(new TimeScheduledAffineSystem<T>(
      *eq_model, std::move(x0), std::move(u0), time_period_));
  const auto symbolic_scheduled_model = scheduled_model_->ToAutoDiffXd();
  DRAKE_DEMAND(symbolic_scheduled_model != nullptr);
  // TODO(jadecastro): We always asssume we start at t = 0 under this
  // scheme. Implement a state-dependent scheduling scheme.

  // The following is unneeded in DirTran, since it punts on context.
  /*
  auto state_vector =
      base_context_->get_mutable_discrete_state()->get_mutable_vector();
  auto input_vector = std::make_unique<BasicVector<T>>(u0->cols());
  const auto state_ref = x0->value(0);
  state_vector->SetFromVector(state_ref);
  const auto input_ref = u0->value(0);
  input_vector->SetFromVector(input_ref);
  base_context_->SetInputPortValue(
      0,
      std::make_unique<FreestandingInputPortValue>(
          std::move(input_vector)));
  */

  
}

template <typename T>
void LinearModelPredictiveController<T>::CalcControl(
    const Context<T>& context, BasicVector<T>* control) const {
  const double t = context.get_time();
  const Eigen::VectorBlock<const VectorX<T>> current_state =
      this->EvalEigenVectorInput(context, state_input_index_);

  std::cout << " time: " << context.get_time() << std::endl;

  if (base_context_ != nullptr) {  // Regulate to a fixed equilibrium.
    const Eigen::VectorXd current_input =
        SetupAndSolveQp(*base_context_, current_state);

    InputPortDescriptor<double> descriptor(nullptr, 0, kVectorValued, 0);
    const VectorX<T> input_ref =
        base_context_->EvalVectorInput(nullptr, descriptor)->CopyToVector();

    control->SetFromVector(current_input + input_ref);
    std::cout << " inputs:\n" << current_input + input_ref << std::endl;

  } else {  // Regulate the system to the current trajectory value.
    std::cout << " Time-varying case " << std::endl;
    const VectorX<T> state_ref = scheduled_model_->x0(t);
    std::cout << " state ref:\n" << state_ref << std::endl;
    const Eigen::VectorXd current_input =
        SetupAndSolveQp(current_state, state_ref, t);

    const VectorX<T> input_ref = scheduled_model_->u0(t);
    std::cout << " input ref:\n" << input_ref << std::endl;
    control->SetFromVector(current_input + input_ref);
    std::cout << " inputs:\n" << current_input + input_ref << std::endl;
  }
}

template <typename T>
VectorX<T> LinearModelPredictiveController<T>::SetupAndSolveQp(
    const VectorX<T>& current_state, const VectorX<T>& state_ref, const T& time)
    const {
  DRAKE_DEMAND(scheduled_model_ != nullptr);

  const int kNumSampleTimes = (int)(time_horizon_ / time_period_ + 0.5);
  std::cout << " Num time samples: " << kNumSampleTimes << std::endl;

  /*
  MultipleShooting prog(
      num_inputs_, num_states_, kNumSampleTimes, time_period_);

  // Add dynamic constraint for the time-varying affine system.
  const auto symbolic_model = linear_model->ToSymbolic();
  DRAKE_DEMAND(symbolic_model != nullptr);
  for (int i = 0; i < kNumSampleTimes - 1; i++) {
    VectorX<symbolic::Expression> update = symbolic_model->discrete_update(0);
    symbolic::Substitution sub;
    sub.emplace(symbolic_model->time(), i * time_period_);
    for (int j = 0; j < num_states_; j++) {
      sub.emplace(symbolic_model->discrete_state(0)[j], prog.state(i)[j]);
    }
    for (int j = 0; j < num_inputs_; j++) {
      sub.emplace(symbolic_model->input(0)[j], prog.input(i)[j]);
    }
    for (int j = 0; j < num_states_; j++) {
      update(j) = update(j).Substitute(sub);
    }
    prog.AddLinearConstraint(prog.state(i + 1) == update);
    prog.AddRunningCost();
  }
  */

  // Needs to be reworked to include the base trajectory states/inputs at each
  // time step.
  const auto model_context = scheduled_model_->CreateDefaultContext();
  model_context->set_time(time);
  DirectTranscription prog(scheduled_model_.get(), *model_context,
                           kNumSampleTimes - time / time_period_);
  // TODO Desire a tighter coupling between model_context's time and number of
  // samples.

  const auto state_error = prog.state();
  const auto input_error = prog.input();

  prog.AddRunningCost(state_error.transpose() * Q_ * state_error +
                      1e2 * input_error.transpose() * R_ * input_error);

  prog.AddLinearConstraint(prog.initial_state() == current_state - state_ref);

  // Don't demand optimality.
  const auto result = prog.Solve();
  std::cout << " Solution Result: " << result << std::endl;

  // Plot time histories for two of the states of the solution
  // Note: see call_matlab.h for instructions on viewing the plot.
  Eigen::VectorXd times = prog.GetSampleTimes();
  Eigen::MatrixXd inputs = prog.GetInputSamples();
  Eigen::MatrixXd states = prog.GetStateSamples();

  std::cout << " state error:\n" << states.col(0) << std::endl;
  std::cout << " input error:\n" << inputs.col(0) << std::endl;
  std::cout << " states:\n" << states.col(0) + state_ref << std::endl;


  for (int i{0}; i < times.size(); ++i) {
    states(0,i) = states(0,i) + scheduled_model_->x0(times(i) + time)(0);
    states(1,i) = states(1,i) + scheduled_model_->x0(times(i) + time)(1);
    inputs(0,i) = inputs(0,i) + scheduled_model_->u0(times(i) + time)(0);
  }

  common::CallMatlab("figure");
  common::CallMatlab("plot", times, states.row(0));
  common::CallMatlab("xlabel", "time");
  common::CallMatlab("ylabel", "x0");
  //common::CallMatlab("figure");
  //common::CallMatlab("plot", times, states.row(1));
  //common::CallMatlab("xlabel", "time");
  //common::CallMatlab("ylabel", "x1");
  common::CallMatlab("figure");
  common::CallMatlab("plot", times, inputs.row(0));
  common::CallMatlab("xlabel", "time");
  common::CallMatlab("ylabel", "u0");

  return prog.GetInputSamples().col(0);
}

template <typename T>
VectorX<T> LinearModelPredictiveController<T>::SetupAndSolveQp(
    const Context<T>& base_context, const VectorX<T>& current_state) const {
  DRAKE_DEMAND(linear_model_ != nullptr);

  const int kNumSampleTimes = (int)(time_horizon_ / time_period_ + 0.5);
  std::cout << " Num time samples: " << kNumSampleTimes << std::endl;

  DirectTranscription prog(linear_model_.get(), *base_context_,
                           kNumSampleTimes);

  const auto state_error = prog.state();
  const auto input_error = prog.input();

  prog.AddRunningCost(state_error.transpose() * Q_ * state_error +
                      input_error.transpose() * R_ * input_error);

  const VectorX<T> state_ref =
      base_context.get_discrete_state()->get_vector()->CopyToVector();
  prog.AddLinearConstraint(prog.initial_state() == current_state - state_ref);

  DRAKE_DEMAND(prog.Solve() == solvers::SolutionResult::kSolutionFound);

  // Plot time histories for two of the states of the solution
  // Note: see call_matlab.h for instructions on viewing the plot.
  /*
  Eigen::VectorXd times = prog.GetSampleTimes();
  Eigen::MatrixXd inputs = prog.GetInputSamples();
  Eigen::MatrixXd states = prog.GetStateSamples();

  std::cout << " state error:\n" << states.col(0) << std::endl;
  std::cout << " input error:\n" << inputs.col(0) << std::endl;
  std::cout << " states:\n" << states.col(0) + state_ref << std::endl;

  common::CallMatlab("figure");
  common::CallMatlab("plot", times, states.row(0));
  common::CallMatlab("xlabel", "time");
  common::CallMatlab("ylabel", "x0");
  common::CallMatlab("figure");
  common::CallMatlab("plot", times, states.row(1));
  common::CallMatlab("xlabel", "time");
  common::CallMatlab("ylabel", "x1");
  */

  return prog.GetInputSamples().col(0);
}

template class LinearModelPredictiveController<double>;

}  // namespace controllers
}  // namespace systems
}  // namespace drake
