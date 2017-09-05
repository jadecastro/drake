#include "drake/systems/controllers/linear_model_predictive_controller.h"

#include <string>

#include "drake/common/eigen_types.h"
#include "drake/common/proto/call_matlab.h"
#include "drake/systems/trajectory_optimization/direct_transcription.h"

namespace drake {
namespace systems {
namespace controllers {

using solvers::VectorXDecisionVariable;
using trajectory_optimization::DirectTranscription;

template <typename T>
LinearModelPredictiveController<T>::LinearModelPredictiveController(
    std::unique_ptr<systems::System<double>> model,
    std::unique_ptr<systems::Context<double>> base_context,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, double time_period,
    double time_horizon)
    : state_input_index_(
          this->DeclareInputPort(kVectorValued, Q.cols()).get_index()),
      control_output_index_(
          this->DeclareVectorOutputPort(
                  BasicVector<T>(R.cols()),
                  &LinearModelPredictiveController<T>::CalcControl)
              .get_index()),
      num_states_(base_context_->get_discrete_state()->get_vector()->size()),
      num_inputs_(model_->get_input_port(0).size()),
      model_(std::move(model)), base_context_(std::move(base_context)),
      Q_(Q),
      R_(R),
      time_period_(time_period),
      time_horizon_(time_horizon) {
  DRAKE_DEMAND(time_period_ > 0.);
  DRAKE_DEMAND(time_horizon_ > 0.);

  // Check that the model is SISO and has discrete states belonging to a single
  // group.
  DRAKE_DEMAND(base_context_->get_num_discrete_state_groups() == 1);
  DRAKE_DEMAND(base_context_->get_continuous_state()->size() == 0);
  DRAKE_DEMAND(base_context_->get_num_abstract_state_groups() == 0);
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
  }
}

template <typename T>
LinearModelPredictiveController<T>::LinearModelPredictiveController(
    std::unique_ptr<systems::System<double>> model,
    std::unique_ptr<PiecewisePolynomialTrajectory> x0,
    std::unique_ptr<PiecewisePolynomialTrajectory> u0, const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R, double time_period, double time_horizon)
    : LinearModelPredictiveController(std::move(model), nullptr, Q, R,
                                      time_period, time_horizon) {
  scheduled_model_.reset(new TimeScheduledAffineSystem<T>(
      *model_, std::move(x0), std::move(u0), time_period_));
  // TODO(jadecastro): We always asssume we start at t = 0 under this
  // scheme. Implement a state-dependent scheduling scheme.

  // Sound?
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
    // ^^ Is this sound??

    control->SetFromVector(current_input + input_ref);
    std::cout << " inputs:\n" << current_input + input_ref << std::endl;

  } else {  // Regulate the system to the current trajectory value.
    const Eigen::VectorXd current_input = SetupAndSolveQp(current_state, t);

    const VectorX<T> input_ref = scheduled_model_->u0(t);
    control->SetFromVector(current_input + input_ref);
    std::cout << " inputs:\n" << current_input + input_ref << std::endl;
  }
}

template <typename T>
Eigen::VectorXd
LinearModelPredictiveController<T>::SetupAndSolveQp(
    const Eigen::VectorXd& current_state, double t) const {
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
  DirectTranscription prog(scheduled_model_.get(), *base_context_,
                           kNumSampleTimes);

  const auto state_error = prog.state();
  const auto input_error = prog.input();

  prog.AddRunningCost(state_error.transpose() * Q_ * state_error +
                      input_error.transpose() * R_ * input_error);

  const VectorX<T> state_ref = scheduled_model_->x0(t);
  prog.AddLinearConstraint(prog.initial_state() == current_state - state_ref);

  DRAKE_DEMAND(prog.Solve() == solvers::SolutionResult::kSolutionFound);

  // Plot time histories for two of the states of the solution
  // Note: see call_matlab.h for instructions on viewing the plot.
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

  return prog.GetInputSamples().col(0);
}

template <typename T>
Eigen::VectorXd
LinearModelPredictiveController<T>::SetupAndSolveQp(
    const Context<T>& base_context, const Eigen::VectorXd& current_state)
    const {
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

  return prog.GetInputSamples().col(0);
}

template class LinearModelPredictiveController<double>;

}  // namespace controllers
}  // namespace systems
}  // namespace drake
