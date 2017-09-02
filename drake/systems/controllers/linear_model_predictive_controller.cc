#include "drake/systems/controllers/linear_model_predictive_controller.h"

#include <string>

#include "drake/common/eigen_types.h"
#include "drake/common/proto/call_matlab.h"
#include "drake/systems/trajectory_optimization/direct_transcription.h"

#include "drake/examples/acrobot/acrobot_plant.h"  // Testing...

namespace drake {
namespace systems {
namespace controllers {

using solvers::VectorXDecisionVariable;
using trajectory_optimization::DirectTranscription;

template <typename T>
LinearModelPredictiveController<T>::LinearModelPredictiveController(
    std::unique_ptr<systems::System<double>> model,
    std::unique_ptr<systems::Context<double>> base_context,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    double time_period,
    double time_horizon)
    : state_input_index_(
          this->DeclareInputPort(kVectorValued, Q.cols()).get_index()),
      control_output_index_(this->DeclareVectorOutputPort(
          BasicVector<T>(R.cols()),
          &LinearModelPredictiveController<T>::CalcControl).get_index()),
      model_(std::move(model)), base_context_(std::move(base_context)),
      Q_(Q), R_(R), time_period_(time_period), time_horizon_(time_horizon) {
  DRAKE_DEMAND(time_period_ > 0.);
  DRAKE_DEMAND(time_horizon_ > 0.);

  // Check that the model is SISO and has states that belong only to one
  // discrete state group.
  DRAKE_DEMAND(base_context_->get_num_discrete_state_groups() == 1);
  DRAKE_DEMAND(base_context_->get_continuous_state()->size() == 0);
  DRAKE_DEMAND(base_context_->get_num_abstract_state_groups() == 0);
  DRAKE_DEMAND(model_->get_num_input_ports() == 1);
  DRAKE_DEMAND(model_->get_num_output_ports() == 1);

  // Check that the provided  x0, u0, Q, R are consistent with the model.
  const double n = base_context_->get_discrete_state()->get_vector()->size();
  const double m = model_->get_input_port(0).size();
  DRAKE_DEMAND(n > 0 && m > 0);
  DRAKE_DEMAND(Q.rows() == n && Q.cols() == n);
  DRAKE_DEMAND(R.rows() == m && R.cols() == m);

  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
  if (R_cholesky.info() != Eigen::Success) {
    throw std::runtime_error("R must be positive definite");
  }

  this->DeclarePeriodicDiscreteUpdate(time_period_);

  if (base_context_ != nullptr) {
    // Attempt to linearize the system at that context.
    linear_model_ = Linearize(*model_, *base_context_);
  }
}

// TODO: Adopt a constructor hierarchy.
template <typename T>
LinearModelPredictiveController<T>::LinearModelPredictiveController(
   std::unique_ptr<systems::System<double>> model,
   std::unique_ptr<PiecewisePolynomialTrajectory> x0,
   std::unique_ptr<PiecewisePolynomialTrajectory> u0,
   const Eigen::MatrixXd& Q,
   const Eigen::MatrixXd& R,
   double time_period,
   double time_horizon)
    : state_input_index_(
          this->DeclareInputPort(kVectorValued, x0->cols()).get_index()),
      control_output_index_(this->DeclareVectorOutputPort(
          BasicVector<T>(u0->cols()),
          &LinearModelPredictiveController<T>::CalcControl).get_index()),
      model_(std::move(model)), base_context_(nullptr),
      x0_(std::move(x0)), u0_(std::move(u0)),
      Q_(Q), R_(R), time_period_(time_period), time_horizon_(time_horizon) {
  DRAKE_DEMAND(time_period_ > 0.);
  DRAKE_DEMAND(time_horizon_ > 0.);

  // Check that the model is SISO and has states that belong only to one
  // discrete state group.
  const auto context = model_->CreateDefaultContext();
  DRAKE_DEMAND(context->get_num_discrete_state_groups() == 1);
  DRAKE_DEMAND(context->get_continuous_state()->size() == 0);
  DRAKE_DEMAND(context->get_num_abstract_state_groups() == 0);
  DRAKE_DEMAND(model_->get_num_input_ports() == 1);
  DRAKE_DEMAND(model_->get_num_output_ports() == 1);

  // Check that the provided  x0, u0, Q, R are consistent with the model.
  const double n = context->get_discrete_state()->get_vector()->size();
  const double m = model_->get_input_port(0).size();
  DRAKE_DEMAND(n > 0 && m > 0);
  DRAKE_DEMAND(x0_->rows() == n && x0_->cols() == n);
  DRAKE_DEMAND(u0_->rows() == 1 && u0_->cols() == m);
  DRAKE_DEMAND(Q_.rows() == n && Q_.cols() == n);
  DRAKE_DEMAND(R_.rows() == m && R_.cols() == m);

  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
  DRAKE_THROW_UNLESS(R_cholesky.info() == Eigen::Success);

  this->DeclarePeriodicDiscreteUpdate(time_period_);
}

template <typename T>
void LinearModelPredictiveController<T>::CalcControl(
    const Context<T>& context, BasicVector<T>* control) const {
  const double t = context.get_time();
  const Eigen::VectorBlock<const VectorX<T>> current_state =
      this->EvalEigenVectorInput(context, state_input_index_);

  const int kNumSampleTimes = (int)(time_horizon_ / time_period_ + 0.5);
  std::cout << " Num time samples: " << kNumSampleTimes << std::endl;

  if (base_context_ == nullptr) {
    // Linearize the system about the current trajectory value.
    auto base_context = model_->CreateDefaultContext();
    base_context->get_mutable_discrete_state()->get_mutable_vector()->
        SetFromVector(x0_->value(t));
    auto input_vector = std::make_unique<BasicVector<T>>(u0_->cols());
    input_vector->SetFromVector(u0_->value(t));
    base_context->SetInputPortValue(
        0,
        std::make_unique<FreestandingInputPortValue>(std::move(input_vector)));

    // Also perhaps need to set parameters to obtain a linearization about a
    // non-equilibrium condition.

    auto linear_model = Linearize(*model_, *base_context);

    // Set up a DirectTranscription problem.
    DirectTranscription prog(linear_model.get(), *base_context,
                             kNumSampleTimes);

    prog.AddLinearConstraint(prog.initial_state() == current_state);

    /*
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

    // Add a quadratic cost at each time step.
    prog.AddCost(state_error.transpose() * Q * state_error +
                 input_error.transpose() * R * input_error);
    */

    DRAKE_DEMAND(prog.Solve() == solvers::SolutionResult::kSolutionFound);

    PiecewisePolynomialTrajectory input_traj =
        prog.ReconstructInputTrajectory();
    control->SetFromVector(input_traj.value(0));

  } else {
    // Set up a DirectTranscription problem.
    DirectTranscription prog(linear_model_.get(), *base_context_,
                             kNumSampleTimes);

    const VectorX<T> state_ref =
        base_context_->get_discrete_state()->get_vector()->CopyToVector();
    InputPortDescriptor<double> descriptor(nullptr, 0, kVectorValued, 0);
    const VectorX<T> input_ref =
        base_context_->EvalVectorInput(nullptr, descriptor)->CopyToVector();
    // ^^ Sound?

    // Testing to see how continuous linearization compares with the discrete.
    /*
    auto continuous_acrobot =
        std::make_unique<examples::acrobot::AcrobotPlant<double>>();
    auto continuous_base_context = continuous_acrobot->CreateDefaultContext();
    continuous_base_context->FixInputPort(0, input_ref);
    continuous_base_context->get_mutable_continuous_state_vector()->
        SetFromVector(base_context_->get_discrete_state(0)->get_value());
    auto continuous_linear_model =
        Linearize(*continuous_acrobot, *continuous_base_context);
    std::cout << " Ac " << continuous_linear_model->A() << std::endl;
    std::cout << " Ad " << linear_model_->A() << std::endl;
    std::cout << " Bc " << continuous_linear_model->B() << std::endl;
    std::cout << " Bd " << linear_model_->B() << std::endl;
    std::cout << " Cc " << continuous_linear_model->C() << std::endl;
    std::cout << " Cd " << linear_model_->C() << std::endl;
    std::cout << " Dc " << continuous_linear_model->D() << std::endl;
    std::cout << " Dd " << linear_model_->D() << std::endl;
    */

    //std::cout << " state_ref:\n" << state_ref << std::endl;
    //std::cout << " input_ref:\n" << input_ref << std::endl;
    /*
    const Eigen::Ref<const MatrixX<symbolic::Expression>> state_error =
        prog.state().cast<symbolic::Expression>() - state_ref;
    const Eigen::Ref<const MatrixX<symbolic::Expression>> input_error =
        prog.input().cast<symbolic::Expression>() - input_ref;
    */

    const auto state_error = prog.state() - state_ref;
    const auto input_error = prog.input() - input_ref;

    prog.AddLinearConstraint(prog.initial_state() == current_state);

    // Add some bounds to the states and inputs.
    /*
    const int num_states = prog.state().size();
    prog.AddConstraintToAllKnotPoints(prog.state() <=
                                      50. * Eigen::VectorXd::Ones(num_states));
    prog.AddConstraintToAllKnotPoints(prog.state() >=
                                      -50. * Eigen::VectorXd::Ones(num_states));
    const int num_inputs = prog.input().size();
    prog.AddConstraintToAllKnotPoints(prog.input() <=
                                      50. * Eigen::VectorXd::Ones(num_inputs));
    prog.AddConstraintToAllKnotPoints(prog.input() >=
                                      -50. * Eigen::VectorXd::Ones(num_inputs));
    */

    // Initial trajectory is a straight line, assuming control is zero
    // currently.
    prog.SetInitialTrajectory(
        PiecewisePolynomial<double>::FirstOrderHold(
            {0, time_horizon_}, {0. * input_ref, input_ref}),
        PiecewisePolynomial<double>::FirstOrderHold(
            {0, time_horizon_}, {current_state, state_ref}));

    prog.AddRunningCost(state_error.transpose() * Q_ * state_error +
                        input_error.transpose() * R_ * input_error);
    // prog.AddRunningCost(prog.state().transpose() * prog.state());
    // prog.AddQuadraticErrorCost(Q_, state_ref, prog.state());

    // Runnning cost does not seem to work... so try a final cost?
    //prog.AddFinalCost(state_error.transpose() * Q_ * state_error +
    //                  input_error.transpose() * R_ * input_error);

    std::cout << " Cost: " << state_error.transpose() * Q_ * state_error +
        input_error.transpose() * R_ * input_error << std::endl;

    //DRAKE_DEMAND(prog.Solve() == solvers::SolutionResult::kSolutionFound);
    const int result = prog.Solve();
    std::cout << " Sol RESULT: " << result << std::endl;

    // Plot time histories for two of the states of the solution
    // Note: see call_matlab.h for instructions on viewing the plot.
    Eigen::VectorXd times = prog.GetSampleTimes();
    Eigen::MatrixXd inputs = prog.GetInputSamples();
    Eigen::MatrixXd states = prog.GetStateSamples();
    std::cout << " time: " << context.get_time() << std::endl;
    std::cout << " states:\n" << states.col(0) << std::endl;
    std::cout << " inputs:\n" << inputs.col(0) << std::endl;
    std::cout << " state error:\n" << states.col(0) - state_ref << std::endl;
    std::cout << " input error:\n" << inputs.col(0) - input_ref << std::endl;
    common::CallMatlab("figure");
    common::CallMatlab("plot", times, states.row(0));
    common::CallMatlab("xlabel", "time");
    common::CallMatlab("ylabel", "x0");
    common::CallMatlab("figure");
    common::CallMatlab("plot", times, states.row(1));
    common::CallMatlab("xlabel", "time");
    common::CallMatlab("ylabel", "x1");

    PiecewisePolynomialTrajectory input_traj =
        prog.ReconstructInputTrajectory();
    std::cout << " applied inputs:\n" << input_traj.value(0) << std::endl;
    std::cout << " \n" << std::endl;
    control->SetFromVector(input_traj.value(0));
  }
}

template class LinearModelPredictiveController<double>;

}  // namespace controllers
}  // namespace systems
}  // namespace drake
