#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/extract_double.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {

/// A system wrapper that adjusts derivatives or discrete updates of a System
/// such that it reaches pseudo-equilibrium at a state that is not a fixed point
/// of that System.  In particular this class allows the insertion of a
/// time-varying base_vector such that:
///
///   @f[ \dot{x(t)} = f(t, x(t), u(t)) + base_vector(t) @f]
///
/// if the system is continuous time, or
///
///   @f[ x(k+1) = f(k, x(k), u(k)) + base_vector(k) @f]
///
/// if the system is discrete time.  The base_vector is an initially zero
/// parameter vector that either modifies the derivatives (for CT systems) or
/// the updates (for DT systems).
///
/// ***** TODO Update this blurb and rename the class to remove the notion of
/// equilibrium:
///
/// When simulated, this system behaves just as the original underlying System.
/// However, each time base_vector is updated, the system will behave as if the
/// system is in equilibrium at base_vector.  When base_vector is obtained from
/// a call to S<T>::CalcTimeDerivatives() at a particular context, the
/// EquilibriumSystem may be used to linearize about that (non-equilibrium)
/// context.
template <typename T>
class EquilibriumSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EquilibriumSystem)

  // TODO(jadecastro) Figure out how to construct based on doubles, not T.
  EquilibriumSystem(std::unique_ptr<System<T>> subsystem,
                    const double time_period)
      : subsystem_(std::move(subsystem)),
        subcontext_(subsystem_->CreateDefaultContext()),
        time_period_(time_period) {
    const int num_xc = subcontext_->get_continuous_state_vector().size();
    DRAKE_DEMAND(subcontext_->get_num_discrete_state_groups() == 1);
    const int num_xd = subcontext_->get_discrete_state(0)->size();
    DRAKE_DEMAND(!num_xc || !num_xd);

    // System is assumed SISO.
    DRAKE_DEMAND(subsystem_->get_num_input_ports() == 1);
    DRAKE_DEMAND(subsystem_->get_num_output_ports() == 1);

    const int input_size = subsystem_->get_input_port(0).size();
    this->DeclareVectorInputPort(BasicVector<T>(input_size));
    BasicVector<T> model_vector(subsystem_->get_output_port(0).size());
    this->DeclareVectorOutputPort(model_vector, &EquilibriumSystem::CalcState);

    if (time_period_ != 0.) {
      this->DeclarePeriodicDiscreteUpdate(time_period_);
    }

    // We are appending an additional parameter to the existing vector.
    base_vector_index_ = subcontext_->num_numeric_parameters();
  }

  BasicVector<T>* get_mutable_base_vector(Context<T>* context) const {
    return this->GetMutableNumericParameter(context, base_vector_index_);
  }

 protected:
  void CalcState(const Context<T>& context, BasicVector<T>* output) const {
    if (time_period_ == 0.) {
      const VectorBase<T>& context_state =
          context.get_continuous_state_vector();
      const BasicVector<T>* const state =
          dynamic_cast<const BasicVector<T>*>(&context_state);
      DRAKE_DEMAND(state != nullptr);
      output->set_value(state->get_value());
    } else {
      output->set_value(context.get_discrete_state(0)->get_value());
    }
  }

 private:
  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {
    SetTimeStateAndParametersFromT(context);
    subsystem_->CalcTimeDerivatives(*subcontext_, derivatives);

    const BasicVector<T>& base_derivatives =
        this->template GetNumericParameter<BasicVector>(context,
                                                        base_vector_index_);
    derivatives->SetFromVector(derivatives->CopyToVector() -
                               base_derivatives.get_value());
  }

  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>&,
      DiscreteValues<T>* updates) const override {
    SetTimeStateAndParametersFromT(context);
    subsystem_->CalcDiscreteVariableUpdates(*subcontext_, updates);

    const BasicVector<T>& base_states =
        this->template GetNumericParameter<BasicVector>(context,
                                                        base_vector_index_);
    updates->get_mutable_vector()->SetFromVector(
        updates->get_vector()->get_value() - base_states.get_value());
  }

  std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const override {
    auto vector = std::make_unique<BasicVector<T>>(
        subcontext_->get_continuous_state_vector().CopyToVector());
    return std::make_unique<ContinuousState<T>>(std::move(vector));
  }

  std::unique_ptr<DiscreteValues<T>> AllocateDiscreteState() const override {
    auto vector = std::make_unique<BasicVector<T>>(
        subcontext_->get_discrete_state(0)->CopyToVector());
    return std::make_unique<DiscreteValues<T>>(std::move(vector));
  }

  std::unique_ptr<Parameters<T>> AllocateParameters() const override {
    std::vector<std::unique_ptr<BasicVector<T>>> params;
    params.reserve(subcontext_->num_numeric_parameters() + 1);
    for (int i = 0; i < subcontext_->num_numeric_parameters(); ++i) {
      auto param = subcontext_->get_numeric_parameter(i)->Clone();
      params.emplace_back(std::move(param));
    }
    // Append an additional set of parameters for the base derivatives/updates.
    params.emplace_back(std::make_unique<BasicVector<T>>(
        std::max(subcontext_->get_continuous_state_vector().size(),
                 subcontext_->get_discrete_state(0)->size())));

    return std::make_unique<Parameters<T>>(std::move(params));
  }

  void SetTimeStateAndParametersFromT(const Context<T>& context) const {
    subcontext_->set_time(context.get_time());
    subcontext_->get_mutable_state()->CopyFrom(context.get_state());
    for (int i{0}; i < subcontext_->num_numeric_parameters(); ++i) {
      subcontext_->get_mutable_numeric_parameter(i)->SetFromVector(
          context.get_numeric_parameter(i)->get_value());
    }
    subcontext_->FixInputPort(0,
                              this->EvalVectorInput(context, 0)->get_value());
  }

  // Initialize the appended parameters to zero values.
  void SetDefaultParameters(const LeafContext<T>&,
                            Parameters<T>* parameters) const override {
    BasicVector<T>* p =
        parameters->get_mutable_numeric_parameter(base_vector_index_);
    p->SetFromVector(VectorX<T>::Constant(p->size(), 0.));
  }

  EquilibriumSystem<AutoDiffXd>* DoToAutoDiffXd() const final {
    return new EquilibriumSystem<AutoDiffXd>(subsystem_->ToAutoDiffXd(),
                                             time_period_);
  }

  const std::unique_ptr<System<T>> subsystem_;
  const std::unique_ptr<Context<T>> subcontext_;
  const double time_period_{0.};

  int base_vector_index_{-1};
};

namespace {

// A helper function for cloning a PiecewisePolynomialTrajectory.
static std::unique_ptr<PiecewisePolynomialTrajectory> ClonePpt(
    const PiecewisePolynomialTrajectory& ppt) {
  const PiecewisePolynomial<double>& pp = ppt.get_piecewise_polynomial();
  return std::make_unique<PiecewisePolynomialTrajectory>(pp);
}

}  // namespace

/// A system containing a time-varying system of the form:
///
/// @f[ xdot(t) = A(t)x(t) + B(t)u(t) + f0(t)  @f]
/// @f[ y(t) = A(t)x(t) + B(t)u(t) + y0(t)  @f]
///
/// where the parameters A, B, C, D, f0, and y0 are taken from a nominal
/// state/input trajectory that is supplied to the constructor.  In particular,
/// f0 = x0(k) - x0(k-1);
template <typename T>
class TimeScheduledAffineSystem : public TimeVaryingAffineSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimeScheduledAffineSystem)

  TimeScheduledAffineSystem(const EquilibriumSystem<double>& model,
                            std::unique_ptr<PiecewisePolynomialTrajectory> x0,
                            std::unique_ptr<PiecewisePolynomialTrajectory> u0,
                            double time_period)
      : TimeVaryingAffineSystem<T>(x0->rows(), u0->rows(),
                                   model.get_output_port(0).size(),
                                   time_period) {
    // Check x0, u0 against the model.

    x0_ = std::move(x0);
    u0_ = std::move(u0);

    const std::vector<double> times =
        x0_->get_piecewise_polynomial().getSegmentTimes();
    std::vector<MatrixX<double>> A_vector{};
    std::vector<MatrixX<double>> B_vector{};
    std::vector<MatrixX<double>> C_vector{};
    std::vector<MatrixX<double>> D_vector{};
    for (auto t : times) {
      std::unique_ptr<Context<double>> base_context =
          model.CreateDefaultContext();
      auto state_vector =
          base_context->get_mutable_discrete_state()->get_mutable_vector();

      const VectorX<double> state_ref = x0_->value(t);
      state_vector->set_value(state_ref);

      const VectorX<double> input_ref = u0_->value(t);
      auto input_vector = std::make_unique<BasicVector<double>>(u0_->cols());
      input_vector->set_value(input_ref);
      base_context->SetInputPortValue(
          0, std::make_unique<FreestandingInputPortValue>(
                 std::move(input_vector)));

      // Set parameters to obtain a linearization about a non-equilibrium
      // condition.
      SetBaseDerivativesAt(model, base_context.get());

      const std::unique_ptr<LinearSystem<double>> linear_model =
          Linearize(model, *base_context);

      A_vector.emplace_back(linear_model->A());
      B_vector.emplace_back(linear_model->B());
      C_vector.emplace_back(linear_model->C());
      D_vector.emplace_back(linear_model->D());
    }

    // Create matrices for a piecewise-linear system.
    A_.reset(new PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::FirstOrderHold(times, A_vector)));
    B_.reset(new PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::FirstOrderHold(times, B_vector)));
    C_.reset(new PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::FirstOrderHold(times, C_vector)));
    D_.reset(new PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::FirstOrderHold(times, D_vector)));
  }

  TimeScheduledAffineSystem(std::unique_ptr<PiecewisePolynomialTrajectory> A,
                            std::unique_ptr<PiecewisePolynomialTrajectory> B,
                            std::unique_ptr<PiecewisePolynomialTrajectory> C,
                            std::unique_ptr<PiecewisePolynomialTrajectory> D,
                            std::unique_ptr<PiecewisePolynomialTrajectory> x0,
                            std::unique_ptr<PiecewisePolynomialTrajectory> u0,
                            double time_period)
      : TimeVaryingAffineSystem<T>(A->cols(), B->cols(), C->rows(),
                                   time_period) {
    A_ = std::move(A);
    B_ = std::move(B);
    C_ = std::move(C);
    D_ = std::move(D);
    x0_ = std::move(x0);
    u0_ = std::move(u0);
  }

  ~TimeScheduledAffineSystem() override {}

  VectorX<T> x0(const T& t) const {
    return x0_->value(ExtractDoubleOrThrow(t));
  }
  VectorX<T> u0(const T& t) const {
    return u0_->value(ExtractDoubleOrThrow(t));
  }

  MatrixX<T> A(const T& t) const override {
    return A_->value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> B(const T& t) const override {
    return B_->value(ExtractDoubleOrThrow(t));
  }
  VectorX<T> f0(const T& t) const override {
    using std::max;
    //const T tprev = max(0., t - this->time_period());
    return VectorX<T>::Zero(C(t).rows()); //x0(t) - x0(tprev);
  }
  MatrixX<T> C(const T& t) const override {
    return C_->value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> D(const T& t) const override {
    return D_->value(ExtractDoubleOrThrow(t));
  }
  VectorX<T> y0(const T& t) const override {
    return VectorX<T>::Zero(C(t).rows()); //C(t) * x0(t) + D(t) * u0(t);
  }

 private:
  // System<T> overrides.
  TimeScheduledAffineSystem<AutoDiffXd>* DoToAutoDiffXd() const final {
    return new TimeScheduledAffineSystem<AutoDiffXd>(
        ClonePpt(*A_), ClonePpt(*B_), ClonePpt(*C_), ClonePpt(*D_),
        ClonePpt(*x0_), ClonePpt(*u0_), this->time_period());
  }

  /*
  TimeScheduledAffineSystem<symbolic::Expression>* DoToSymbolic() const final {
    return new TimeScheduledAffineSystem<symbolic::Expression>(
        ClonePpt(*A_), ClonePpt(*B_), ClonePpt(*C_), ClonePpt(*D_),
        ClonePpt(*x0_), ClonePpt(*u0_), this->time_period());
  }
  */

  void SetBaseDerivativesAt(const EquilibriumSystem<double>& model,
                            Context<double>* base_context) const {
    // Set the time derivatives at the base-point in order to linearize the
    // system about non-equilibrium.  Note that these derivatives are brought in
    // as parameters.
    BasicVector<double>* base_vector =
        model.get_mutable_base_vector(base_context);
    if (this->time_period() == 0.) {
      auto derivatives = model.AllocateTimeDerivatives();

      model.CalcTimeDerivatives(*base_context, derivatives.get());

      base_vector->SetFrom(derivatives->get_vector());
    } else {
      auto updates = model.AllocateDiscreteVariables();

      model.CalcDiscreteVariableUpdates(*base_context, updates.get());

      Eigen::VectorXd current_state =
          base_context->get_discrete_state(0)->get_value();
      base_vector->SetFromVector(updates->get_vector()->get_value() -
                                 current_state);
    }
  }

  // Nominal (reference) trajectories.
  std::unique_ptr<PiecewisePolynomialTrajectory> A_;
  std::unique_ptr<PiecewisePolynomialTrajectory> B_;
  std::unique_ptr<PiecewisePolynomialTrajectory> C_;
  std::unique_ptr<PiecewisePolynomialTrajectory> D_;

  std::unique_ptr<PiecewisePolynomialTrajectory> x0_;
  std::unique_ptr<PiecewisePolynomialTrajectory> u0_;
};

// TODO(jadecastro) Throw if output doesn't match states.

/// Implements a basic Model Predictive Controller based on a discrete plant
/// model.  By restricting to affine systems and affine constraints, the
/// approach is appeals to a QP solver.  Note that this implementation is basic
/// in the sense that the QP is not solved in a sequential manner, but rather
/// solved in whole at every time step.
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
  /// system and regulate the system.  This context *must* correpond to a fixed
  /// point of the plant model at the provided input.
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
  LinearModelPredictiveController(std::unique_ptr<System<double>> model,
                                  std::unique_ptr<Context<double>> base_context,
                                  const Eigen::MatrixXd& Q,
                                  const Eigen::MatrixXd& R, double time_period,
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
      std::unique_ptr<System<double>> model,
      std::unique_ptr<PiecewisePolynomialTrajectory> x0,
      std::unique_ptr<PiecewisePolynomialTrajectory> u0,
      const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, double time_period,
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
  VectorX<T> SetupAndSolveQp(const VectorX<T>& current_state,
                             const VectorX<T>& state_ref, const T& time) const;

  VectorX<T> SetupAndSolveQp(const Context<T>& base_context,
                             const VectorX<T>& current_state) const;

  const int state_input_index_{-1};
  const int control_output_index_{-1};

  // TODO(jadecastro) Figure out how to make this const and have it still work.
  std::unique_ptr<System<double>> model_;
  // The base context is the reference point to regulate.
  const std::unique_ptr<Context<double>> base_context_;

  const int num_states_{};
  const int num_inputs_{};

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
