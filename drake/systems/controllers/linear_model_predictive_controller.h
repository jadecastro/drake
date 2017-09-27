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
class EquilibriumSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EquilibriumSystem)

  // Constructor for EquilibriumSystem.
  // @param subsystem The system model (must outlive this class).
  // @param time_period The time period for this model (@p time_period = 0 ->
  // continuous-time).
  EquilibriumSystem(const System<double>& subsystem, const double time_period)
      : LeafSystem<T>(SystemTypeTag<systems::controllers::EquilibriumSystem>{}),
        subsystem_(subsystem),
        subcontext_(subsystem_.CreateDefaultContext()),
        diff_subsystem_(subsystem_.ToAutoDiffXd()),
        diff_subcontext_(diff_subsystem_->CreateDefaultContext()),
        time_period_(time_period) {
    const int num_xc = subcontext_->get_continuous_state_vector().size();
    DRAKE_DEMAND(subcontext_->get_num_discrete_state_groups() == 1);
    const int num_xd = subcontext_->get_discrete_state(0)->size();
    DRAKE_DEMAND(!num_xc || !num_xd);

    // System is assumed SISO.
    DRAKE_DEMAND(subsystem_.get_num_input_ports() == 1);
    DRAKE_DEMAND(subsystem_.get_num_output_ports() == 1);

    const int input_size = subsystem_.get_input_port(0).size();
    this->DeclareVectorInputPort(BasicVector<T>(input_size));
    BasicVector<T> model_vector(subsystem_.get_output_port(0).size());
    this->DeclareVectorOutputPort(model_vector, &EquilibriumSystem::CalcState);

    if (time_period_ != 0.) {
      this->DeclarePeriodicDiscreteUpdate(time_period_);
    }

    // We are appending an additional parameter to the existing vector.
    base_vector_index_ = subcontext_->num_numeric_parameters();
  }

  template <typename U>
  EquilibriumSystem(const EquilibriumSystem<U>& other)
      : EquilibriumSystem(other.subsystem_, other.time_period_) {}

  BasicVector<T>* get_mutable_base_vector(Context<T>* context) const {
    return this->GetMutableNumericParameter(context, base_vector_index_);
  }

  double time_period() const { return time_period_; }

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
  template <typename>
  friend class EquilibriumSystem;

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {
    SetTimeStateAndParametersFromT(context);
    GetSubsystem(T(0.)).CalcTimeDerivatives(*GetSubContext(T(0.)), derivatives);

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
    GetSubsystem(T(0.)).CalcDiscreteVariableUpdates(*GetSubContext(T(0.)),
                                                    updates);

    const BasicVector<T>& base_states =
        this->template GetNumericParameter<BasicVector>(context,
                                                        base_vector_index_);
    updates->get_mutable_vector()->SetFromVector(
        updates->get_vector()->get_value() - base_states.get_value());
  }

  std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const override {
    auto vector = std::make_unique<BasicVector<T>>(
        GetSubContext(T(0.))->get_continuous_state_vector().CopyToVector());
    return std::make_unique<ContinuousState<T>>(std::move(vector));
  }

  std::unique_ptr<DiscreteValues<T>> AllocateDiscreteState() const override {
    auto vector = std::make_unique<BasicVector<T>>(
        GetSubContext(T(0.))->get_discrete_state(0)->CopyToVector());
    return std::make_unique<DiscreteValues<T>>(std::move(vector));
  }

  std::unique_ptr<Parameters<T>> AllocateParameters() const override {
    std::vector<std::unique_ptr<BasicVector<T>>> params;
    const Context<T>* subcontext = GetSubContext(T(0.));
    params.reserve(subcontext->num_numeric_parameters() + 1);
    for (int i = 0; i < subcontext->num_numeric_parameters(); ++i) {
      auto param = subcontext->get_numeric_parameter(i)->Clone();
      params.emplace_back(std::move(param));
    }
    // Append an additional set of parameters for the base derivatives/updates.
    params.emplace_back(std::make_unique<BasicVector<T>>(
        std::max(subcontext->get_continuous_state_vector().size(),
                 subcontext->get_discrete_state(0)->size())));

    return std::make_unique<Parameters<T>>(std::move(params));
  }

  // Initialize the appended parameters to zero values.
  void SetDefaultParameters(const LeafContext<T>&,
                            Parameters<T>* parameters) const override {
    BasicVector<T>* p =
        parameters->get_mutable_numeric_parameter(base_vector_index_);
    p->SetFromVector(VectorX<T>::Constant(p->size(), 0.));
  }

  void SetTimeStateAndParametersFromT(const Context<T>& context) const {
    Context<T>* subcontext = GetSubContext(T(0.));
    subcontext->set_time(context.get_time());
    subcontext->get_mutable_state()->CopyFrom(context.get_state());
    for (int i{0}; i < subcontext->num_numeric_parameters(); ++i) {
      subcontext->get_mutable_numeric_parameter(i)->SetFromVector(
          context.get_numeric_parameter(i)->get_value());
    }
    subcontext->FixInputPort(0, this->EvalVectorInput(context, 0)->get_value());
  }

  // TODO(jadecastro) The parameter is silly. Get rid of it?  Or instead use
  // enable_if?
  const System<AutoDiffXd>& GetSubsystem(const AutoDiffXd) const {
    DRAKE_DEMAND(diff_subsystem_ != nullptr);
    return *diff_subsystem_;
  }

  const System<double>& GetSubsystem(const double) const { return subsystem_; }

  Context<AutoDiffXd>* GetSubContext(const AutoDiffXd) const {
    DRAKE_DEMAND(diff_subcontext_ != nullptr);
    return diff_subcontext_.get();
  }

  Context<double>* GetSubContext(const double) const {
    DRAKE_DEMAND(subcontext_ != nullptr);
    return subcontext_.get();
  }

  const System<double>& subsystem_;
  const std::unique_ptr<Context<double>> subcontext_;
  const std::unique_ptr<System<AutoDiffXd>> diff_subsystem_;
  const std::unique_ptr<Context<AutoDiffXd>> diff_subcontext_;

  const double time_period_{0.};

  int base_vector_index_{-1};
};

namespace {

struct TimeVaryingData {
  /// Default constructor.
  TimeVaryingData() = default;
  /// Fully-parameterized constructor.
  TimeVaryingData(const PiecewisePolynomialTrajectory& Ain,
                  const PiecewisePolynomialTrajectory& Bin,
                  const PiecewisePolynomialTrajectory& Cin,
                  const PiecewisePolynomialTrajectory& Din,
                  const PiecewisePolynomialTrajectory& x0_in,
                  const PiecewisePolynomialTrajectory& u0_in)
      : A(Ain), B(Bin), C(Cin), D(Din), x0(x0_in), u0(u0_in) {}

  PiecewisePolynomialTrajectory A{PiecewisePolynomial<double>()};
  PiecewisePolynomialTrajectory B{PiecewisePolynomial<double>()};
  PiecewisePolynomialTrajectory C{PiecewisePolynomial<double>()};
  PiecewisePolynomialTrajectory D{PiecewisePolynomial<double>()};
  PiecewisePolynomialTrajectory x0{PiecewisePolynomial<double>()};
  PiecewisePolynomialTrajectory u0{PiecewisePolynomial<double>()};
};

}  // namespace

/// A system containing a time-varying system of the form:
///
/// @f[ xdot(t) = A(t)x(t) + B(t)u(t) + f0(t)  @f]
/// @f[ y(t) = A(t)x(t) + B(t)u(t) + y0(t)  @f]
///
/// where the parameters A, B, C, D, f0, and y0 are taken from a nominal
/// state/input trajectory that is supplied to the constructor.  In particular,
/// f0 = x0(k) - x0(k-1);
//
// ****** TODO ^ Fix this notation!
template <typename T>
class TimeScheduledAffineSystem final : public TimeVaryingAffineSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimeScheduledAffineSystem)

  TimeScheduledAffineSystem(const EquilibriumSystem<double>& model,
                            std::unique_ptr<PiecewisePolynomialTrajectory> x0,
                            std::unique_ptr<PiecewisePolynomialTrajectory> u0,
                            double time_period)
      : TimeScheduledAffineSystem<T>(
            SystemTypeTag<systems::controllers::TimeScheduledAffineSystem>{},
            MakeTimeVaryingData(model, *x0, *u0), time_period) {}

  TimeScheduledAffineSystem(const TimeVaryingData& data, double time_period)
      : TimeScheduledAffineSystem<T>(
            SystemTypeTag<systems::controllers::TimeScheduledAffineSystem>{},
            data, time_period) {}

  ~TimeScheduledAffineSystem() override {}

  template <typename U>
  TimeScheduledAffineSystem(const TimeScheduledAffineSystem<U>& other)
      : TimeScheduledAffineSystem(other.data_, other.time_period()) {}

  VectorX<T> x0(const T& t) const {
    return data_.x0.value(ExtractDoubleOrThrow(t));
  }
  VectorX<T> u0(const T& t) const {
    return data_.u0.value(ExtractDoubleOrThrow(t));
  }

  MatrixX<T> A(const T& t) const override {
    return data_.A.value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> B(const T& t) const override {
    return data_.B.value(ExtractDoubleOrThrow(t));
  }
  VectorX<T> f0(const T& t) const override {
    return VectorX<T>::Zero(C(t).rows());  // <-- Replace with num_outputs.
  }
  MatrixX<T> C(const T& t) const override {
    return data_.C.value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> D(const T& t) const override {
    return data_.D.value(ExtractDoubleOrThrow(t));
  }
  VectorX<T> y0(const T& t) const override {
    return VectorX<T>::Zero(C(t).rows());  // C(t) * x0(t) + D(t) * u0(t);
  }

  double start_time() const { return data_.x0.get_start_time(); }
  double end_time() const { return data_.x0.get_end_time(); }

 protected:
  TimeScheduledAffineSystem(SystemScalarConverter converter,
                            const TimeVaryingData& data, double time_period)
      : TimeVaryingAffineSystem<T>(std::move(converter), data.A.rows(),
                                   data.B.cols(), data.C.rows(), time_period),
        data_(data) {}

 private:
  template <typename>
  friend class TimeScheduledAffineSystem;

  void SetBaseDerivativesAt(const EquilibriumSystem<double>& model,
                            Context<double>* base_context) const {
    // Set the time derivatives at the base-point in order to linearize the
    // system about non-equilibrium.  Note that these derivatives are brought in
    // as parameters.
    BasicVector<double>* base_vector =
        model.get_mutable_base_vector(base_context);
    if (model.time_period() == 0.) {
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

  TimeVaryingData MakeTimeVaryingData(const EquilibriumSystem<double>& model,
                                      const PiecewisePolynomialTrajectory& x0,
                                      const PiecewisePolynomialTrajectory& u0) {
    // TODO(jadecastro) Basic checks of x0, u0 against the model.

    TimeVaryingData result;
    result.x0 = x0;
    result.u0 = u0;
    // x0 and u0 must be consistent with respect to the dynamics of the plant
    // model...

    // TODO(jadecastro) A safer route would be to specify the trajectory for u0
    // and only an initial x0, which is then pumped through the system over that
    // time horizon.  However, this might make things less efficient....

    const std::vector<double> times =
        result.x0.get_piecewise_polynomial().getSegmentTimes();
    std::vector<MatrixX<double>> A_vector{};
    std::vector<MatrixX<double>> B_vector{};
    std::vector<MatrixX<double>> C_vector{};
    std::vector<MatrixX<double>> D_vector{};
    for (auto t : times) {
      // Make a base_context that contains this time index's trajectory data.
      std::unique_ptr<Context<double>> base_context =
          model.CreateDefaultContext();
      auto state_vector =
          base_context->get_mutable_discrete_state()->get_mutable_vector();

      const VectorX<double> state_ref = result.x0.value(t);
      state_vector->set_value(state_ref);

      const VectorX<double> input_ref = result.u0.value(t);
      auto input_vector =
          std::make_unique<BasicVector<double>>(result.u0.rows());
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
    std::cout << " MakeTimeVaryingData " << std::endl;

    // Create matrices for a piecewise-linear system.
    result.A = PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::FirstOrderHold(times, A_vector));
    result.B = PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::FirstOrderHold(times, B_vector));
    result.C = PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::FirstOrderHold(times, C_vector));
    result.D = PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::FirstOrderHold(times, D_vector));

    return result;
  }

  // Nominal (reference) trajectories.
  TimeVaryingData data_;
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

}  // namesp5ace controllers

// Exclude symbolic::Expression from the scalartype conversion of
// EquilibriumSystem.
namespace scalar_conversion {
template <>
struct Traits<controllers::EquilibriumSystem> : public NonSymbolicTraits {};
template <>
struct Traits<controllers::TimeScheduledAffineSystem>
    : public NonSymbolicTraits {};
}  // namespace scalar_conversion

}  // namespace systems
}  // namespace drake
