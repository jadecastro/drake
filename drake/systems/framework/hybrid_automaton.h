#pragma once

// TODO: triage this list.
#include <algorithm>
#include <functional>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

#include "drake/systems/framework/hybrid_automaton_context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

using std::unique_ptr;
using std::make_unique;
using std::cerr;
using std::endl;

// Helper to attempt a dynamic_cast on a type-T pointer, failing if the result
// is nullptr.
template <class T, class... Args>
T dynamic_cast_or_die(Args&&... args) {
  T result = dynamic_cast<T>(T(std::forward<Args>(args)...));
  DRAKE_DEMAND(result != nullptr);
  return result;
}

template <typename T>
class HybridAutomatonBuilder;

namespace internal {

/// A callback that returns true if any of the events in @p actions has type @p
/// type.
template <typename T>
bool HasEvent(const UpdateActions<T> actions,
              typename DiscreteEvent<T>::ActionType type) {
  for (const DiscreteEvent<T>& event : actions.events) {
    if (event.action == type) return true;
  }
  return false;
}

/// HybridAutomatonOutput is an implementation of SystemOutput that holds
/// and exposes unowned OutputPort pointers.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
class HybridAutomatonOutput : public SystemOutput<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HybridAutomatonOutput)

  HybridAutomatonOutput() = default;

  // Required virtual member functions.
  int get_num_ports() const override { return static_cast<int>(ports_.size()); }

  OutputPort* get_mutable_port(int index) override {
    DRAKE_DEMAND(index >= 0 && index < get_num_ports());
    return ports_[index];
  }

  const OutputPort& get_port(int index) const override {
    DRAKE_DEMAND(index >= 0 && index < get_num_ports());
    return *ports_[index];
  }

  std::vector<OutputPort*>* get_mutable_ports() { return &ports_; }

 protected:
  // Returns a clone that has the same number of output ports, set to nullptr.
  HybridAutomatonOutput<T>* DoClone() const override {
    HybridAutomatonOutput<T>* clone = new HybridAutomatonOutput<T>();
    clone->ports_.resize(get_num_ports());
    return clone;
  }

 private:
  std::vector<OutputPort*> ports_;
};

}  // namespace internal


// TODO(jadecastro): Some comments are in order.
template <typename T>
class ModeTransition {
  typedef int ModeId;

 public:
  // TODO(jadecastro): Use setters instead, like in RigidBody.
  explicit ModeTransition(
      const std::pair<ModeId, ModeId> edge,
      std::vector<symbolic::Expression> guard,
      std::vector<symbolic::Expression> reset)
      : edge_(edge), guard_(guard), reset_(reset) {}

  explicit ModeTransition(
      const std::pair<ModeId, ModeId> edge) : edge_(edge) {}

  const std::pair<ModeId, ModeId> get_edge() const { return edge_; }

  ModeId get_predecessor() const { return edge_.first; }

  ModeId get_successor() const { return edge_.second; }

  const std::vector<symbolic::Expression> get_guard() const { return guard_; }

  std::vector<symbolic::Expression>* get_mutable_guard() { return &guard_; }

  // Sets the vector of formulas representing the guard, wiping anything already
  // stored.
  // TODO(jadecastro): Perform checks?
  void set_guard(std::vector<symbolic::Expression>& guard) { guard_ = guard; }

  const std::vector<symbolic::Expression> get_reset() const { return reset_; }

  std::vector<symbolic::Expression>* get_mutable_reset() { return &reset_; }

  // Sets the vector of formulas representing the reset, wiping anything already
  // stored.
  // TODO(jadecastro): Check that dimensions are consistent.
  //                   ^^^ Can we bypass context to extract these dims from
  //                   System?
  void set_reset_throw_if_incompatible(
      std::vector<symbolic::Expression>& reset) {
    reset_ = reset;
  }

  /// Returns a clone that includes a deep copy of all the output ports.
  // TODO(jadecastro): Decide whether or not we actually need ModalSubsystems to
  // be unique_ptr, and hence need this function.
  unique_ptr<ModeTransition<T>> Clone() const {
    ModeTransition<T>* clone =
        new ModeTransition<T>(edge_, guard_, reset_);
    DRAKE_DEMAND(clone != nullptr);
    return unique_ptr<ModeTransition<T>>(clone);
  }

 private:

  // Pair of ModalSubsystems representing the edge for this transition.
  std::pair<ModeId, ModeId> edge_;
  // Expression representing the guard for this edge.
  std::vector<symbolic::Expression> guard_;  // TODO: Eigen??
  // Expression representing the reset map for this edge.
  std::vector<symbolic::Expression> reset_;  // TODO: Eigen??
};

/// HybridAutomaton collects all of the system-related data necessary for
/// simulation and analysis of hybrid automata.
///
/// Across each discrete jump, the size of the state vector for each
/// ModalSubsystem may change, however the inputs and outputs must be of fixed
/// size. This implies that, while each ModalSubsystem does not have to have
/// consistent input/output dimensions for each port type
/// (i.e. continuous/discrete/abstract), these inputs and outputs must be
/// resolved to uniquely-defined ports of fixed type and dimension.
/// Additionally, only one initial mode is supported currently.
template <typename T>
class HybridAutomaton : public System<T>,
                        public detail::InputPortEvaluatorInterface<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HybridAutomaton)

  typedef int ModeId;
  typedef int PortId;

  explicit HybridAutomaton(ModeId mode_init) : mode_id_init_(mode_init) {}
  ~HybridAutomaton() override {}

  /// Returns true if any modal subsystem has direct feedthrough.
  // TODO(jadecastro): It seems this should be re-computed for each mode.
  bool has_any_direct_feedthrough() const override {
    return active_has_any_direct_feedthrough_;
  }

  void set_mode_id_init(const ModeId mode_id) { mode_id_init_ = mode_id; }

  void set_num_expected_input_ports(const int num_inports) {
    num_inports_ = num_inports;
  }

  void set_num_expected_output_ports(const int num_outports) {
    num_outports_ = num_outports;
  }

  unique_ptr<Context<T>> AllocateContext() const override {
    // Reserve inputs as specified during initialization.
    auto context =
        std::make_unique<HybridAutomatonContext<T>>();

    // Add the initial ModalSubsystem to the Context.
    // TODO(jadecastro): Throw if a subsystem is already registered.
    const ModalSubsystem<T>* mss = modal_subsystems_[mode_id_init_].get();
    const System<T>* const subsystem = mss->get_system();
    auto modal_context = subsystem->AllocateContext();
    auto modal_output = subsystem->AllocateOutput(*modal_context);
    DRAKE_DEMAND(modal_context != nullptr);
    DRAKE_DEMAND(modal_output != nullptr);

    unique_ptr<ModalSubsystem<T>> mssptr = mss->Clone();
    context->RegisterSubsystem(std::move(mssptr),
                               std::move(modal_context),
                               std::move(modal_output));

    // Build the state for the initially-activated subsystem and register its
    // AbstractState.
    context->MakeState();
    context->SetModalState();

    // Declare the HA-external inputs.
    for (const PortId& id : mss->get_input_port_ids()) {
      context->ExportInput(id);
    }

    return unique_ptr<Context<T>>(context.release());
  }

  // NB: The dimension of the state vector defined here is what's expected
  // whenever AllocateTimeDervatives is called.
  void SetDefaultState(const Context<T>& context, State<T>* state)
      const override {
    auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);
    auto hybrid_state = dynamic_cast_or_die<HybridAutomatonState<T>*>(state);
    // TODO(jadecastro): Throw if the state is inconsistent with context.

    // Set the default state for the HA.
    const auto subcontext = hybrid_context->GetSubsystemContext();
    DRAKE_DEMAND(subcontext != nullptr);
    const ModeId mode_id = hybrid_context->get_mode_id();
    DRAKE_DEMAND(mode_id < num_subsystems());
    auto subsystem = modal_subsystems_[mode_id]->get_system();
    DRAKE_DEMAND(subsystem != nullptr);
    auto substate = hybrid_state->get_mutable_substate();
    DRAKE_DEMAND(substate != nullptr);

    subsystem->SetDefaultState(*subcontext, substate);
  }

  void SetDefaults(Context<T>* context) const final {
    auto hybrid_context =
        dynamic_cast_or_die<HybridAutomatonContext<T>*>(context);

    // Set defaults for the system.
    auto subcontext = hybrid_context->GetMutableSubsystemContext();
    auto subsystem = hybrid_context->GetModalSubsystem()->get_system();
    subsystem->SetDefaults(subcontext);

    // Register the initial ModalSubsystem.
    const ModalSubsystem<T>* mss_new = modal_subsystems_[mode_id_init_].get();
    CreateAndRegisterSystem(mss_new, hybrid_context);
  }

  unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const override {
    const auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);
    const ModalSubsystem<T>* modal_subsystem =
        hybrid_context->GetModalSubsystem();

    // auto output = std::make_unique<internal::HybridAutomatonOutput<T>*>(
    //                               new internal::HybridAutomatonOutput<T>);
    auto output = std::make_unique<internal::HybridAutomatonOutput<T>>();
    output->get_mutable_ports()->resize(
        modal_subsystem->get_num_output_ports());
    ExposeSubsystemOutputs(*hybrid_context, output.get());
    return unique_ptr<SystemOutput<T>>(output.release());
  }

  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override {
    // Down-cast the context and output to HybridAutomatonContext and
    // HybridAutomatonOutput.
    const auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);
    const auto hybrid_output =
        dynamic_cast_or_die<internal::HybridAutomatonOutput<T>*>(output);

    // Populate the output with pointers to the appropriate subsystem outputs in
    // the HybridAutomatonContext. We do this on every call to EvalOutput, so
    // that the diagram_context and diagram_output are not tightly coupled.
    ExposeSubsystemOutputs(*hybrid_context, hybrid_output);

    // Evaluate the subsystem output port.
    EvaluateOutputPort(*hybrid_context, *hybrid_context->GetModalSubsystem());
  }

  // TODO(jadecastro): This is a tricky one... no context from which to derive
  // the size of the ContinuousState vector. It probably makes sense to keep the
  // size immutable for now.  Perhaps the best approach is to maintain a vector
  // of ContinuousStates.
  unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    const ModeId temp_mode_id = 0;
    auto subsystem = modal_subsystems_[temp_mode_id]->get_system();
    return subsystem->AllocateTimeDerivatives();
  }

  /// Aggregates the discrete update variables from each subsystem into a
  /// DiagramDiscreteVariables.
  unique_ptr<DiscreteState<T>> AllocateDiscreteVariables()
      const override {
    const ModeId temp_mode_id = 0;
    auto subsystem = modal_subsystems_[temp_mode_id]->get_system();
    return subsystem->AllocateDiscreteVariables();
  }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {
    const auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);
    const Context<T>* subcontext =
        hybrid_context->GetSubsystemContext();
    // Evaluate the derivative of the current modal subsystem.
    auto subsystem = hybrid_context->GetModalSubsystem()->get_system();
    subsystem->CalcTimeDerivatives(*subcontext, derivatives);
  }

  /// @name Discrete-Update Utilities

  /// Evaluate the invariant associated with this ModalSubsystem at the current
  /// valuation of the state vector.
  std::vector<T> EvalInvariant(const HybridAutomatonContext<T>& context) const {
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
    ModalSubsystem<T>* modal_subsystem = context.GetModalSubsystem();
    const std::vector<symbolic::Expression>& invariant =
        modal_subsystem->get_invariant();
    // TODO(jadecastro): modal_subsystems_ is spitting out garbage if used here.

    // Evaluate the invariant.
    std::vector<T> result;
    for (auto& expression : invariant) {
      result.emplace_back(EvalExpression(context, expression));
    }
    return result;
  }

  /// Evaluate the initial condition associated with this ModalSubsystem at the
  /// current valuation of the state vector.
  std::vector<T> EvalInitialCondition(
      const HybridAutomatonContext<T>& context) const {
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
    ModalSubsystem<T>* modal_subsystem = context.GetModalSubsystem();
    const std::vector<symbolic::Expression>& init =
        modal_subsystem->get_initial_conditions();

    // Evaluate the initial conditions.
    std::vector<T> result;
    for (auto& expression : init) {
      result.emplace_back(EvalExpression(context, expression));
    }
    return result;
  }

  /// Evaluate the guard associated with all of the successors at the current
  /// valuation of the state vector.
  // TODO(jadecastro): Is this the best output structure to use?
  std::vector<std::vector<T>>
  EvalGuard(const HybridAutomatonContext<T>& context) const {
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
    const ModeId mode_id = context.get_mode_id();

    std::vector<ModeTransition<T>*> mode_transitions =
        GetLegalTransitions(mode_id);
    std::vector<std::vector<T>> result;
    for (auto& mode_transition : mode_transitions) {
      // Evaluate the guard conditions.
      std::vector<T> sub_result;
      for (auto& expression : mode_transition->get_guard()) {
        sub_result.emplace_back(EvalExpression(context, expression));
      }
      result.emplace_back(sub_result);
    }
    return result;
  }

  /// Updates the Context by releasing ownership of the active ModalSubsystem
  /// and providing it ownership of a new ModalSubsystem. Furthermore, updates
  /// the AbstractState for the new mode, exports the inputs and outputs, and
  /// performs a reset mapping to initialize the State for the new subsystem.
  ///
  /// This function is intended for use by the Simulator only.

  // TODO(jadecastro): Need to discuss with Sherm et. al., and possibly write a
  // wrapper so that the Simulator doesn't have to deal directly with
  // ModeTransitions.

  // Note that the Simulator should process the guards and invariants for all
  // possible successors and, if applicable, search for zero crossings for these
  // successors. It should choose which transition to make.

  // TODO(jadecastro): Implement PerformInverseTransition for zero-crossing
  // detection via ODE solvers?

  // TODO(jadecastro): Disable calling with other scalar types?
  void DoPerformTransition(const ModeTransition<T>& mode_transition,
                           HybridAutomatonContext<T>* context) const {
    // ********* Debugging ***********
    const ModalSubsystem<T>* mss_post = modal_subsystems_[0].get();
    //     modal_subsystems_[mode_transition.get_successor()].get();
    // *******************************

    const T& time = context->get_time();
    context->DeRegisterSubsystem();  // De-register the active subsystem.

    // Create a pointer to a deepcopy of the 'post' subsystem.
    CreateAndRegisterSystem(mss_post, context);

    // The time after the jump is the same as that just before.
    context->set_time(time);

    //std::cerr << " (DoPerformTransition) Reset size: "
    //          << mode_transition.get_reset().size() << std::endl;

    // Apply non-identity reset maps to the continuous states for the successor.
    // TODO(jadecastro): How to apply the inverse mapping to satisfy the ODE
    // solver?
    //cerr << " Size of reset map: " << mode_transition.get_reset().size()
    //     << endl;
    //if (mode_transition.get_reset().size() == 0) {
      PerformReset(context, mode_transition);
      //}
  }

  /// Determine the mode transition for the active guard @p index.  @p index
  /// should be consistent with the ids of the returned structures of EvalGuard
  /// and EvalInvariant.

  // TODO(jadecastro): Deal with possible misalignment wrt. the previously-
  // computed guard.
  void PerformTransition(const int index, Context<T>* context) const {
    auto hybrid_context =
        dynamic_cast_or_die<HybridAutomatonContext<T>*>(context);

    const ModeId mode_id = hybrid_context->get_mode_id();
    std::vector<ModeTransition<T>*> mode_transitions =
        GetLegalTransitions(mode_id);
    //std::vector<const ModeTransition<T>*> mode_transitions;
    //mode_transitions.push_back(mode_transitions_.find(0)->second.get());

    DoPerformTransition(*mode_transitions[index], hybrid_context);
  }

  /// @name Context-Related Accessors

  /// Returns the subcontext that corresponds to the system @p subsystem.
  /// Classes inheriting from %Diagram need access to this method in order to
  /// pass their constituent subsystems the apropriate subcontext. Aborts if
  /// @p subsystem is not actually a subsystem of this diagram.
  Context<T>* GetMutableSubsystemContext(Context<T>* context) const {
    DRAKE_DEMAND(context != nullptr);
    auto hybrid_context =
        dynamic_cast_or_die<HybridAutomatonContext<T>*>(context);
    return hybrid_context->GetMutableSubsystemContext();
  }

  // TODO(jadecastro): Might be useful, but punting on it for now.
  /*
  /// Retrieves the state for a particular ModalSubsystem.
  State<T>* GetMutableSubsystemState(Context<T>* context,
                                     const System<T>* subsystem) const {
    Context<T>* subcontext = GetMutableSubsystemContext(context, subsystem);
    return subcontext->get_mutable_state();
  }

  /// Retrieves the state for the active ModalSubsystem from the @p state.
  State<T>* GetMutableSubsystemState(State<T>* state,
                                     const System<T>* subsystem) const {
    auto hybrid_state = dynamic_cast_or_die<HybridAutomatonState<T>*>(state);
    return hybrid_state->get_mutable_substate();
  }
  */

  /// Returns the full path of this HybridAutomaton. Satisfies
  /// InputPortEvaluatorInterface.
  void GetPath(std::stringstream* output) const override {
    return System<T>::GetPath(output);
  }

  /// Evaluates the value of the subsystem input port with the given @p id
  /// in the given @p context. Satisfies InputPortEvaluatorInterface.
  ///
  /// This is a framework implementation detail. User code should not call
  /// this function.
  void EvaluateSubsystemInputPort(
      const Context<T>* context,
      const InputPortDescriptor<T>& descriptor) const override {
    auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(context);
    const ModalSubsystem<T>* mss = hybrid_context->GetModalSubsystem();

    // Find the output port connected to the given input port.
    const PortId inport_id{descriptor.get_index()};

    // If the upstream output port is an input of this whole Diagram, ask our
    // parent to evaluate it.
    const auto inport_ids = mss->get_input_port_ids();
    DRAKE_DEMAND(mss->get_num_input_ports() == num_inports_);
    const auto external_it =
        std::find(inport_ids.begin(), inport_ids.end(), inport_id);
    if (external_it != inport_ids.end()) {
      const int i = external_it - inport_ids.begin();
      this->EvalInputPort(*hybrid_context, i);
    }
  }

 protected:
  HybridAutomaton() {}

  void DoPublish(const Context<T>& context) const override {
    auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);
    const auto subsystem = hybrid_context->GetModalSubsystem()->get_system();
    subsystem->Publish(*hybrid_context->GetSubsystemContext());
  }

  // TODO(jadecastro): Skipping DoMapVelocityToQDot for now -- we will fold in
  // DAEs later.

  /// Computes the next update time based on the configured actions, for scalar
  /// types that are arithmetic, or aborts for scalar types that are not
  /// arithmetic.
  void DoCalcNextUpdateTime(const Context<T>& context,
                            UpdateActions<T>* actions) const override {
    DoCalcNextUpdateTimeImpl(context, actions);
  }

  /// Creates a deep copy of this Diagram<double>, converting the scalar type
  /// to AutoDiffXd, and preserving all internal structure. Diagram subclasses
  /// may wish to override to initialize additional member data, or to return a
  /// more specific covariant type.
  /// This is the NVI implementation of ToAutoDiffXd.
  HybridAutomaton<AutoDiffXd>* DoToAutoDiffXd() const override {
    // TODO(jadecastro): Implement ConvertScalarType. It involves hand-crafting
    // the StateMachine, input_port_ids, output_port_ids, and the list of
    // modal_subsystems in the new type.
    DRAKE_ABORT_MSG("WIP: Cannot convert to AutoDiffXd currently.");
    /*
    return ConvertScalarType<AutoDiffXd>([](const System<double>& subsystem) {
        return subsystem.ToAutoDiffXd();
      })
        .release();
    */
  }

 private:
  // Computes the next update time across all the scheduled events, for
  // scalar types that are numeric.
  //
  // @tparam T1 SFINAE boilerplate for the scalar type. Do not set.
  template <typename T1 = T>
  typename std::enable_if<is_numeric<T1>::value>::type DoCalcNextUpdateTimeImpl(
      const Context<T1>& context, UpdateActions<T1>* actions) const {
    auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T1>*>(&context);
    const Context<T1>* subcontext = hybrid_context->GetSubsystemContext();
    DRAKE_DEMAND(subcontext != nullptr);
    const System<T1>* subsystem =
        hybrid_context->GetModalSubsystem()->get_system();
    DRAKE_DEMAND(subsystem != nullptr);

    // Retrieve the update time for this subsystem.
    UpdateActions<T1> sub_action;
    actions->time = subsystem->CalcNextUpdateTime(*subcontext, &sub_action);

    // If no discrete actions are needed, bail early.
    if (actions->time == std::numeric_limits<T1>::infinity()) {
      return;
    }

    // Ignore the subsystems that aren't among the most imminent updates.
    if (sub_action.time <= actions->time) {
      if (internal::HasEvent(sub_action,
                             DiscreteEvent<T1>::kPublishAction)) {

        // Request a publish event, if our subsystems want it.
        DiscreteEvent<T1> event;
        event.action = DiscreteEvent<T1>::kPublishAction;
        event.do_publish = std::bind(&HybridAutomaton<T1>::HandlePublish, this,
                                     std::placeholders::_1, /* context */
                                     sub_action);
        actions->events.emplace_back(event);
      }
      if (internal::HasEvent(sub_action,
                             DiscreteEvent<T1>::kDiscreteUpdateAction)) {

        // Request an update event, if our subsystems want it.
        DiscreteEvent<T1> event;
        event.action = DiscreteEvent<T1>::kDiscreteUpdateAction;
        event.do_calc_discrete_variable_update = std::bind(
            &HybridAutomaton<T1>::HandleUpdate, this,
            std::placeholders::_1, /* context */
            std::placeholders::_2, /* difference state */
            sub_action);
        actions->events.emplace_back(event);
      }
      if (internal::HasEvent(sub_action,
                             DiscreteEvent<T1>::kUnrestrictedUpdateAction)) {

        // Request an update event, if our subsystems want it.
        DiscreteEvent<T1> event;
        event.action = DiscreteEvent<T1>::kUnrestrictedUpdateAction;
        event.do_unrestricted_update = std::bind(
            &HybridAutomaton<T1>::HandleUnrestrictedUpdate, this,
            std::placeholders::_1, /* context */
            std::placeholders::_2, /* state */
            sub_action);
        actions->events.emplace_back(event);
      }
    }
  }

  // Aborts for scalar types that are not numeric, since there is no reasonable
  // definition of "next update time" outside of the real line.
  //
  // @tparam T1 SFINAE boilerplate for the scalar type. Do not set.
  template <typename T1 = T>
  typename std::enable_if<!is_numeric<T1>::value>::type
  DoCalcNextUpdateTimeImpl(const Context<T1>& context,
                           UpdateActions<T1>* actions) const {
    DRAKE_ABORT_MSG(
        "The default implementation of"
        "HybridAutomaton<T>::DoCalcNextUpdateTime only works with types that"
        "are drake::is_numeric.");
  }

  /// Handles Publish callbacks that were registered in DoCalcNextUpdateTime.
  /// Dispatches the Publish events to the subsystems that requested them.
  void HandlePublish(const Context<T>& context,
                     const UpdateActions<T>& sub_actions) const {
    const auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);
    const ModalSubsystem<T>* mss = hybrid_context->GetModalSubsystem();

    const Context<T>* subcontext = hybrid_context->GetSubsystemContext();
    DRAKE_DEMAND(subcontext != nullptr);
    for (const DiscreteEvent<T>& event : sub_actions.events) {
      if (event.action == DiscreteEvent<T>::kPublishAction) {
        mss->get_system()->Publish(*subcontext, event);
      }
    }
  }

  /// Handles Update calbacks that were registered in DoCalcNextUpdateTime.
  /// Dispatches the Publish events to the subsystems that requested them.
  void HandleUpdate(
      const Context<T>& context, DiscreteState<T>* discrete_update,
      const UpdateActions<T>& sub_actions) const {
    const auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);

    // As a baseline, initialize all the difference variables to their
    // current values.
    for (int i = 0; i < discrete_update->size(); ++i) {
      discrete_update->get_mutable_discrete_state(i)->set_value(
          context.get_discrete_state(i)->get_value());
    }

    // Then, allow the subsystem to update a discrete variable.
    // Get the context for the specified system.
    const Context<T>* subcontext = hybrid_context->GetSubsystemContext();
    DRAKE_DEMAND(subcontext != nullptr);

    // Process that system's update actions.
    const System<T>* subsystem =
        hybrid_context->GetModalSubsystem()->get_system();
    for (const DiscreteEvent<T>& event : sub_actions.events) {
      if (event.action == DiscreteEvent<T>::kDiscreteUpdateAction) {
        subsystem->CalcDiscreteVariableUpdates(*subcontext, event,
                                               discrete_update);
      }
    }
  }

  /// Handles Update calbacks that were registered in DoCalcNextUpdateTime.
  /// Dispatches the Publish events to the subsystems that requested them.
  void HandleUnrestrictedUpdate(const Context<T>& context, State<T>* state,
                                const UpdateActions<T>& sub_actions) const {
    const auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);
    auto hybrid_state = dynamic_cast_or_die<HybridAutomatonState<T>*>(state);

    // No need to set state to context's state, since it has already been done
    // in System::CalcUnrestrictedUpdate().

    const Context<T>* subcontext = hybrid_context->GetSubsystemContext();
    DRAKE_DEMAND(subcontext != nullptr);

    // Process that system's update actions.
    const System<T>* subsystem =
        hybrid_context->GetModalSubsystem()->get_system();
    State<T>* substate = hybrid_state->get_mutable_substate();
    DRAKE_DEMAND(substate != nullptr);
    for (const DiscreteEvent<T>& event : sub_actions.events) {
      if (event.action == DiscreteEvent<T>::kDiscreteUpdateAction) {
        subsystem->CalcUnrestrictedUpdate(*subcontext, event, substate);
      }
    }
  }

  // Evaluate a symbolic formula at a particular continuous state.
  T EvalExpression(const HybridAutomatonContext<T>& context,
                   const symbolic::Expression& expression) const {
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
    // TODO(jadecastro): add ^this to all other context-consuming methods as
    // well.

    // Evaluate the function.
    // TODO(jadecastro): Verify consistency of the state dimensions.

    const systems::VectorBase<T>& xc = context.get_continuous_state_vector();
    // TODO(jadecastro): Replace the xc_sym_ with the following implementation.
    //const std::vector<symbolic::Variable> xc_sym =
    //    context.GetModalSubsystem()->get_symbolic_continuous_states();
    //std::vector<symbolic::Variable> xc_sym;
    //for (auto it :
    //         context.GetModalSubsystem()->get_symbolic_continuous_states()) {
    // xc_sym.push_back(std::move(it));
    //}

    symbolic::Environment xc_env;
    //std::vector<symbolic::Variable> state_sym =
    //    context.get_symbolic_state_variables();
    // std::vector<symbolic::Variable> state_sym =
    //    modal_subsystems_[0]->get_symbolic_state_variables();
    for (int i = 0; i < xc.size(); i++) {
      xc_env.insert(xc_sym_[i], xc[i]);
    }

    // TODO(jadecastro): Implement for discrete state groups.
    //const systems::VectorBase<T>& xd = context.get_discrete_state();
    //const std::vector<symbolic::Variable> xd_sym =
    //    context.GetModalSubsystem()->get_symbolic_discrete_state();
    //symbolic::Environment xd_env;
    //for (int i = 0; i < xd.size(); i++) {
    //  xd_env.insert(xd_sym[i], xd[i]);
    //}

    return expression.Evaluate(xc_env);
    // TODO(jadecastro): Return xd_env also.
  }

  // Modifies the HybridAutomatonContext according to the reset map.
  void PerformReset(HybridAutomatonContext<T>* context,
                    const ModeTransition<T>& mode_transition) const {
    ContinuousState<double>* xc = context->get_mutable_continuous_state();

    // Process the reset function.
    const std::vector<symbolic::Expression> reset = mode_transition.get_reset();

    // Set the continuous state. Note that the update for the discrete and
    // abstract states should already have been made in
    // HybridAutomatonContext<T>::MakeState().

    VectorX<T> values(xc->get_mutable_vector()->size());
    for (int i = 0; i < xc->get_mutable_vector()->size(); ++i) {
      values(i) = EvalExpression(*context, reset[i]);
    }
    // TODO(jadecastro): Process resets for discrete states here.
    xc->get_mutable_vector()->SetFromVector(values);
  }

  // Performs all the required operations required to link the desired subsystem
  // to the current HybridAutomatonContext.
  void CreateAndRegisterSystem(const ModalSubsystem<T>* mss,
                               HybridAutomatonContext<T>* hybrid_context)
      const {
    // Create a pointer to a deepcopy of the 'post' subsystem.
    unique_ptr<ModalSubsystem<T>> mss_new = mss->Clone();
    auto subcontext = mss_new->get_system()->CreateDefaultContext();
    auto suboutput = mss_new->get_system()->AllocateOutput(*subcontext);

    hybrid_context->ExportInput(mss_new->get_input_port_ids());
    hybrid_context->ExportOutput(mss_new->get_output_port_ids());

    hybrid_context->RegisterSubsystem(std::move(mss_new), std::move(subcontext),
                                      std::move(suboutput));

    hybrid_context->MakeState();
    hybrid_context->SetModalState();
  }

  // Gets a vector of legal transitions for a given pre mode. A pair (pre, post)
  // is legal iff it exists in the set of edges.
  std::vector<ModeTransition<T>*> GetLegalTransitions(const ModeId mode_id_pre)
      const {

    // ********************************
    std::vector<ModeTransition<T>*> result;
    // Create a functor `select2nd` to convert the resulting multimap to vector.
    //auto select2nd = std::bind(
    //    &std::multimap<int, shared_ptr<ModeTransition<T>>>::value_type::second,
    //    std::placeholders::_1);

    //transform(mode_transitions_.lower_bound(mode_id_pre),
    //          mode_transitions_.upper_bound(mode_id_pre),
    //          back_inserter(result), select2nd);
    //DRAKE_DEMAND(result.size() > 0);
    // ********************************

    result.push_back(mode_transitions_.find(0)->second.get());
    return result;
  }

  // The finite-state machine (automaton) whose modes are the modal subsystems
  // and whose edges are the mode transitions.
  struct StateMachine {
    std::vector<const ModalSubsystem<T>*> modal_subsystems;
    std::vector<const ModeTransition<T>*> mode_transitions;
    std::set<ModeId> initial_modes;
    ModeId mode_id_init;
    int num_inports, num_outports;
  };

  // Constructor for the HybridAutomaton.
  explicit HybridAutomaton(const StateMachine& state_machine) {
    Initialize(state_machine);
  }

  // Validates the given @p state_machine and general set-up.
  // TODO: can we roll this up more cleanly, possibly within the calling
  // function?
  void Initialize(const StateMachine& state_machine) {
    // TODO(jadecastro): Check reachability of subsystems from the initial
    // mode (prune, if necessary).

    // Ensure that we have data to transfer and that our data structures are
    // empty.
    DRAKE_DEMAND(!state_machine.modal_subsystems.empty());
    DRAKE_DEMAND(!state_machine.mode_transitions.empty());
    DRAKE_DEMAND(modal_subsystems_.empty());
    DRAKE_DEMAND(mode_transitions_.empty());
    DRAKE_DEMAND(initial_modes_.empty());

    // TODO(jadecastro): Remove and replace with an in-place version.
    for (auto it : state_machine.modal_subsystems[0]->
             get_symbolic_continuous_states()) {
      xc_sym_.push_back(std::move(it));
    }

    for (auto& mss : state_machine.modal_subsystems) {
      modal_subsystems_.push_back(unique_ptr<ModalSubsystem<T>>(mss->Clone()));
    }
    //for (auto& mt : state_machine.mode_transitions) {
    for (auto i = 0;
         i < static_cast<int>(state_machine.mode_transitions.size()); ++i) {
      mode_transitions_.insert(
          std::make_pair(i, unique_ptr<ModeTransition<T>>(
              state_machine.mode_transitions[i]->Clone())));
    }
    initial_modes_ = state_machine.initial_modes;
    mode_id_init_ = state_machine.mode_id_init;
    num_inports_ = state_machine.num_inports;
    num_outports_ = state_machine.num_outports;

    // TODO(jadecastro): Do this for all, or just the initial mode?
    //for (auto& modal_subsystem : modal_subsystems_) {
    //System<T>* subsystem = modal_subsystem->get_system();
      // subsystem->set_parent(this); // TODO(jadecastro): Fix: Parent already
      // set!
    //}

    // Perform some checks to determine that each ModalSubsystem indeed
    // satisfies the invariants for the chosen input/output ports.
    DRAKE_ASSERT(PortsAreValid());
    DRAKE_ASSERT(PortsAreConsistent());

    // Add the inputs, and check their invariants,
    const System<T>* subsystem = modal_subsystems_[mode_id_init_]->get_system();
    for (auto id : modal_subsystems_[mode_id_init_]->get_input_port_ids()) {
      ExportInput(*subsystem, id);
    }
    for (auto id : modal_subsystems_[mode_id_init_]->get_output_port_ids()) {
      //cerr << " Outport id: " << id << endl;
      //cerr << " Num output ports: " << subsystem->get_output_ports().size()
      //     << endl;
      ExportOutput(*subsystem, id);
    }

    // Verify that the provided mode_id_init_ is sane.
    // TODO(jadecastro): Work on this (use the hint here):
    //   http://stackoverflow.com/questions/589985/vectors-structs-and-stdfind
    //if (std::find_if(modal_subsystems_.begin(),
    //              modal_subsystems_.end(),
    //              mode_id_init_) != modal_subsystems_.end().get_mode_id()) {
    //  throw std::logic_error("invalid mode_id_init provided!");
    //}

    // If no initial modes have been specified, use all of the registered modes.
    if (!state_machine.initial_modes.empty()) {
      initial_modes_ = state_machine.initial_modes;
    } else {
      for (auto& mss : modal_subsystems_) {
        ModeId mode_id = mss->get_mode_id();
        initial_modes_.insert(mode_id);
      }
    }

    // Throw if our prespecified initial mode is invalid.
    DRAKE_DEMAND(initial_modes_.find(mode_id_init_) != initial_modes_.end());
  }

  // Returns true if every port enumerated in @p input_port_ids and @p
  // output_port_ids are sane with respect to each subsystem within @p
  // modal_subsystems_.
  bool PortsAreValid() const {
    for (const auto& modal_subsystem : modal_subsystems_) {
      const System<T>* subsystem = modal_subsystem->get_system();
      DRAKE_DEMAND(subsystem != nullptr);
      for (const PortId& inport : modal_subsystem->get_input_port_ids()) {
        if (inport < 0 || inport >= modal_subsystem->get_num_input_ports()) {
          return false;
        }
        // TODO(jadecastro): Check that port types are consistent, and, if
        // DiscreteValue or BasicVector, the dimensions agree too.
      }
      for (const PortId& outport : modal_subsystem->get_output_port_ids()) {
        if (outport < 0 || outport >= modal_subsystem->get_num_output_ports()) {
          return false;
        }
        // TODO(jadecastro): Check that port types are consistent, and, if
        // DiscreteValue or BasicVector, the dimensions agree too.
      }
    }
    return true;
  }

  // Returns true if the port cardinality is consistent across each subsystem
  // within @p modal_subsystems_.
  bool PortsAreConsistent() const {
    for (const auto& modal_subsystem : modal_subsystems_) {
      if (modal_subsystem->get_num_input_ports() != num_inports_ ||
          modal_subsystem->get_num_output_ports() != num_outports_) {
        return false;
      }
    }
    return true;
  }

  // Exposes the given port as an input of the HybridAutomaton. This function is
  // called during initialization and when a mode transition is enacted.
  void ExportInput(const System<T>& subsystem, const PortId port_id) {
    // Fail quickly if this system is not a ModalSubsystem for this HA.
    //GetSystemIndexOrAbort(sys);

    // Add this port to our externally visible topology.
    const auto& subsystem_descriptor = subsystem.get_input_port(port_id);
    this->DeclareInputPort(subsystem_descriptor.get_data_type(),
                           subsystem_descriptor.size());
  }

  // Exposes the given port as an output of the Diagram. This function is called
  // during initialization and when a mode transition is enacted.
  void ExportOutput(const System<T>& subsystem, const PortId port_id) {
    // Fail quickly if this system is not a ModalSubsystem for this HA.
    //GetSystemIndexOrAbort(sys);

    // Add this port to our externally visible topology.
    const auto& subsystem_descriptor = subsystem.get_output_port(port_id);
    this->DeclareOutputPort(subsystem_descriptor.get_data_type(),
                            subsystem_descriptor.size());
  }

  // Evaluates the value of the output port with the given @p id in the given
  // @p context.
  void EvaluateOutputPort(const HybridAutomatonContext<T>& context,
                          const ModalSubsystem<T>& modal_subsystem) const {
    const System<T>* const subsystem =
        context.GetModalSubsystem()->get_system();
    // TODO(jadecastro):   vvv Do we need this?
    //SPDLOG_TRACE(log(), "Evaluating output for subsystem {}, port {}",
    //             system->GetPath(), id.second);
    const Context<T>* subsystem_context = context.GetSubsystemContext();
    DRAKE_DEMAND(subsystem_context != nullptr);
    SystemOutput<T>* subsystem_output = context.GetSubsystemOutput();
    DRAKE_DEMAND(subsystem_output != nullptr);
    subsystem->CalcOutput(*subsystem_context, subsystem_output);
  }

  // Sets up the OutputPort pointers in @p output to point to the subsystem
  // outputs, found in @p context.
  void ExposeSubsystemOutputs(
      const HybridAutomatonContext<T>& hybrid_context,
      internal::HybridAutomatonOutput<T>* output) const {
    // The number of output ports of this diagram must equal the number of
    // ports in the provided DiagramOutput.
    const ModalSubsystem<T>* modal_subsystem =
        hybrid_context.GetModalSubsystem();
    const auto output_port_ids = modal_subsystem->get_output_port_ids();
    const int num_outports = modal_subsystem->get_num_output_ports();
    DRAKE_DEMAND(output->get_num_ports() == num_outports);

    for (int i = 0; i < num_outports; ++i) {
      const PortId& port_id = output_port_ids[i];
      // For each configured output port ID, obtain from the
      // HybridAutomatonContext the actual OutputPort that produces it.
      SystemOutput<T>* subsystem_output = hybrid_context.GetSubsystemOutput();
      DRAKE_DEMAND(subsystem_output != nullptr);
      OutputPort* output_port = subsystem_output->get_mutable_port(port_id);
      DRAKE_DEMAND(output_port != nullptr);

      // Make a pointer to the ith OutputPort.
      (*output->get_mutable_ports())[i] = output_port;
    }
  }

  int num_subsystems() const {
    return static_cast<int>(modal_subsystems_.size());
  }

  int num_transitions() const {
    return static_cast<int>(mode_transitions_.size());
  }

  // TODO(jadecastro): System swapping without the const-ness restriction on
  // direct-feedthrough of systems.
  bool active_has_any_direct_feedthrough_{false};

  // TODO(jadecastro): Better to store/access data as a map or a multimap?
  // TODO(jadecastro): Shared ptr
  std::vector<unique_ptr<ModalSubsystem<T>>> modal_subsystems_;
  // TODO(jadecastro): The ModeId key is redundant with edge_.first in
  // ModeTransition<T>.
  std::multimap<int, unique_ptr<ModeTransition<T>>> mode_transitions_;
  // Mode 0 is the default intiial mode unless otherwise specified.
  ModeId mode_id_init_{0};  // TODO(jadecastro): <---Initialize this.

  std::set<ModeId> initial_modes_;

  // Fixed input/output port dimensions for the HA.
  int num_inports_;
  int num_outports_;

  // Store symbolic variables here.
  // TODO(jadecatro): Remove this.
  std::vector<symbolic::Variable> xc_sym_;

  // TODO(jadecastro): symbolic::Variable throws an obscure error when a
  // variable is copied rather than moved.

  friend class HybridAutomatonBuilder<T>;
};

}  // namespace systems
}  // namespace drake
