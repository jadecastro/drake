#pragma once

// TODO: triage this list.
#include <algorithm>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>

// TODO: triage this list.
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/abstract_state.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/hybrid_automaton_context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

// Helper to attempt a dynamic_cast on a type-T pointer, failing if the result
// is nullptr.
template <class T, class... Args>
T* dynamic_cast_or_die(Args&&... args) {
  T* result = dynamic_cast<T*>(new T(std::forward<Args>(args)...));
  DRAKE_DEMAND(result != nullptr);
  return result;
}

template <typename T>
class HybridAutomatonBuilder;

namespace internal {

/// HybridAutomatonOutput is an implementation of SystemOutput that holds
/// and exposes unowned OutputPort pointers.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
class HybridAutomatonOutput : public SystemOutput<T> {
 public:
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

// TODO(jadecastro): I don't think we need these utilities, however keeping it
// around until absolutely sure.
/*
template <typename T>
class DiagramTimeDerivatives : public DiagramContinuousState<T> {
 public:
  explicit DiagramTimeDerivatives(
      std::vector<std::unique_ptr<ContinuousState<T>>> substates)
      : DiagramContinuousState<T>(Unpack(substates)),
        substates_(std::move(substates)) {}

  ~DiagramTimeDerivatives() override {}

 private:
  template <typename U>
  std::vector<U*> Unpack(const std::vector<std::unique_ptr<U>>& in) {
    std::vector<U*> out(in.size());
    std::transform(in.begin(), in.end(), out.begin(),
                   [](auto& p) { return p.get(); });
    return out;
  }

  std::vector<std::unique_ptr<ContinuousState<T>>> substates_;
};
*/

}  // namespace internal


// TODO(jadecastro): Some comments are in order.
template <typename T>
class ModeTransition {
 public:
  // TODO(jadecastro): Use setters instead, like in RigidBody.
  explicit ModeTransition(
      const std::pair<ModalSubsystem<T>*, ModalSubsystem<T>*> edge,
      std::vector<symbolic::Formula>* guard,
      std::vector<symbolic::Formula>* reset)
      : edge_(edge), guard_(guard), reset_(reset) {}

  explicit ModeTransition(
      const std::pair<ModalSubsystem<T>*, ModalSubsystem<T>*> edge)
      : edge_(edge) {}

  const std::pair<ModalSubsystem<T>*, ModalSubsystem<T>*> get_edge() const {
    return edge_;
  }

  /// Returns a clone that includes a deep copy of all the output ports.
  // TODO(jadecastro): Decide whether or not we actually need ModalSubsystems to
  // be unique_ptr, and hence need this function.
  std::unique_ptr<ModeTransition<T>> Clone() const {
    ModeTransition<T>* clone =
        new ModeTransition<T>(edge_, guard_, reset_);
    DRAKE_DEMAND(clone != nullptr);
    return std::unique_ptr<ModeTransition<T>>(clone);
  }

 private:
  // Pair of ModalSubsystems representing the edge for this transition.
  std::pair<ModalSubsystem<T>*, ModalSubsystem<T>*> edge_;
  // Formula representing the guard for this edge.
  std::vector<symbolic::Formula>* guard_;  // TODO: Eigen??
  // Formula representing the reset map for this edge.
  std::vector<symbolic::Formula>* reset_;  // TODO: Eigen??
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

/// Notes:
///  1) Modal_subsystems do not have to have consistent state dimensions.
///
///  2) For each subsystem, the number of input and output ports must be
/// consistent across all other subsystems in the HA, as must their type and
/// dimensions (for continuous/discrete-valued ports). To accomplish this, a
/// subset of the ports for each system can be selected at initialization.
///
///  3) Only one initial modal_state is allowed.
template <typename T>
class HybridAutomaton : public System<T> {
 public:
  typedef int ModeId;
  typedef int PortIdentifier;
  // Formula representing the reset map for this transition.
  // TODO: Do we want a formula with limited symantics here, or something?
  // const std::vector<symbolic::Formula>*

  // typedef typename std::pair<const ModalSubsystem*,
  // const ModalSubsystem*> ModalSubsystemPair;

  ~HybridAutomaton() override {}

  ///
  // symbolic::Formula* get_mutable_invariant(int index) {
  //  DRAKE_ASSERT(index >= 0 && index < size());
  //
  //  return data_[index];
  //}

  /// Returns the list of modal subsystems.
  //std::vector<const ModalSubsystem<T>*> GetModalSubsystems() const {
    /*
    std::vector<const ModalSubsystem<T>*> result;
    result.reserve(modal_subsystems_.size());
    for (const auto& mss : modal_subsystems_) {
      result.push_back(mss);
    }
    */
  std::vector<ModalSubsystem<T>*> GetModalSubsystems() const {
    return modal_subsystems_;
  }

  /// Returns true if any modal subsystem has direct feedthrough.
  // TODO(jadecastro): It seems this should be re-computed for each mode.
  bool has_any_direct_feedthrough() const override {
    return active_has_any_direct_feedthrough_;
  }

  std::unique_ptr<Context<T>> AllocateContext() const override {
    // Reserve inputs as specified during initialization.
    auto context =
        std::make_unique<HybridAutomatonContext<T>>();

    // TODO(jadecastro): Throw if a subsystem is already registered.
    // Add the initial ModalSubsystem to the Context.
    ModalSubsystem<T>* mss = modal_subsystems_[mode_id_init_];
    System<T>* subsystem = mss->get_system();
    auto modal_context = subsystem->AllocateContext();
    auto modal_output = subsystem->AllocateOutput(*modal_context);
    context->RegisterSubsystem(std::make_unique<ModalSubsystem<T>>(*mss),
                               std::move(modal_context),
                               std::move(modal_output));

    // Build the state for the initially-activated subsystem and register its
    // AbstractState.
    context->MakeState();
    context->SetModalState();

    // Declare the HA-external inputs.
    for (const PortIdentifier& id : mss->get_input_port_ids()) {
      context->ExportInput(id);
    }

    return std::unique_ptr<Context<T>>(context.release());
  }

  // NB: The dimension of the state vector defined here is what's expected
  // whenever AllocateTimeDervatives is called.
  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override {
    auto hybrid_context =
        dynamic_cast<const HybridAutomatonContext<T>*>(&context);
    DRAKE_DEMAND(hybrid_context != nullptr);
    // TODO(jadecastro): Throw if the state is inconsistent with context.

    // Set the default state for the HA.
    const auto subcontext = hybrid_context->GetSubsystemContext();
    DRAKE_DEMAND(subcontext != nullptr);
    const ModeId mode_id = hybrid_context->get_mode_id();
    DRAKE_DEMAND(mode_id < num_subsystems());
    auto subsystem = modal_subsystems_[mode_id]->get_system();
    subsystem->SetDefaultState(*subcontext, state);
  }

  void SetDefaults(Context<T>* context) const final {
    auto hybrid_context =
        dynamic_cast<HybridAutomatonContext<T>*>(context);
    DRAKE_DEMAND(hybrid_context != nullptr);

    // Set defaults for the system.
    auto subcontext = hybrid_context->GetMutableSubsystemContext();
    auto subsystem = hybrid_context->GetModalSubsystem()->get_system();
    subsystem->SetDefaults(subcontext);
  }

  // TODO(jadecastro): Check that the outputs really switch upon a
  // modal_subsystem switch, as expected.
  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const override {
    const auto hybrid_context =
        dynamic_cast<const HybridAutomatonContext<T>*>(&context);
    DRAKE_DEMAND(hybrid_context != nullptr);
    const ModalSubsystem<T>* modal_subsystem =
        hybrid_context->GetModalSubsystem();

    // auto output = std::make_unique<internal::HybridAutomatonOutput<T>*>(
    //                               new internal::HybridAutomatonOutput<T>);
    auto output = std::make_unique<internal::HybridAutomatonOutput<T>>();
    output->get_mutable_ports()->resize(
        modal_subsystem->get_num_output_ports());
    ExposeSubsystemOutputs(*hybrid_context, output.get());
    return std::unique_ptr<SystemOutput<T>>(output.release());
  }

  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override {
    // Down-cast the context and output to HybridAutomatonContext and
    // HybridAutomatonOutput.
    const auto hybrid_context =
        dynamic_cast<const HybridAutomatonContext<T>*>(&context);
    DRAKE_DEMAND(hybrid_context != nullptr);
    const auto hybrid_output =
        dynamic_cast<internal::HybridAutomatonOutput<T>*>(output);
    DRAKE_DEMAND(hybrid_output != nullptr);

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
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    const ModeId temp_mode_id = 0;
    auto subsystem = modal_subsystems_[temp_mode_id]->get_system();
    return subsystem->AllocateTimeDerivatives();
  }

  /// Aggregates the discrete update variables from each subsystem into a
  /// DiagramDiscreteVariables.
  std::unique_ptr<DiscreteState<T>> AllocateDiscreteVariables()
      const override {
    const ModeId temp_mode_id = 0;
    auto subsystem = modal_subsystems_[temp_mode_id]->get_system();
    return subsystem->AllocateDiscreteVariables();
  }

  void EvalTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const override {
    const auto hybrid_context =
        dynamic_cast<const HybridAutomatonContext<T>*>(&context);
    DRAKE_DEMAND(hybrid_context != nullptr);
    const Context<T>* subcontext =
        hybrid_context->GetSubsystemContext();
    // Evaluate the derivative of the current modal subsystem.
    auto subsystem = hybrid_context->GetModalSubsystem()->get_system();
    subsystem->EvalTimeDerivatives(*subcontext, derivatives);
  }

  /// @name Methods called upon at execution-time
  /// Evaluate the guard at the current valuation of the state vector.
  T EvalInvariant(const HybridAutomatonContext<T>& context) const {
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
    // TODO: add ^this check to all other context-consuming methods as well.

    // Evaluate the guard function.
    const systems::VectorBase<T>& state = context.get_continuous_state_vector();
    symbolic::Environment state_env;
    for (int i = 0; i < state.size(); i++) {
      // if ( state[i] == double ) {
      std::ostringstream key;
      key << "x" << i;
      symbolic::Variable state_var{key.str()};
      state_env.insert(state_var, state[i]);
      //} else {
      // throw std::runtime_error("types other than double not implemented.");
      //}
    }

    const ModeId mode_id = context.get_mode_id();
    const auto invariant_formula =
        modal_subsystems_[mode_id]->get_invariant();  // invariant (TODO: make
                                                       // a getter)
    // The guard is satisfied (returns a non-positive value) when
    // the ball's position is less than or equal to zero and its
    // velocity is non-positive.
    return (invariant_formula.at(0)).Evaluate(state_env);
  }

  // TODO(jadecastro): Implement PerformTransition (Time will tell if the
  // registration/deregistration idea is okay as far as implementation goes.)
  //
  // The idea is to instantiate systems dynamically, giving up ownership as we
  // do. Specifically:
  //
  //  1) Updates the modal_state.
  //
  //  2) Performs a reset mapping that takes the states from the current context
  //  and maps it to the a valid initial state for the successor mode.
  //
  //  3) Re-points the input and output ports to the correct place.
  //  Essentially, does ExposeInput and ExposeOutput again.  And, I imagine,
  //  eveything else required to make EvaluateOutputPort happy.
  void PerformTransition(std::unique_ptr<ModalSubsystem<T>>& mss,
                         Context<T>* context) {
    auto hybrid_context =
        dynamic_cast<HybridAutomatonContext<T>*>(context);
    DRAKE_DEMAND(hybrid_context != nullptr);
    auto subcontext = mss->get_system()->CreateDefaultContext();
    auto suboutput = mss->get_system()->AllocateOutput(*subcontext);

    // Store any vital info about the modal subsystem we will be installing.
    active_has_any_direct_feedthrough_ =
        mss->get_system()->has_any_direct_feedthrough();

    hybrid_context->DeRegisterSubsystem();  // De-register the active subsystem.
    hybrid_context->RegisterSubsystem(std::move(mss), std::move(subcontext),
                                std::move(suboutput));
    hybrid_context->MakeState();
    hybrid_context->SetModalState();
  }

  /// @name Context-Related Accessors
  /// Returns the subcontext that corresponds to the moda_subsystem.  Classes
  /// inheriting from HybridAutomaton need access to this method in order to
  /// pass their constituent subsystems the apropriate subcontext.

  /// Retrieves the state derivatives for a particular subsystem from the
  /// derivatives. Aborts if @p subsystem is not actually a subsystem of this
  /// diagram. Returns nullptr if @p subsystem is stateless.

  /// TODO(jadecastro): Might be useful, but let's punt on it for now.
  /*
  const ContinuousState<T>* GetSubsystemDerivatives(
      const ContinuousState<T>& derivatives,
      const ModalSystem<T>* modal_subsystem) const {
    DRAKE_DEMAND(subsystem != nullptr);
    DRAKE_DEMAND(derivatives != nullptr);
    const ModeId id = get_mode_id(modal_subsystem);
    return derivatives->get_substate(id);
  }
  */

  /// Returns a constant reference to the subcontext that corresponds to the
  /// system @p subsystem.
  /// Classes inheriting from %Diagram need access to this method in order to
  /// pass their constituent subsystems the apropriate subcontext. Aborts if
  /// @p subsystem is not actually a subsystem of this diagram.

  // TODO(jadecastro): Decide what to do with this. Seems like it's never used,
  // since we're always calling the HybridAutomatonContext version of it.
  /*
  const Context<T>& GetSubsystemContext(
      const Context<T>& context, const ModalSubystem* modal_subsystem) const {
    DRAKE_DEMAND(modal_subsystem != nullptr);
    auto& hybrid_context =
        dynamic_cast<const HybridAutomatonContext<T>&>(context);
    return *hybrid_context.GetSubsystemContext();
  }
  */

  /// Returns the subcontext that corresponds to the system @p subsystem.
  /// Classes inheriting from %Diagram need access to this method in order to
  /// pass their constituent subsystems the apropriate subcontext. Aborts if
  /// @p subsystem is not actually a subsystem of this diagram.
  Context<T>* GetMutableSubsystemContext(
      Context<T>* context, const ModalSubsystem<T>* modal_subsystem) const {
    DRAKE_DEMAND(context != nullptr);
    DRAKE_DEMAND(modal_subsystem != nullptr);
    auto hybrid_context =
        dynamic_cast<HybridAutomatonContext<T>*>(context);
    DRAKE_DEMAND(hybrid_context != nullptr);
    // const ModeId id = get_mode_id(modal_subsystem);
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

  /// Retrieves the state for a particular subsystem from the @p state for the
  /// entire diagram. Aborts if @p subsystem is not actually a subsystem of this
  /// diagram.
  State<T>* GetMutableSubsystemState(State<T>* state,
                                     const System<T>* subsystem) const {
    const int i = GetSystemIndexOrAbort(subsystem);
    auto diagram_state = dynamic_cast<DiagramState<T>*>(state);
    DRAKE_DEMAND(diagram_state != nullptr);
    return diagram_state->get_mutable_substate(i);
  }
  */

  /// Returns the full path of this Diagram in the tree of Diagrams. Implemented
  /// here to satisfy InputPortEvaluatorInterface, although we want the exact
  /// same behavior as in System.
  void GetPath(std::stringstream* output) const override {
    return System<T>::GetPath(output);
  }

 protected:
  HybridAutomaton() {}

  void DoPublish(const Context<T>& context) const override {
    auto hybrid_context =
        dynamic_cast<const HybridAutomatonContext<T>*>(&context);
    DRAKE_DEMAND(hybrid_context != nullptr);
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
        dynamic_cast<const HybridAutomatonContext<T1>*>(&context);
    DRAKE_DEMAND(hybrid_context != nullptr);
    const Context<T1>* subcontext = hybrid_context->GetSubsystemContext();
    DRAKE_DEMAND(subcontext != nullptr);
    const System<T1>* subsystem =
        hybrid_context->GetModalSubsystem()->get_system();
    DRAKE_DEMAND(subsystem != nullptr);
    const ModeId id = hybrid_context->GetModalSubsystem()->get_mode_id();

    // Retrieve the update time for this subsystem.
    UpdateActions<T1> sub_action;
    actions->time = subsystem->CalcNextUpdateTime(*subcontext, &sub_action);

    // If no discrete actions are needed, bail early.
    if (actions->time == std::numeric_limits<T1>::infinity()) {
      return;
    }

    std::pair<int, UpdateActions<T1>> publisher;
    std::pair<int, UpdateActions<T1>> updater;
    // Ignore the subsystems that aren't among the most imminent updates.
    if (sub_action.time <= actions->time) {
      if (internal::HasEvent(sub_action,
                             DiscreteEvent<T1>::kPublishAction)) {
        publisher = std::make_pair(id, sub_action);
      }
      if (internal::HasEvent(sub_action,
                             DiscreteEvent<T1>::kUpdateAction)) {
        updater = std::make_pair(id, sub_action);
      }
    }
    DRAKE_ASSERT(!publisher || !updater);

    // Request a publish event, if our subsystems want it.
    if (!publisher) {
      DiscreteEvent<T1> event;
      event.action = DiscreteEvent<T1>::kPublishAction;
      event.do_publish = std::bind(&HybridAutomaton<T1>::HandlePublish, this,
                                   std::placeholders::_1, /* context */
                                   publisher);
      actions->events.push_back(event);
    }

    // Request an update event, if our subsystems want it.
    if (!updater) {
      DiscreteEvent<T1> event;
      event.action = DiscreteEvent<T1>::kUpdateAction;
      event.do_update = std::bind(&HybridAutomaton<T1>::HandleUpdate, this,
                                  std::placeholders::_1, /* context */
                                  std::placeholders::_2, /* difference state */
                                  updater);
      actions->events.push_back(event);
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
        "The default implementation of Diagram<T>::DoCalcNextUpdateTime "
        "only works with types that are drake::is_numeric.");
  }

  /// Handles Publish callbacks that were registered in DoCalcNextUpdateTime.
  /// Dispatches the Publish events to the subsystems that requested them.
  void HandlePublish(
      const Context<T>& context,
      const std::pair<int, UpdateActions<T>>& sub_action) const {
    const auto hybrid_context =
        dynamic_cast<const HybridAutomatonContext<T>*>(&context);
    DRAKE_DEMAND(hybrid_context != nullptr);
    //const int index = sub_action.first;
    const UpdateActions<T>& action_details = sub_action.second;

    const Context<T>* subcontext =
        hybrid_context->GetSubsystemContext();
    DRAKE_DEMAND(subcontext != nullptr);
    for (const DiscreteEvent<T>& event : action_details.events) {
      if (event.action == DiscreteEvent<T>::kPublishAction) {
        modal_subsystems_[index]->get_system()->Publish(*subcontext, event);
      }
    }
  }

  /// Handles Update calbacks that were registered in DoCalcNextUpdateTime.
  /// Dispatches the Publish events to the subsystems that requested them.
  void HandleUpdate(
      const Context<T>& context, DiscreteState<T>* discrete_update,
      const std::pair<int, UpdateActions<T>>& sub_action) const {
    const auto hybrid_context =
        dynamic_cast<const HybridAutomatonContext<T>*>(&context);
    DRAKE_DEMAND(hybrid_context != nullptr);

    // As a baseline, initialize all the difference variables to their
    // current values.
    for (int i = 0; i < discrete_update->size(); ++i) {
      discrete_update->get_mutable_discrete_state(i)->set_value(
          context.get_discrete_state(i)->get_value());
    }

    // Then, allow the subsystem to update a discrete variable.
    //const int index = action.first;
    const UpdateActions<T>& action_details = action.second;

    // Get the context and the difference state for the specified system.
    const Context<T>* subcontext =
        hybrid_context->GetSubsystemContext();
    DRAKE_DEMAND(subcontext != nullptr);

    // Process that system's update actions.
    const System<T>* subsystem =
        hybrid_context->GetModalSubsystem()->get_system();
    for (const DiscreteEvent<T>& event : action_details.events) {
      if (event.action == DiscreteEvent<T>::kUpdateAction) {
        subsystem->EvalDiscreteVariableUpdates(*subcontext, event,
                                               discrete_update);
      }
    }
  }

  // The finite-state machine (automaton) whose modes are the modal subsystems
  // and whose edges are the mode transitions.
  // TODO(jadecastro): Add a set of initial modes.
  struct StateMachine {
    std::vector<ModalSubsystem<T>*> modal_subsystems;
    std::vector<ModeTransition<T>*> mode_transitions;
  };

  // Constructor for the HybridAutomaton.
  explicit HybridAutomaton(const StateMachine& state_machine) {
    Initialize(state_machine);
  }

  // Validates the given @p state_machine and general set-up.
  // TODO: can we roll this up more cleanly, possibly within the calling
  // function?
  void Initialize(const StateMachine& state_machine) {
    // Expect the modal subsytems to be empty (this suffices to conclude that
    // the list of mode transitions is also empty).
    DRAKE_DEMAND(modal_subsystems_.empty());
    // Ensure that we have been given a nontrivial state machine to initialize.
    DRAKE_DEMAND(!state_machine.modal_subsystems.empty());

    // Copy the data from the state_machine into private member variables.
    modal_subsystems_ = state_machine.modal_subsystems;
    mode_transitions_ = state_machine.mode_transitions;
    num_inports_ = modal_subsystems_[0]->get_input_port_ids().size();
    num_outports_ = modal_subsystems_[0]->get_input_port_ids().size();
    // TODO(jadecastro): Check reachability of subsystems from the initial
    // mode (prune, if necessary).

    // Perform some checks to determine that each ModalSubsystem indeed
    // satisfies the invariants for the chosen input/output ports.
    DRAKE_ASSERT(PortsAreValid());
    DRAKE_ASSERT(PortsAreConsistent());

    // Add the inputs, and check their invariants,
    // defaulting arbitrarily at mode 0.
    //
    // TODO(jadecastro): Get the defaults from the initial
    // HybridAutomatonContext.
    const ModeId mode_id = 0;
    const System<T>* subsystem = modal_subsystems_[mode_id]->get_system();
    for (auto id : modal_subsystems_[mode_id]->get_input_port_ids()) {
      ExportInput(*subsystem, id);
    }
    for (auto id : modal_subsystems_[mode_id]->get_output_port_ids()) {
      ExportOutput(*subsystem, id);
    }
  }

  // Returns true if every port enumerated in @p input_port_ids and @p
  // output_port_ids are sane with respect to each subsystem within @p
  // modal_subsystems_.
  bool PortsAreValid() const {
    for (const auto& modal_subsystem : modal_subsystems_) {
      const System<T>* subsystem = modal_subsystem->get_system();
      for (const PortIdentifier& inport :
               modal_subsystem->get_input_port_ids()) {
        if (inport < 0 || inport >= subsystem->get_num_input_ports()) {
          return false;
        }
        // TODO(jadecastro): Check that port types are consistent, and, if
        // DiscreteValue or BasicVector, the dimensions agree too.
      }
      for (const PortIdentifier& outport :
               modal_subsystem->get_output_port_ids()) {
        if (outport < 0 || outport >= subsystem->get_num_output_ports()) {
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

  // Takes ownership of the modal subsystems from HybridAutomatonBuilder. This
  // is only used when converting between scalar types.
  void Own(std::vector<std::unique_ptr<ModalSubsystem<T>>> modal_subsystems) {
    // We must be given something to own.
    DRAKE_DEMAND(!modal_subsystems.empty());
    // We must not already own any subsystems.
    DRAKE_DEMAND(modal_subsystems_.empty());
    // All of those checks having passed, take ownership of the subsystems.
    modal_subsystems_ = std::move(modal_subsystems);

    // TODO: decide whether or not the systems need to know who their parent is.
    for (auto& modal_subsystem : modal_subsystems_) {
      System<T>* subsystem = modal_subsystem->get_system();
      subsystem->set_parent(this);
    }
  }

  // Exposes the given port as an input of the Diagram. This function is called
  // during initialization and when a mode transition is enacted.
  void ExportInput(const System<T>& subsystem, const PortIdentifier port_id) {
    // Fail quickly if this system is not a ModalSubsystem for this HA.
    //GetSystemIndexOrAbort(sys);

    // Add this port to our externally visible topology.
    const auto& subsystem_ports = subsystem.get_input_ports();
    if (port_id < 0 || port_id >= static_cast<int>(subsystem_ports.size())) {
      throw std::out_of_range("Input port out of range.");
    }
    const auto& subsystem_descriptor = subsystem_ports[port_id];
    SystemPortDescriptor<T> descriptor(
        this, kInputPort, this->get_num_input_ports(),
        subsystem_descriptor.get_data_type(), subsystem_descriptor.get_size());
    this->DeclareInputPort(descriptor);
  }

  // Exposes the given port as an output of the Diagram. This function is called
  // during initialization and when a mode transition is enacted.
  void ExportOutput(const System<T>& subsystem, const PortIdentifier port_id) {
    // Fail quickly if this system is not a ModalSubsystem for this HA.
    //GetSystemIndexOrAbort(sys);

    // Add this port to our externally visible topology.
    const auto& subsystem_ports = subsystem.get_output_ports();
    if (port_id < 0 || port_id >= static_cast<int>(subsystem_ports.size())) {
      throw std::out_of_range("Output port out of range.");
    }
    const auto& subsystem_descriptor = subsystem_ports[port_id];
    SystemPortDescriptor<T> descriptor(
        this, kOutputPort, this->get_num_output_ports(),
        subsystem_descriptor.get_data_type(), subsystem_descriptor.get_size());
    this->DeclareOutputPort(descriptor);
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
    subsystem->EvalOutput(*subsystem_context, subsystem_output);
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
      const PortIdentifier& port_id = output_port_ids[i];
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

  // HybridAutomaton objects are neither copyable nor moveable.
  HybridAutomaton(const HybridAutomaton<T>& other) = delete;
  HybridAutomaton& operator=(const HybridAutomaton<T>& other) = delete;
  HybridAutomaton(HybridAutomaton<T>&& other) = delete;
  HybridAutomaton& operator=(HybridAutomaton<T>&& other) = delete;

  bool active_has_any_direct_feedthrough_;

  // TODO(jadecastro): Better to store/access data as a map or a multimap?
  std::vector<ModalSubsystem<T>*> modal_subsystems_;
  std::vector<ModeTransition<T>*> mode_transitions_;
  ModeId mode_id_init_;

  // Fixed input/output port dimensions for the HA.
  int num_inports_;
  int num_outports_;

  friend class HybridAutomatonBuilder<T>;
};

}  // namespace systems
}  // namespace drake
