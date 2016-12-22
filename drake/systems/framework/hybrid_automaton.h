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

/// DiagramOutput is an implementation of SystemOutput that holds unowned
/// OutputPort pointers. It is used to expose the outputs of constituent systems
/// as outputs of a Diagram.
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

// TODO(jadecastro): I don't think we need this, keeping it 'till absolutely
// sure.
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


/// Hybrid Automaton.

/// Limitations:
///  1) modal_subsystems can be Diagrams or (Leaf)Systems, but cannot themselves
/// be HybridAutomata.
/// TODO: Is there a way to mark systems as being "finalized" such that no
/// other system type can inherit from it?
///  2) modal_subsystems do not have to have consistent state dimensions,
/// input dimensions, nor output dimensions. (really a non-limitation)
/// TODO: Can we accommodate this most general case in the first spiral?
///  3) Only one initial modal_state is allowed.
///  4) The state dimensions can change over time (~check~), but the number of
/// input and output ports must be invariant to protect upstream producers and
/// downstream consumers.
/// TODO: Check the viability of a dynamically-sized state vector within
/// the hybrid automaton context.
///  5) ....
template <typename T>
class HybridAutomaton : public System<T> {
 public:
  typedef int ModeId;

  // TODO: Should we have fields of ModalSubsystem in System<T> instead?
  // Pro: Tighter coupling between System and its attributes.
  // Con: These attributes are really a property of hybrid automata and
  //   are (mostly) meaningless on a standalone basis.
  /*
  typedef typename std::tuple<
      // System model.
      const System<T>*,
      // Formula representing the invariant for this mode.
      const std::vector<symbolic::Formula>*,  // TODO: Eigen??
      // Formula representing the initial conditions for this mode.
      const std::vector<symbolic::Formula>*,  // TODO: Eigen??
      // Index for this mode.
      ModeId>
      ModalSubsystem;
  */

  typedef typename std::tuple<
      // Modal subsystem pair.
      const std::pair<ModalSubsystem*, ModalSubsystem*>*,  // TODO: nested ptrs?
      // Formula representing the guard for this transition.
      const std::vector<symbolic::Formula>*>
      ModeTransition;  // TODO: Eigen??
  // Formula representing the reset map for this transition.
  // TODO: Do we want a formula with limited symantics here, or something?
  // const std::vector<symbolic::Formula>*

  // typedef typename std::pair<const ModalSubsystem*,
  // const ModalSubsystem*> ModalSubsystemPair;
  typedef typename std::pair<const System<T>*, int> PortIdentifier;

  ~HybridAutomaton() override {}

  ///
  // symbolic::Formula* get_mutable_invariant(int index) {
  //  DRAKE_ASSERT(index >= 0 && index < size());
  //
  //  return data_[index];
  //}

  /// Returns the list of modal subsystems.
  std::vector<const systems::System<T>*> GetSystems() const {
    std::vector<const systems::System<T>*> result;
    result.reserve(registered_systems_.size());
    for (const auto& system : registered_systems_) {
      result.push_back(system.get());
    }
    return result;
  }

  /// Returns true if any modal subsystem has direct feedthrough.
  bool has_any_direct_feedthrough() const override {
    for (const auto& modal_subsystem : modal_subsystems_) {
      auto system = get_subsystem(modal_subsystem);
      if (!system->has_any_direct_feedthrough()) {
        return false;
      }
    }
    return true;
  }

  std::unique_ptr<Context<T>> AllocateContext() const override {
    const int num_systems = static_cast<int>(modal_subsystems_.size());
    // Reserve inputs as specified during initialization.
    auto context = std::make_unique<HybridAutomatonContext<T>>(num_systems);

    // Add each constituent system to the Context.
    // TODO(jadecastro): This, or just the current active subsystem??
    for (int i = 0; i < num_systems; ++i) {
      auto system = get_subsystem(modal_subsystems_[i]);
      auto modal_context = system->AllocateContext();
      auto modal_output = system->AllocateOutput(*modal_context);
      auto modal_subsystem = ModalSubsystem(i, system);
      context->AddModalSubsystem(i, std::move(modal_context),
                                 std::move(modal_output));
    }

    // Declare the HA-external inputs.
    //
    // TODO(jadecastro): Remove hard-wired mode index. Compute it here or get it
    // from the current context.
    const int mode_id = 0;
    const ModalSubsystem modal_subsystem = modal_subsystems_[mode_id];
    for (const PortIdentifier& id : modal_subsystem.get_input_port_ids()) {
      context->ExportInput(id);  // TODO: <--- Correct this.
    }

    // Build the state for the initially-activated subsystem in the HA.
    context->MakeState(mode_id);

    return std::unique_ptr<Context<T>>(context.release());
  }

  // NB: The dimension of the state vector defined here is what's expected
  // whenever AllocateTimeDervatives is called.
  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override {
    const int num_systems = static_cast<int>(modal_subsystems_.size());
    auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);
    auto subsystem_state =
        dynamic_cast_or_die<HybridAutomatonState<T>*>(state);

    // Set default state for the HA.
    // TODO(jadecastro): Only need to do this for the current instantiated
    // modal_subsystem?  Ans: NO!  But we're doing it anyway b/c otherwise I
    // think we would have some serious memory issues.
    auto subcontext = hybrid_context->GetSubsystemContext();
    DRAKE_DEMAND(subcontext != nullptr);
    auto substate = subsystem_state->get_mutable_state();
    DRAKE_DEMAND(substate != nullptr);
    const ModeId id = hybrid_context->get_mode_id();
    DRAKE_DEMAND(id < num_systems);
    auto subsystem = modal_subsystems_[id].get_system();
    subsystem->SetDefaultState(*subcontext, substate);
  }

  void SetDefaults(Context<T>* context) const final {
    const int num_systems = static_cast<int>(modal_subsystems_.size());
    auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(context);

    // Set defaults of each constituent system.
    // TODO(jadecastro): Only need to do this for the current instantiated
    // modal_subsystem?
    auto subcontext = hybrid_context->GetMutableSubsystemContext();
    const ModeId id = hybrid_context->get_mode_id();
    auto subsystem = modal_subsystems_[id].get_system();
    subsystem.SetDefaults(subcontext);
  }

  // TODO(jadecastro): This is needs serious attention.
  //  - Do the outputs really switch upon a modal_subsystem switch, as expected?
  //  - We need to make sure the output is consistent, at least with respect to
  //    the cardinality of the ports and consistency of the types.
  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const override {
    auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);
    const ModalSubsystem* modal_subsystem =
        hybrid_context->get_modal_subsystem();

    // auto output = std::make_unique<internal::HybridAutomatonOutput<T>*>(
    //                               new internal::HybridAutomatonOutput<T>);
    auto output = std::make_unique<internal::HybridAutomatonOutput<T>>();
    output->get_mutable_ports()->resize(
        modal_subsystem->get_output_port_ids().size());
    ExposeSubsystemOutputs(*diagram_context, output.get());
    return std::unique_ptr<SystemOutput<T>>(output.release());
  }

  /// ======= Hybrid Automaton Execution Methods ======
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override {
    // Down-cast the context and output to HybridAutomatonContext and
    // DiagramOutput.
    auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);
    auto hybrid_output =
        dynamic_cast_or_die<internal::HybridAutomatonOutput<T>*>(output);

    // Populate the output with pointers to the appropriate subsystem outputs in
    // the HybridAutomatonContext. We do this on every call to EvalOutput, so
    // that the diagram_context and diagram_output are not tightly coupled.
    ExposeSubsystemOutputs(*hybrid_context, hybrid_output);

    // Evaluate the subsystem output port.
    modal_subsystem = hybrid_output->get_modal_subsystem();
    EvaluateOutputPort(*diagram_context, modal_subsystem);
  }

  // TODO(jadecastro): This is a tricky one... no context from which to derive
  // the size of the ContinuousState vector. It probably makes sense to keep the
  // size immutable for now.  Perhaps the best approach is to maintain a vector
  // of ContinuousStates.
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    const ModeId temp_mode_id = 0;
    ModalSubsystem modal_subsystem = modal_subsystems_[temp_mode_id];
    auto system = get_subsystem(modal_subsystem);
    return system->AllocateTimeDerivatives();
  }

  /// Aggregates the discrete update variables from each subsystem into a
  /// DiagramDiscreteVariables.
  std::unique_ptr<DiscreteState<T>> AllocateDiscreteVariables()
      const override {
    const ModeId temp_mode_id = 0;
    ModalSubsystem modal_subsystem = modal_subsystems_[temp_mode_id];
    auto system = get_subsystem(modal_subsystem);
    return system->AllocateDiscreteVariables();
  }

  void EvalTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const override {
    auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);
    const Context<T>* subcontext =
        hybrid_context->GetSubsystemContext();
    // Evaluate the derivative of the current modal subsystem.
    system->EvalTimeDerivatives(*subcontext, derivatives);
  }

  // ====== Modal State Processing Functions ======

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

    const ModeId id = context.get_mode_id(context);
    auto invariant_formula =
        *std::get<1>(modal_subsystems_[id]);  // invariant (TODO: make a getter)
    // The guard is satisfied (returns a non-positive value) when
    // the ball's position is less than or equal to zero and its
    // velocity is non-positive.
    return (invariant_formula.at(0)).Evaluate(state_env);
  }

  // TODO(jadecastro): Implement PerformTransition:
  // The idea is as follows:
  //
  //  1) Updates the modal_state.
  //
  //  2) Performs a reset mapping that takes the states from the current context
  //  and maps it to the a valid initial state for the successor mode.
  //
  //  3) Re-points the input and output ports to the correct place.
  //  Essentially, does ExposeInput and ExposeOutput again.  And, I imagine,
  //  eveything else required to make EvaluateOutputPort happy.




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
      Context<T>* context, const ModalSubsystem* modal_subsystem) const {
    DRAKE_DEMAND(context != nullptr);
    DRAKE_DEMAND(modal_subsystem != nullptr);
    auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(context);
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
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);
    auto subsystem = hybrid_context->get_modal_subsystem()->get_system();
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
  Diagram<AutoDiffXd>* DoToAutoDiffXd() const override {
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
        dynamic_cast_or_die<const DiagramContext<T1>*>(&context);
    const Context<T1>* subcontext = diagram_context->GetSubsystemContext();
    DRAKE_DEMAND(subcontext != nullptr);
    const System<T1>* subsystem =
        subcontext->get_modal_subsystem()->get_system();
    DRAKE_DEMAND(subsystem != nullptr);
    const ModeId id = subcontext->get_modal_subsystem()->get_mode_id();

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
      event.do_publish = std::bind(&Diagram<T1>::HandlePublish, this,
                                   std::placeholders::_1, /* context */
                                   publisher);
      actions->events.push_back(event);
    }

    // Request an update event, if our subsystems want it.
    if (!updater) {
      DiscreteEvent<T1> event;
      event.action = DiscreteEvent<T1>::kUpdateAction;
      event.do_update = std::bind(&Diagram<T1>::HandleUpdate, this,
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
    auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);
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
    auto hybrid_context =
        dynamic_cast_or_die<const HybridAutomatonContext<T>*>(&context);

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
    System<T>* subsystem =
        hybrid_context->get_modal_subsystem()->get_system();
    for (const DiscreteEvent<T>& event : action_details.events) {
      if (event.action == DiscreteEvent<T>::kUpdateAction) {
        subsystem->EvalDiscreteVariableUpdates(*subcontext, event,
                                               discrete_update);
      }
    }
  }

  // The finite-state machine (automaton) whose modes are the modal subsystems
  // and whose edges are the mode transitions.
  struct StateMachine {
    std::vector<ModalSubsystem> modal_subsystems;
    std::vector<ModeTransition> mode_transitions;
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

    // Perform a check to determine that the ports are valid wrt the underlying
    // subsystems and consistent across all subsystems.
    DRAKE_ASSERT(PortsAreValid());
    DRAKE_ASSERT(PortsAreConsistent());

    // Add the inputs to the Diagram topology, and check their invariants.
    for (int id = 0; id < num_inports_; ++id) {
      ExportInput(id);
    }
    for (int id = 0; id < num_outports_; ++id) {
      ExportOutput(id);
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
      }
      for (const PortIdentifier& outport :
               modal_subsystem->get_output_port_ids()) {
        if (outport < 0 || outport >= subsystem->get_num_output_ports()) {
          return false;
        }
      }
    }
    return true;
  }

  // Returns true if the port cardinality is consistent across each subsystem
  // within @p modal_subsystems_.
  bool PortsAreConsistent() const {
    for (const auto& modal_subsystem : modal_subsystems_) {
      if (modal_subsystem->get_input_port_ids().size() != num_inports_ ||
          modal_subsystem->get_output_port_ids().size() != num_outports_) {
        return false;
      }
    }
    return true;
  }

  // Takes ownership of the modal subsystems from HybridAutomatonBuilder. This
  // is only used when converting between scalar types.
  void Own(std::vector<std::unique_ptr<System<T>>> modal_subsystems) {
    // We must be given something to own.
    DRAKE_DEMAND(!modal_subsystems.empty());
    // We must not already own any subsystems.
    DRAKE_DEMAND(modal_subsystems_.empty());
    // All of those checks having passed, take ownership of the subsystems.
    modal_subsystems_ = std::move(modal_subsystems);

    // TODO: decide whether or not the systems need to know who their parent is.
    for (auto& modal_subsystem : modal_subsystems_) {
      System<T>* subsystem = modal_subsystem.get_system();
      subsystem->set_parent(this);
    }
  }

  // Exposes the given port as an input of the Diagram. This function is called
  // during initialization and when a mode transition is enacted.
  void ExportInput(const PortIdentifier& port) {
    const System<T>* const sys = port.first;
    const int port_index = port.second;
    // Fail quickly if this system is not a ModalSubsystem for this HA.
    //GetSystemIndexOrAbort(sys);

    // Add this port to our externally visible topology.
    const auto& subsystem_ports = sys->get_input_ports();
    if (port_index < 0 ||
        port_index >= static_cast<int>(subsystem_ports.size())) {
      throw std::out_of_range("Input port out of range.");
    }
    const auto& subsystem_descriptor = subsystem_ports[port_index];
    SystemPortDescriptor<T> descriptor(
        this, kInputPort, this->get_num_input_ports(),
        subsystem_descriptor.get_data_type(), subsystem_descriptor.get_size());
    this->DeclareInputPort(descriptor);
  }

  // Exposes the given port as an output of the Diagram. This function is called
  // during initialization and when a mode transition is enacted.
  void ExportOutput(const PortIdentifier& port) {
    const System<T>* const sys = port.first;
    const int port_index = port.second;
    // Fail quickly if this system is not a ModalSubsystem for this HA.
    //GetSystemIndexOrAbort(sys);

    // Add this port to our externally visible topology.
    const auto& subsystem_ports = sys->get_output_ports();
    if (port_index < 0 ||
        port_index >= static_cast<int>(subsystem_ports.size())) {
      throw std::out_of_range("Output port out of range.");
    }
    const auto& subsystem_descriptor = subsystem_ports[port_index];
    SystemPortDescriptor<T> descriptor(
        this, kOutputPort, this->get_num_output_ports(),
        subsystem_descriptor.get_data_type(), subsystem_descriptor.get_size());
    this->DeclareOutputPort(descriptor);
  }

  // Evaluates the value of the output port with the given @p id in the given
  // @p context.
  void EvaluateOutputPort(const HybridAutomatonContext<T>& context,
                          const ModalSubsystem& modal_subsystem) const {
    //const System<T>* const system = modal_subsystem->get_system();
    //const ModeId id = modal_subsystem->get_mode_id;
    // TODO(jadecastro): vv Do we need this?
    //SPDLOG_TRACE(log(), "Evaluating output for subsystem {}, port {}",
    //             system->GetPath(), id.second);
    const Context<T>* subsystem_context = context.GetSubsystemContext();
    DRAKE_DEMAND(subsystem_context != nullptr);
    const SystemOutput<T>* subsystem_output = context.GetSubsystemOutput();
    DRAKE_DEMAND(subsystem_output != nullptr);
    system->EvalOutput(*subsystem_context, subsystem_output);
  }

  // Sets up the OutputPort pointers in @p output to point to the subsystem
  // outputs, found in @p context.
  void ExposeSubsystemOutputs(
      const HybridAutomatonContext<T>& hybrid_context,
      internal::HybridAutomatonOutput<T>* output) const {
    // The number of output ports of this diagram must equal the number of
    // ports in the provided DiagramOutput.
    const ModalSubsystem* modal_subsystem =
        hybrid_context.get_modal_subsystem();
    const auto output_port_ids = modal_subsystem->get_output_port_ids();
    const int num_ports =
        static_cast<int>(output_port_ids.size());
    DRAKE_DEMAND(output->get_num_ports() == num_ports);

    for (int i = 0; i < num_ports; ++i) {
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

  // HybridAutomaton objects are neither copyable nor moveable.
  HybridAutomaton(const HybridAutomaton<T>& other) = delete;
  HybridAutomaton& operator=(const HybridAutomaton<T>& other) = delete;
  HybridAutomaton(HybridAutomaton<T>&& other) = delete;
  HybridAutomaton& operator=(HybridAutomaton<T>&& other) = delete;

  // TODO: deprecate.
  std::vector<std::unique_ptr<System<T>>> registered_systems_;

  // Data structures for the hybrid automaton.
  // TODO(jadecastro): Better to store/access data as a map or a multimap?
  std::vector<ModalSubsystem> modal_subsystems_;
  std::vector<ModeTransition> mode_transitions_;

  // Input/output port dimensions.
  int num_inports_;
  int num_outports_;

  friend class HybridAutomatonBuilder<T>;
};

}  // namespace systems
}  // namespace drake
