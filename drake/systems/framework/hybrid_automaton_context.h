#pragma once

// TODO: triage this list.
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

// TODO: triage this list.
#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

// TODO(jadecastro): How to concatenate abstract states from subsystems to the
// parent hybrid system.
//
// TODO(jadecastro): Better use of unique pointers for memory management and
// ownership hierarchy.  See, e.g.:
// https://github.com/ToyotaResearchInstitute/drake-maliput/commit/ ...
//    cc29ae0c2ea265150a4d37554c4e63bf0751d30e
//
// TODO(jadecastro): Implement and test capability to capture hybrid
// system-of-hybrid-systems functionality.
//
// TODO(jadecastro): Consistent naming modalsubsystems vs. modal_subsystems, id
// vs. mode_id, ....

/// HybridAutomatonState is a State, annotated with un-owned pointers
/// to all the mutable subsystem states that it spans.
template <typename T>
  class HybridAutomatonState : public State<T> {
 public:
  /// Constructs a HybridAutomatonSubsystemState consisting of @p size
  /// substates.
  explicit HybridAutomatonState<T>(int size) : State<T>(),
      modal_substates_(size) {}

  /// Returns the substate at @p index.
  State<T>* get_mutable_substate(const int index) {
    DRAKE_DEMAND(index >= 0 && index < (int)modal_substates_.size());
    return modal_substates_[index];
  }

  /// Sets the substate at @p index to @p substate, or aborts if @p index is out
  /// of bounds.
  void set_substate(const int index, State<T>* modal_substate) {
    DRAKE_DEMAND(index >= 0 && index < (int)modal_substates_.size());
    modal_substates_[index] = modal_substate;
  }

 private:
  std::vector<State<T>*> modal_substates_;
};


// TODO(jadecastro): Some comments are in order.
template <typename T>
class ModalSubsystem {
 public:
  typedef int ModeId;
  typedef int PortIdentifier;

  explicit ModalSubsystem(
      ModeId mode_id, System<T>* system,
      std::vector<symbolic::Formula>* invariant,
      std::vector<symbolic::Formula>* initial_conditions,
      std::vector<PortIdentifier> input_port_ids,
      std::vector<PortIdentifier> output_port_ids)
      : mode_id_(mode_id), system_(system), invariant_(invariant),
        initial_conditions_(initial_conditions),
        input_port_ids_(input_port_ids), output_port_ids_(output_port_ids) {}

  explicit ModalSubsystem(
      ModeId mode_id, System<T>* system,
      std::vector<symbolic::Formula>* invariant,
      std::vector<symbolic::Formula>* initial_conditions)
      : mode_id_(mode_id), system_(system), invariant_(invariant),
        initial_conditions_(initial_conditions) { SetDefaultPortIds(); }

  explicit ModalSubsystem(
      ModeId mode_id, System<T>* system,
      std::vector<PortIdentifier> input_port_ids,
      std::vector<PortIdentifier> output_port_ids)
      : mode_id_(mode_id), system_(system), input_port_ids_(input_port_ids),
        output_port_ids_(output_port_ids){ SetDefaultPortIds(); }

  explicit ModalSubsystem(
      ModeId mode_id, System<T>* system)
      : mode_id_(mode_id), system_(system) { SetDefaultPortIds(); }

  ModeId get_mode_id() const { return mode_id_; }
  System<T>* get_system() const {return system_; }
  int get_num_input_ports() const {
    return static_cast<int>(input_port_ids_.size());
  }

  const std::vector<PortIdentifier> get_input_port_ids() const {
    return input_port_ids_;
  }
  std::vector<PortIdentifier>* get_mutable_input_port_ids() {
    return &input_port_ids_;
  }
  const std::vector<PortIdentifier> get_output_port_ids() const {
    return output_port_ids_;
  }
  // TODO(jadecastro): Do we really need the following two getters?
  // TODO: const?
  PortIdentifier get_input_port_id(const int index) const {
    DRAKE_DEMAND(index >=0 && index < (int)input_port_ids_.size());
    return input_port_ids_[index];
  }
  PortIdentifier get_output_port_id(const int index) const {
    DRAKE_DEMAND(index >=0 && index < (int)output_port_ids_.size());
    return output_port_ids_[index];
  }
 private:
  // Index for this mode.
  ModeId mode_id_;
  // TODO(jadecastro): Allow ModeId to take on an `enum` here in place of the
  // `int`.
  // The system model.
  const System<T>* system_;
  // Index set of the input and output ports.
  std::vector<PortIdentifier> input_port_ids_;
  std::vector<PortIdentifier> output_port_ids_;
  // Formula representing the invariant for this mode.
  std::vector<symbolic::Formula>* invariant_;  // TODO: Eigen??
  // Formula representing the initial conditions for this mode.
  std::vector<symbolic::Formula>* initial_conditions_;  // TODO: Eigen??

  void SetDefaultPortIds() {
    if (input_port_ids_.empty()) {
      for (int id = 0; id < system_->get_num_input_ports(); ++id) {
        input_port_ids_.emplace_back(id);
      }
    }
    if (output_port_ids_.empty()) {
      for (int id = 0; id < system_->get_num_output_ports(); ++id) {
        output_port_ids_.emplace_back(id);
      }
    }
    if (invariant_->empty()) {
      invariant_->emplace_back(symbolic::Formula::False());
    }
    if (initial_conditions_->empty()) {
      initial_conditions_->emplace_back(symbolic::Formula::True());
    }
  }
};

/// The HybridAutomatonContext is a container for all of the data necessary to
/// uniquely determine the computations performed by a HybridAutomaton (HA).
/// Specifically, a HybridAutomatonContext contains contexts and outputs for all
/// modal subsystems in the finite-state machine. In addition, it augments the
/// subsystem contexts with an abstract (modal) state designating the current
/// active ModalSubsystem.
///
/// In the context, the size of the state vector may change, but the inputs and
/// outputs must be of fixed size.
///
/// In general, users should not need to interact with a HybridAutomatonContext
/// directly. Use the accessors on Hybrid Automaton instead.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class HybridAutomatonContext : public Context<T> {
 public:
  typedef int ModeId;
  typedef int PortIdentifier;

  /// Constructs a HybridAutomatonContext with a fixed number @p num_subsystems
  /// in a way that allows for dynamic re-sizing during discrete events.
  explicit HybridAutomatonContext(const int num_subsystems)
      : outputs_(num_subsystems), contexts_(num_subsystems),
        state_(num_subsystems) {}

  /// Declares a new subsystem in the HybridAutomatonContext.  Subsystems are
  /// identified by number. If the subsystem has already been declared, aborts.
  ///
  /// User code should not call this method. It is for use during Hybrid context
  /// allocation only.
  //
  // TODO(jadecastro): Replace with const ModalSubsystem<T>&.
  void AddModalSubsystem(ModalSubsystem<T>* modal_subsystem,
                         std::unique_ptr<Context<T>> context,
                         std::unique_ptr<SystemOutput<T>> output) {
    //DRAKE_DEMAND(id <= contexts_.size() &&
    //             contexts_.size() == outputs_.size());
    // TODO: fix gcc-4.9 errors^
    const ModeId mode_id = modal_subsystem->get_mode_id();
    DRAKE_DEMAND(contexts_[mode_id] == nullptr);
    DRAKE_DEMAND(outputs_[mode_id] == nullptr);
    context->set_parent(this);
    contexts_[mode_id] = std::move(context);
    outputs_[mode_id] = std::move(output);
    modal_subsystems_.emplace_back(modal_subsystem);
    //symbolic_states_[mode_id] = std::move();
  }

  /// Declares that a particular input port of a particular subsystem is an
  /// input to the entire Diagram that allocates this Context. Aborts if the
  /// subsystem has not been added to the DiagramContext.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void ExportInput(const ModeId& mode_id, const PortIdentifier& port_id) {
    DRAKE_DEMAND(contexts_[mode_id] != nullptr);  // Ensure that the context
                                                  // exists.
    //DRAKE_ASSERT(!modal_subsystems_[mode_id]->get_input_port_ids().empty());
    modal_subsystems_[mode_id]->
        get_mutable_input_port_ids()->emplace_back(port_id);
  }

  /// Generates the state vector for the entire diagram by wrapping the states
  /// of all the constituent diagrams.
  ///
  /// User code should not call this method. It is for use during
  /// HybridAutomaton context allocation only.
  //
  // TODO(jadecastro): This and DoClone has changed a bit in master... we'll
  // have to refactor it slightly.
  void MakeState(const ModeId mode_id) {
    //const int num_subsystems = static_cast<int>(contexts_.size());
    std::vector<AbstractValue*> hybrid_xm;

    Context<T>* context = contexts_[mode_id].get();
    state_.set_substate(mode_id, context->get_mutable_state());
    // Continuous
    //ContinuousState<T>* hybrid_xc =
    //    context->get_mutable_continuous_state();
    // TODO(jadecastro): Some hackiness to get this guy to compile.
    // Discrete
    const std::vector<BasicVector<T>*>& hybrid_xd =
        context->get_mutable_discrete_state()->get_data();
    // Abstract
    AbstractState* subsystem_xm = context->get_mutable_abstract_state();
    for (int i_xm = 0; i_xm < subsystem_xm->size(); ++i_xm) {
      hybrid_xm.push_back(&subsystem_xm->get_mutable_abstract_state(i_xm));
    }
    // Append another, final element corresponding to the modal subsystem of the
    // HA.
    // ********** CHECK: other implementations of AbstractValue.
    const auto xm_mode_id =
        std::unique_ptr<AbstractValue>(
            new Value<ModalSubsystem<T>>(*modal_subsystems_[mode_id]));
    hybrid_xm.push_back(xm_mode_id.get());
    // TODO(jadecastro): Should we store this paricular index?

    // The wrapper states do not own the constituent state.
    //this->set_continuous_state(std::make_unique<ContinuousState<T>>(
    //std::make_unique<BasicVector<T>>(hybrid_xc->get_mutable_vector()), 0, 0,
    //hybrid_xc->size()));
    //
    // *************!!*#**($ TODO(jadecastro): Commenting it out for now:
    // compilation errors when trying to contruct ContinuousState.
    // TODO(jadecastro): This can be de-prioritized until we understand if this
    // is even needed -- c.f. LeafContext.
    this->set_discrete_state(std::make_unique<DiscreteState<T>>(hybrid_xd));
    this->set_abstract_state(std::make_unique<AbstractState>(hybrid_xm));

    if (context->get_continuous_state() != nullptr) {
      const ContinuousState<T>& xc = *context->get_continuous_state();
      const int num_q = xc.get_generalized_position().size();
      const int num_v = xc.get_generalized_velocity().size();
      const int num_z = xc.get_misc_continuous_state().size();
      const BasicVector<T>& xc_vector =
          dynamic_cast<const BasicVector<T>&>(xc.get_vector());
      this->set_continuous_state(std::make_unique<ContinuousState<T>>(
          xc_vector.Clone(), num_q, num_v, num_z));
    }
  }
  // TODO(jadecastro): Likely a temporary function 'till we wrangle with the API
  // updates to Context<T>.

  /// Returns the output structure for a given constituent system at @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// HybridAutomatonContext at that index.
  SystemOutput<T>* GetSubsystemOutput() const {
    const int num_outputs = static_cast<int>(outputs_.size());
    const ModeId mode_id = this->get_mode_id();
    DRAKE_DEMAND(mode_id >= 0 && mode_id < num_outputs);
    DRAKE_DEMAND(outputs_[mode_id] != nullptr);
    // TODO(jadecastro): Implement `dynamic_cast_or_die`.
     return outputs_[mode_id].get();
  }

  /// Returns the context structure for a given constituent system @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// HybridAutomatonContext at that index.
  const Context<T>* GetSubsystemContext() const {
    const int num_contexts = static_cast<int>(contexts_.size());
    const ModeId id = this->get_mode_id();
    DRAKE_DEMAND(id >= 0 && id < num_contexts);
    DRAKE_DEMAND(contexts_[id] != nullptr);
    // TODO(jadecastro): Implement `dynamic_cast_or_die`.
    return contexts_[id].get();
  }

  /// Returns the context structure for a given subsystem @p index.  Aborts if
  /// @p index is out of bounds, or if no system has been added to the
  /// HybridAutomatonContext at that index.
  Context<T>* GetMutableSubsystemContext() {
    const int num_contexts = static_cast<int>(contexts_.size());
    const ModeId id = this->get_mode_id();
    // TODO: needs mode, which is incompatible with `System` API!!
    DRAKE_DEMAND(id >= 0 && id < num_contexts);
    DRAKE_DEMAND(contexts_[id] != nullptr);
    // TODO(jadecastro): These lines have been repeated throughout... let's
    // consolidate.
    return contexts_[id].get();
  }

  /*
  // TODO(jadecastro): make the naming consistent.
  /// Returns the substate at @p index.
  State<T>* get_mutable_subsystem_state() {
    const int num_states = static_cast<int>(contexts_.size());
    // TODO(jadecastro): Ensure contexts_ has the same cardinality as
    // substates_.
    ModeId id = this->get_mode_id();
    // TODO: needs mode, which is incompatible with `System` API!!
    DRAKE_DEMAND(id >= 0 && id < num_states);
    DRAKE_DEMAND(contexts_[id] != nullptr);
    return substates_[index];
  }
  */

  /// Recursively sets the time on this context and all subcontexts.
  // TODO: should we only advance time for the current active subsystem?
  void set_time(const T& time_sec) override {
    Context<T>::set_time(time_sec);
    for (auto& subcontext : contexts_) {
      if (subcontext != nullptr) {
        subcontext->set_time(time_sec);
      }
    }
  }

  int get_num_input_ports() const override {
    const int num_contexts = static_cast<int>(contexts_.size());
    const ModeId mode_id = this->get_mode_id();
    // TODO: needs mode, which is incompatible with `System` API!!
    DRAKE_DEMAND(mode_id >= 0 && mode_id < num_contexts);
    DRAKE_DEMAND(contexts_[mode_id] != nullptr);
    // TODO(jadecastro): Consolidate the above into a separate function, to
    // avoid repitition.
    return modal_subsystems_[mode_id]->get_num_input_ports();
  }

  void SetInputPort(int port_index, std::unique_ptr<InputPort> port) override {
    // TODO(jadecastro): DependentInputPort? (i.e. eval some output to get the
    // input?)
    const int num_contexts = static_cast<int>(contexts_.size());
    const ModeId mode_id = this->get_mode_id();
    // TODO: needs mode, which is incompatible with `System` API!!
    DRAKE_DEMAND(mode_id >= 0 && mode_id < num_contexts);
    DRAKE_DEMAND(contexts_[mode_id] != nullptr);  // TODO: Make sure we actually
                                                  // have a context to derive
                                                  // the mode from!
    // TODO(jadecastro): Consolidate the above into a separate function, to
    // avoid repitition.
    DRAKE_ASSERT(mode_id >= 0);  // TODO:  && index < get_num_input_ports());
    PortIdentifier subsystem_port_id =
        modal_subsystems_[mode_id]->get_input_port_id(port_index);
    GetMutableSubsystemContext()
        ->SetInputPort(subsystem_port_id, std::move(port));
  }

  ModalSubsystem<T>* get_modal_subsystem() const {
    //const int num_xm = state_.get_abstract_state()->size();
    const AbstractState* xm = this->get_state().get_abstract_state();
    const int size_xm = xm->size();  // TODO(jadecastro): Seems a bit silly.
    return xm->get_abstract_state(size_xm-1).GetValue<ModalSubsystem<T>*>();
  }

  ModeId get_mode_id() const {
    const ModalSubsystem<T>* modal_subsystem = this->get_modal_subsystem();
    //DRAKE_DEMAND(modal_subsystem != nullptr);
    return modal_subsystem->get_mode_id();
  }

  // Mandatory overrides.
  const State<T>& get_state() const override { return state_; }

  State<T>* get_mutable_state() override { return &state_; }

 protected:
  HybridAutomatonContext<T>* DoClone() const override {
    DRAKE_ASSERT(contexts_.size() == outputs_.size());
    const int num_subsystems = static_cast<int>(contexts_.size());
    HybridAutomatonContext<T>* clone
      = new HybridAutomatonContext(num_subsystems);

    // Clone all the subsystem contexts and outputs.  This basically repeats
    // everything in CreateDefaultContext.

    // TODO: Would a simple call to that function still do the trick with less
    // overhead?
    for (ModeId i = 0; i < num_subsystems; ++i) {
      DRAKE_DEMAND(contexts_[i] != nullptr);
      DRAKE_DEMAND(outputs_[i] != nullptr);
      clone->AddModalSubsystem(modal_subsystems_[i], contexts_[i]->Clone(),
                               outputs_[i]->Clone());
    }

    // Clone the external input structure.
    ModalSubsystem<T>* modal_subsystem = get_modal_subsystem();
    for (int i = 0; i < modal_subsystem->get_num_input_ports(); ++i) {
      clone->ExportInput(i, modal_subsystem->get_input_port_id(i));
    }

    // Build the state for the initially-activated subsystem in the HA.
    const ModeId mode_id = 0;   // TODO(jadecastro): Set this value externally.
    clone->MakeState(mode_id);

    // Make deep copies of everything else using the default copy constructors.
    *clone->get_mutable_step_info() = this->get_step_info();

    return clone;
  }

  /// Returns the input port at the given @p index, whose subsystem is derived
  /// from the context.
  const InputPort* GetInputPort(int port_index) const override {
    // TODO(jadecastro): DependentInputPort? (i.e. eval some output to get the
    // input?)
    const int num_contexts = static_cast<int>(contexts_.size());
    const ModeId mode_id = this->get_mode_id();
    // TODO: needs mode, which is incompatible with `System` API!!
    DRAKE_DEMAND(mode_id >= 0 && mode_id < num_contexts);
    DRAKE_DEMAND(contexts_[mode_id] != nullptr);
    // TODO(jadecastro): Consolidate the above into a separate function, to
    // avoid repitition.
    //DRAKE_ASSERT(mode_id >= 0);  // TODO:  && index < get_num_input_ports());
    PortIdentifier subsystem_port_id =
        modal_subsystems_[mode_id]->get_input_port_id(port_index);
    return Context<T>::GetInputPort(*GetSubsystemContext(),
                                    subsystem_port_id);
  }

 private:
  // TODO(jadecastro): Implement this.
  /*
  std::unique_ptr<symbolic::Variable>
  MakeSymbolicVariableFromState() {

  }
  */

  // Containers for all the registered modal subsystems.
  std::vector<std::unique_ptr<SystemOutput<T>>> outputs_;
  std::vector<std::unique_ptr<Context<T>>> contexts_;

  // The internal state of the System.
  HybridAutomatonState<T> state_;

  // ****************** Do we actually need this?
  // ******************* Contrarily, do we actually also need mode_transitions_?
  std::vector<ModalSubsystem<T>*> modal_subsystems_;
};

}  // namespace systems
}  // namespace drake
