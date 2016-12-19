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
#include "drake/systems/framework/abstract_state.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

// TODO(jadecastro): How to concatenate abstract states from
// subsystems to the parent hybrid system.
//
// TODO(jadecastro): Better use of unique pointers for memory
// management and ownership hierarchy.  See, e.g.:
// https://github.com/ToyotaResearchInstitute/drake-maliput/commit/ ...
//    cc29ae0c2ea265150a4d37554c4e63bf0751d30e
//
// TODO(jadecastro): Implement and test capability to capture hybrid
// system-of-hybrid-systems functionality.
//
// TODO(jadecastro): Consistent naming modalsubsystems
// vs. modal_subsystems, id vs. mode_id, ....

/// HybridAutomatonSubsystemState is a State, annotated with un-owned
/// pointers to all the mutable subsystem states that it spans.
template <typename T>
  class HybridAutomatonState : public State<T> {
 public:
  /// Constructs a HybridAutomatonSubsystemState consisting of @p size
  /// substates.
  explicit HybridAutomatonState<T>(int size) : State<T>(),
      modal_substates_(size) {}

  /// Returns the substate at @p index.
  State<T>* get_mutable_substate(const int index) {
    DRAKE_DEMAND(index >= 0 && index < modal_substates_.size());
    return modal_substates_[index];
  }

  /// Sets the substate at @p index to @p substate, or aborts if @p index is
  /// out of bounds.
  void set_substate(const int index, State<T>* modal_substate) {
    DRAKE_DEMAND(index >= 0 && index < modal_substates_.size());
    modal_substates_[index] = modal_substates;
  }

 private:
  std::vector<State<T>*> modal_substates_;
};


template <typename T>
class ModalSubsystem {
 public:
  explicit ModalSubsystem(
      ModeId mode_id, System<T>* system,
      const std::vector<symbolic::Formula>* invariant,
      const std::vector<symbolic::Formula>* initial_conditions)
      : modal_id_(mode_id), system_(system), invariant_(invariant),
        initial_conditions_(initial_conditions) {}

  explicit ModalSubsystem(
      ModeId mode_id, System<T>* system)
      : modal_id_(mode_id), system_(system) {}

  ModeId get_mode_id() { return mode_id_; }
  System<T>* get_system() {return system_; }
 private:
  // Index for this mode.
  ModeId mode_id_;
  // TODO(jadecastro): Attach a string name too (how does Diagram do this?)
  // System model.
  const System<T>* system_;
  // Formula representing the invariant for this mode.
  const std::vector<symbolic::Formula>* invariant_;  // TODO: Eigen??
  // Formula representing the initial conditions for this mode.
  const std::vector<symbolic::Formula>* initial_conditions_;  // TODO: Eigen??
};


/// The HybridAutomatonContext is a container for all of the data
/// necessary to uniquely determine the computations performed by a
/// HybridAutomaton (HA). Specifically, a HybridAutomatonContext contains
/// contexts and outputs for all modal subsystems in the finite-state
/// machine.
///
/// In the context, the size of the state vector may change, but the
/// inputs and outputs must be of fixed size.
///
/// In general, users should not need to interact with a
/// HybridAutomatonContext directly. Use the accessors on Hybrid
/// Automaton instead.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class HybridAutomatonContext : public Context<T> {
 public:
  typedef int ModeId;

  /// Constructs a HybridAutomatonContext with a fixed number @p
  /// num_subsystems in a way that allows for dynamic re-sizing during
  /// discrete events.
  explicit HybridAutomatonContext(const int num_subsystems)
      : outputs_(num_subsystems), contexts_(num_subsystems) {}

  /// Declares a new subsystem in the HybridAutomatonContext.
  /// Subsystems are identified by number. If the subsystem has
  /// already been declared, aborts.
  ///
  /// User code should not call this method. It is for use during Hybrid
  /// context allocation only.
  void AddModalSubsystem(ModalSubsystem modal_subsystem,
                         std::unique_ptr<Context<T>> context,
                         std::unique_ptr<SystemOutput<T>> output) {
    //DRAKE_DEMAND(id <= contexts_.size() &&
    //             contexts_.size() == outputs_.size());
    // TODO: fix gcc-4.9 errors^
    ModeId id = modal_subsystem.get_mode_id();
    DRAKE_DEMAND(contexts_[id] == nullptr);
    DRAKE_DEMAND(outputs_[id] == nullptr);
    context->set_parent(this);
    contexts_[id] = std::move(context);
    outputs_[id] = std::move(output);
    modal_subsystems_.emplace_back(modal_subsystem);
    //symbolic_states_[id] = std::move();
  }

  /// Generates the state vector for the entire diagram by wrapping the states
  /// of all the constituent diagrams.
  ///
  /// User code should not call this method. It is for use during
  /// HybridAutomaton context allocation only.
  void MakeState(const ModeId mode_id) {
    const int num_subsystems = static_cast<int>(contexts_.size());
    std::vector<AbstractValue*> hybrid_xm;

    Context<T>* context = contexts_[mode_id].get();
    state_.set_substate(id, context->get_mutable_state());
    // Continuous
    const ContinuousState<T>* hybrid_xc =
        context->get_mutable_continuous_state();
    // Discrete
    const std::vector<BasicVector<T>*>& hybrid_xd =
        context->get_mutable_discrete_state()->get_data();
    // Abstract
    AbstractState* xm = context->get_mutable_abstract_state();
    for (int i_xm = 0; i_xm < xm->size(); ++i_xm) {
      hybrid_xm.push_back(&xm->get_mutable_abstract_state(i_xm));
    }
    // The last abstract element corresponds to the modal state of the HA.
    hybrid_xm.push_back(&modal_subsystems_[mode_id]);

    // The wrapper states do not own the constituent state.
    this->set_continuous_state(
        std::make_unique<ContinuousState<T>>(hybrid_xc));
    this->set_discrete_state(std::make_unique<DiscreteState<T>>(hybrid_xd));
    this->set_abstract_state(std::make_unique<AbstractState>(hybrid_xm));
  }

  /// Returns the output structure for a given constituent system at
  /// @p index.  Aborts if @p index is out of bounds, or if no system
  /// has been added to the HybridAutomatonContext at that index.
  SystemOutput<T>* GetSubsystemOutput() const {
    const int num_outputs = static_cast<int>(outputs_.size());
    ModeId id = this->get_mode_id();
    DRAKE_DEMAND(id >= 0 && id < num_outputs);
    DRAKE_DEMAND(outputs_[id] != nullptr);
    return outputs_[id].get();
  }

  /// Returns the context structure for a given constituent system @p
  /// index.  Aborts if @p index is out of bounds, or if no system has
  /// been added to the HybridAutomatonContext at that index.
  const Context<T>* GetSubsystemContext() const {
    const int num_contexts = static_cast<int>(contexts_.size());
    ModeId id = this->get_mode_id();
    DRAKE_DEMAND(id >= 0 && id < num_contexts);
    DRAKE_DEMAND(contexts_[id] != nullptr);
    return contexts_[id].get();
  }

  /// Returns the context structure for a given subsystem @p index.
  /// Aborts if @p index is out of bounds, or if no system has been
  /// added to the HybridAutomatonContext at that index.
  Context<T>* GetMutableSubsystemContext() {
    const int num_contexts = static_cast<int>(contexts_.size());
    ModeId id = this->get_mode_id();
    // TODO: needs mode, which is incompatible with `System` API!!
    DRAKE_DEMAND(id >= 0 && id < num_contexts);
    DRAKE_DEMAND(contexts_[id] != nullptr);
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
    return static_cast<int>(inputs_.size());
  }

  void SetInputPort(int index, std::unique_ptr<InputPort> port) override {
    DRAKE_ASSERT(index >= 0 && index < get_num_input_ports());
    ModeId id = 1;
    inputs_[id][index] = std::move(port);
  }

  ModalSubsystem* get_modal_subsystem() const {
    return dynamic_cast_or_die<ModalSubsystem*>(modal_subsystem_state_);
    // **********************************************************$^^*&(&*(&%^&$$
    // ******** Get modal_subsystem_state_ from somewhere!!
  }

  ModeId get_mode_id() const {
    ModeId mode_id = get_modal_subsystem()->mode;
    return mode_id;
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

    // Clone all the subsystem contexts and outputs.  This basically
    // repeats everything in CreateDefaultContext.

    // TODO: Would a simple call to that function still do the
    // trick with less overhead?
    for (ModeId i = 0; i < num_subsystems; ++i) {
      DRAKE_DEMAND(contexts_[i] != nullptr);
      DRAKE_DEMAND(outputs_[i] != nullptr);
      clone->AddModalSubsystem(i, contexts_[i]->Clone(), outputs_[i]->Clone());
    }

    // Clone the external input structure.
    for (const PortIdentifier& id : input_ids_) {
      clone->ExportInput(id);
    }

    // Build the state for the initially-activated subsystem in the HA.
    mode_id = 0;   // TODO(jadecastro): Set this value externally.
    clone->MakeState(mode_id);

    // Make deep copies of everything else using the default copy constructors.
    *clone->get_mutable_step_info() = this->get_step_info();

    return clone;
  }

  const InputPort* GetInputPort(int index) const override {
    DRAKE_ASSERT(index >= 0 && index < get_num_input_ports());
    ModeId id = 1;
    return inputs_[id][index].get();
  }

 private:
  /*
  std::unique_ptr<symbolic::Variable>
  MakeSymbolicVariableFromState() {

  }
  */
  std::vector<std::vector<std::unique_ptr<InputPort>>> inputs_;
  std::vector<std::unique_ptr<SystemOutput<T>>> outputs_;
  std::vector<std::unique_ptr<Context<T>>> contexts_;

  // The internal state of the System.
  State<T> state_;

  std::vector<ModalSubsystem*> modal_subsystems_;
};

}  // namespace systems
}  // namespace drake
