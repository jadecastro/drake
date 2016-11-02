#pragma once

// TODO: triage this list.
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

// TODO: triage this list.
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_continuous_state.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/modal_state.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace systems {

/// The HybridAutomatonContext is a container for all of the data
/// necessary to uniquely determine the computations performed by a
/// HybridAutomaton. Specifically, a HybridAutomatonContext contains
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

  // TODO: can we get around this lengthy redeclaration by friending?
  typedef typename std::tuple<
    // System model.
    const System<T>*,
    // Formula representing the invariant for this mode.
    const std::vector<symbolic::Formula>*,  // TODO: std::list??
    // Formula representing the initial conditions for this mode.
    const std::vector<symbolic::Formula>*,  // TODO: std::list??
    // Index for this mode.
    ModeId> ModalSubsystem;

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
  void AddModalSubsystem(ModeId id,
                         std::unique_ptr<Context<T>> context,
                         std::unique_ptr<SystemOutput<T>> output) {
    //DRAKE_DEMAND(id <= contexts_.size() && contexts_.size() == outputs_.size());
    // TODO: fix gcc-4.9 errors^
    DRAKE_DEMAND(contexts_[id] == nullptr);
    DRAKE_DEMAND(outputs_[id] == nullptr);
    context->set_parent(this);
    contexts_[id] = std::move(context);
    outputs_[id] = std::move(output);
    //symbolic_states_[id] = std::move();

    // Create a state for this particular mode @p id.
    MakeHybridAutomatonState(id);
  }

  // Generates the state vector for the active subsystem by promoting its
  // context and augmenting it with the modal state.
  void MakeHybridAutomatonState(const ModeId id) {
    // Create a context with the continuous state.
    std::unique_ptr<ContinuousState<T>> substate
      = std::unique_ptr<ContinuousState<T>>(
                           contexts_[id]->get_mutable_continuous_state());
    this->set_continuous_state(
        std::move(substate));

    // Instantiate the new modal state.
    std::vector<AbstractValue*> xm;
    std::unique_ptr<AbstractValue> id_ptr
      = std::unique_ptr<AbstractValue>(new Value<ModeId>(id));
    xm.push_back(id_ptr.get());
    this->set_modal_state(
        std::make_unique<ModalState>(std::move(xm)));
  }

/// Returns the output structure for a given constituent system at
  /// @p index.  Aborts if @p index is out of bounds, or if no system
  /// has been added to the HybridAutomatonContext at that index.
  SystemOutput<T>* GetSubsystemOutput(const Context<T>& context) const {
    const int num_outputs = static_cast<int>(outputs_.size());
    ModeId id = this->get_mode_id(context);
    DRAKE_DEMAND(id >= 0 && id < num_outputs);
    DRAKE_DEMAND(outputs_[id] != nullptr);
    return outputs_[id].get();
  }

  /// Returns the context structure for a given constituent system @p
  /// index.  Aborts if @p index is out of bounds, or if no system has
  /// been added to the HybridAutomatonContext at that index.
  const Context<T>* GetSubsystemContext(const Context<T>& context) const {
    const int num_contexts = static_cast<int>(contexts_.size());
    ModeId id = this->get_mode_id(context);
    DRAKE_DEMAND(id >= 0 && id < num_contexts);
    DRAKE_DEMAND(contexts_[id] != nullptr);
    return contexts_[id].get();
  }

  /// Returns the context structure for a given subsystem @p index.
  /// Aborts if @p index is out of bounds, or if no system has been
  /// added to the HybridAutomatonContext at that index.
  Context<T>* GetMutableSubsystemContext(const Context<T>& context) {
    const int num_contexts = static_cast<int>(contexts_.size());
    ModeId id = this->get_mode_id(context);
    // TODO: needs mode, which is incompatible with `System` API!!
    DRAKE_DEMAND(id >= 0 && id < num_contexts);
    DRAKE_DEMAND(contexts_[id] != nullptr);
    return contexts_[id].get();
  }

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

  ModeId get_mode_id(const Context<T>& context) const {
    //EXPECT_EQ(1, context.get_mutable_modal_state()->size());
    //                     ^ fix
    return context.template get_modal_state<ModeId>(0);
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

      // Make deep copies of the inputs into FreestandingInputPorts.
      for (const auto& port : this->inputs_[i]) {
        if (port == nullptr) {
          clone->inputs_[i].emplace_back(nullptr);
        } else {
          clone->inputs_[i].emplace_back(new FreestandingInputPort(
                                port->template get_vector_data<T>()->Clone()));
        }
      }
    }

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
  std::unique_ptr<symbolic::Variable>
  MakeSymbolicVariableFromState() {

  }

  std::vector<std::vector<std::unique_ptr<InputPort>>> inputs_;
  std::vector<std::unique_ptr<SystemOutput<T>>> outputs_;
  std::vector<std::unique_ptr<Context<T>>> contexts_;

  // The internal state of the System.
  State<T> state_;

  ModalSubsystem modal_subsystem_;
};

}  // namespace systems
}  // namespace drake
