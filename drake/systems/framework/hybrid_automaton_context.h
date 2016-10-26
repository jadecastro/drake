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
//#include "drake/systems/framework/diagram_continuous_state.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/supervector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/modal_state.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace systems {

/// The HybridAutomatonContext is a container for all of the data
/// necessary to uniquely determine the computations performed by a
/// HybridAutomaton. Specifically, a HybridAutomatonContext contains
/// contexts and outputs for all the constituent Systems, wired up as
/// specified by calls to `HybridAutomatonContext::Connect`.
///
/// In the context, the size of the context vector may change, as can the inputs
/// and outputs.
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
  //TODO: delete!
  typedef int SystemIndex;
  typedef int PortIndex;
  typedef std::pair<SystemIndex, PortIndex> PortIdentifier;

  typedef int ModeId;

  // TODO: can we get around this lengthy redeclaration by friending?
  typedef typename std::tuple<
    // System model.
    const System<T>*,
    // Formula representing the invariant for this mode.
    const std::vector<symbolic::Formula>*,  // TODO: Eigen??
    // Formula representing the initial conditions for this mode.
    const std::vector<symbolic::Formula>*,  // TODO: Eigen??
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
    DRAKE_DEMAND(id <= contexts_.size() && contexts_.size() == outputs_.size());
    DRAKE_DEMAND(contexts_[id] == nullptr);
    DRAKE_DEMAND(outputs_[id] == nullptr);
    context->set_parent(this);
    contexts_[id] = std::move(context);
    outputs_[id] = std::move(output);
  }

  /// Returns the output structure for a given constituent system at
  /// @p index.  Aborts if @p index is out of bounds, or if no system
  /// has been added to the HybridAutomatonContext at that index.
  SystemOutput<T>* GetSubsystemOutput(ModeId id) const {
    const int num_outputs = static_cast<int>(outputs_.size());
    DRAKE_DEMAND(id >= 0 && id < num_outputs);
    DRAKE_DEMAND(outputs_[id] != nullptr);
    return outputs_[id].get();
  }

  /// Returns the context structure for a given constituent system @p
  /// index.  Aborts if @p index is out of bounds, or if no system has
  /// been added to the HybridAutomatonContext at that index.
  /// TODO(david-german-tri): Rename to get_subsystem_context.
  const Context<T>* GetSubsystemContext(ModeId id) const {
    const int num_contexts = static_cast<int>(contexts_.size());
    DRAKE_DEMAND(id >= 0 && id < num_contexts);
    DRAKE_DEMAND(contexts_[id] != nullptr);
    return contexts_[id].get();
  }

  /// Returns the context structure for a given subsystem @p index.
  /// Aborts if @p index is out of bounds, or if no system has been
  /// added to the HybridAutomatonContext at that index.
  /// TODO(david-german-tri): Rename to get_mutable_subsystem_context.
  Context<T>* GetMutableSubsystemContext() {
    const int num_contexts = static_cast<int>(contexts_.size());
    id = get_mode_id();
    DRAKE_DEMAND(id >= 0 && id < num_contexts);
    DRAKE_DEMAND(contexts_[id] != nullptr);
    return contexts_[id].get();
  }

  /// Recursively sets the time on this context and all subcontexts.
  // TODO: should we only update time for the current active subsystem?
  void set_time(const T& time_sec) override {
    Context<T>::set_time(time_sec);
    for (auto& subcontext : contexts_) {
      if (subcontext != nullptr) {
        subcontext->set_time(time_sec);
      }
    }
  }

  int get_num_input_ports() const override {
    return static_cast<int>(input_ids_.size());
  }

  void SetInputPort(int index, std::unique_ptr<InputPort> port) override {
    DRAKE_ASSERT(index >= 0 && index < get_num_input_ports());
    // TODO(david-german-tri): Set invalidation callbacks.
    GetMutableSubsystemContext()
      ->SetInputPort(index, std::move(port));
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

    // Make deep copies of everything else using the default copy constructors.
    *clone->get_mutable_step_info() = this->get_step_info();

    return clone;
  }

  /// Returns the input port at the given @p index, which of course belongs
  /// to the subsystem whose input was exposed at that index.

  // TODO: need?
  const InputPort* GetInputPort(int index) const override {
    if (index < 0 || index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    const PortIdentifier& id = input_ids_[index];
    SystemIndex system_index = id.first;
    PortIndex port_index = id.second;
    return Context<T>::GetInputPort(*GetSubsystemContext(system_index),
                                    port_index);
  }

 private:
  ModeId get_mode_id(const Context<T>& context) {
    context
  }

  std::vector<PortIdentifier> input_ids_;

  std::vector<std::unique_ptr<SystemOutput<T>>> outputs_;
  std::vector<std::unique_ptr<Context<T>>> contexts_;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  //std::map<ModeId, ModeId> mode_transition_;

  // The hybrid automaton data.
  //std::vector<ModalSubsystem> modal_subsystems_;
  //std::vector<ModeTransition> mode_transitions_;

  // The internal state of the System.
  State<T> state_;

  ModalSubsystem modal_subsystem_;
};

}  // namespace systems
}  // namespace drake
