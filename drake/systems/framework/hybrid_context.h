#pragma once

// TODO: go through these!!
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/hybrid_continuous_state.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/supervector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// The HybridContext is a container for all of the data necessary to uniquely
/// determine the computations performed by a HybridAutomaton. Specifically, a
/// HybridContext contains contexts and outputs for all the constituent
/// Systems, wired up as specified by calls to `HybridContext::Connect`.
///
/// In the context, the size of the context vector may change, as can the inputs
/// and outputs.
///
/// In general, users should not need to interact with a HybridContext
/// directly. Use the accessors on Hybrid Automaton instead.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class HybridContext : public Context<T> {
 public:
  typedef int SystemIndex;
  typedef int PortIndex;
  typedef std::pair<SystemIndex, PortIndex> PortIdentifier;

  /// Constructs a HybridContext with a fixed number @p num_subsystems in a
  /// way that allows for dynamic re-sizing during discrete events.
  explicit HybridContext(const int num_subsystems)
      : outputs_(num_subsystems), contexts_(num_subsystems) {}

  /// Declares a new subsystem in the HybridContext. Subsystems are identified
  /// by number. If the subsystem has already been declared, aborts.
  ///
  /// User code should not call this method. It is for use during Hybrid
  /// context allocation only.
  void AddSystem(SystemIndex index, std::unique_ptr<Context<T>> context,
                 std::unique_ptr<SystemOutput<T>> output) {
    DRAKE_DEMAND(contexts_[index] == nullptr);
    DRAKE_DEMAND(outputs_[index] == nullptr);
    context->set_parent(this);
    contexts_[index] = std::move(context);
    outputs_[index] = std::move(output);
  }

  /// Declares that the output port specified by @p src is connected to the
  /// input port specified by @p dest.
  ///
  /// User code should not call this method. It is for use during Hybrid
  /// context allocation only.
  void Connect(const PortIdentifier& src, const PortIdentifier& dest) {
    // Identify and validate the source port.
    SystemIndex src_system_index = src.first;
    PortIndex src_port_index = src.second;
    SystemOutput<T>* src_ports = GetSubsystemOutput(src_system_index);
    DRAKE_DEMAND(src_port_index >= 0);
    DRAKE_DEMAND(src_port_index < src_ports->get_num_ports());
    OutputPort* output_port = src_ports->get_mutable_port(src_port_index);

    // Identify and validate the destination port.
    SystemIndex dest_system_index = dest.first;
    PortIndex dest_port_index = dest.second;
    Context<T>* dest_context = GetMutableSubsystemContext(dest_system_index);
    DRAKE_DEMAND(dest_port_index >= 0);
    DRAKE_DEMAND(dest_port_index < dest_context->get_num_input_ports());

    // Construct and install the destination port.
    auto input_port = std::make_unique<DependentInputPort>(output_port);
    dest_context->SetInputPort(dest_port_index, std::move(input_port));

    // Remember the graph structure. We need it in DoClone().
    dependency_graph_[dest] = src;
  }

  /// Generates the state vector for the entire diagram by wrapping the states
  /// of all the constituent diagrams.
  ///
  /// User code should not call this method. It is for use during Hybrid
  /// context allocation only.
  void MakeState() {
    std::vector<ContinuousState<T>*> substates;
    for (auto& context : contexts_) {
      substates.push_back(context->get_mutable_continuous_state());
    }
    this->set_continuous_state(
        std::make_unique<HybridContinuousState<T>>(substates));
  }

  /// Returns the output structure for a given constituent system at @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// HybridContext at that index.
  SystemOutput<T>* GetSubsystemOutput(SystemIndex index) const {
    const int num_outputs = static_cast<int>(outputs_.size());
    DRAKE_DEMAND(index >= 0 && index < num_outputs);
    DRAKE_DEMAND(outputs_[index] != nullptr);
    return outputs_[index].get();
  }

  /// Returns the context structure for a given constituent system @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// HybridContext at that index.
  /// TODO(david-german-tri): Rename to get_subsystem_context.
  const Context<T>* GetSubsystemContext(SystemIndex index) const {
    const int num_contexts = static_cast<int>(contexts_.size());
    DRAKE_DEMAND(index >= 0 && index < num_contexts);
    DRAKE_DEMAND(contexts_[index] != nullptr);
    return contexts_[index].get();
  }

  /// Returns the context structure for a given subsystem @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// HybridContext at that index.
  /// TODO(david-german-tri): Rename to get_mutable_subsystem_context.
  Context<T>* GetMutableSubsystemContext(SystemIndex index) {
    const int num_contexts = static_cast<int>(contexts_.size());
    DRAKE_DEMAND(index >= 0 && index < num_contexts);
    DRAKE_DEMAND(contexts_[index] != nullptr);
    return contexts_[index].get();
  }

  /// Recursively sets the time on this context and all subcontexts.
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

  /// Returns the input port at the given @p index, which of course belongs
  /// to the subsystem whose input was exposed at that index.
  const InputPort* GetInputPort(int index) const override {
    if (index < 0 || index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    const PortIdentifier& id = input_ids_[index];
    SystemIndex system_index = id.first;
    PortIndex port_index = id.second;
    return GetSubsystemContext(system_index)->GetInputPort(port_index);
  }

  void SetInputPort(int index, std::unique_ptr<InputPort> port) override {
    if (index < 0 || index >= get_num_input_ports()) {
      throw std::out_of_range("Input port out of range.");
    }
    const PortIdentifier& id = input_ids_[index];
    SystemIndex system_index = id.first;
    PortIndex port_index = id.second;
    GetMutableSubsystemContext(system_index)
        ->SetInputPort(port_index, std::move(port));
    // TODO(david-german-tri): Set invalidation callbacks.
  }

  const State<T>& get_state() const override { return state_; }

  State<T>* get_mutable_state() override { return &state_; }

 protected:
  HybridContext<T>* DoClone() const override {
    DRAKE_ASSERT(contexts_.size() == outputs_.size());
    const int num_subsystems = static_cast<int>(contexts_.size());
    HybridContext<T>* clone = new HybridContext(num_subsystems);

    // Clone all the subsystem contexts and outputs.
    for (int i = 0; i < num_subsystems; ++i) {
      DRAKE_DEMAND(contexts_[i] != nullptr);
      DRAKE_DEMAND(outputs_[i] != nullptr);
      // When a leaf context is cloned, it will clone the data that currently
      // appears on each of its input ports into a FreestandingInputPort.
      clone->AddSystem(i, contexts_[i]->Clone(), outputs_[i]->Clone());
    }

    // Build a superstate over the subsystem contexts.
    clone->MakeState();

    // Clone the internal graph structure. After this is done, the clone will
    // still have FreestandingInputPorts at the inputs to the Hybrid itself,
    // but all of the intermediate nodes will have DependentInputPorts.
    for (const auto& connection : dependency_graph_) {
      const PortIdentifier& src = connection.second;
      const PortIdentifier& dest = connection.first;
      clone->Connect(src, dest);
    }

    // Clone the external input structure.
    for (const PortIdentifier& id : input_ids_) {
      clone->ExportInput(id);
    }

    // Make deep copies of everything else using the default copy constructors.
    *clone->get_mutable_step_info() = this->get_step_info();

    return clone;
  }

  /// Returns the input port at the given @p index, which of course belongs
  /// to the subsystem whose input was exposed at that index.
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
  std::vector<PortIdentifier> input_ids_;

  std::vector<std::unique_ptr<SystemOutput<T>>> outputs_;
  std::vector<std::unique_ptr<Context<T>>> contexts_;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<PortIdentifier, PortIdentifier> dependency_graph_;

  // The internal state of the System.
  State<T> state_;
};

}  // namespace systems
}  // namespace drake
