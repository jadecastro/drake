#pragma once

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/// HybridAutomaton is a System composed of one or more constituent Systems,
/// arranged in a directed graph where the vertices are the constituent Systems,
/// and the edges connect various Systems.
template <typename T>
class HybridAutomaton : public System<T>,
                public detail::InputPortEvaluatorInterface<T> {
 public:
  typedef typename std::pair<const System<T>*, int> ModeIdentifier;

  ~HybridAutomaton() override {}

  /// Returns the list of contained Systems.
  std::vector<const systems::System<T>*> GetSystems() const {
    std::vector<const systems::System<T>*> result;
    result.reserve(registered_systems_.size());
    for (const auto& system : registered_systems_) {
      result.push_back(system.get());
    }
    return result;
  }

  /// Returns true if any output of the HybridAutomaton might have direct-
  /// feedthrough from any input of the HybridAutomaton.
  bool has_any_direct_feedthrough() const override {
    // For each output, see whether it has direct feedthrough all the way back
    // to any input.

    // TODO: re-visit this...
    for (const auto& output_port_id : output_port_ids_) {
      if (HasDirectFeedthroughFromAnyInput(output_port_id)) {
        return true;
      }
    }
    return false;
  }

  std::unique_ptr<Context<T>> CreateDefaultContext() const override {
    const int num_systems = static_cast<int>(sorted_systems_.size());
    // Reserve inputs as specified during HybridAutomaton initialization.
    auto context = std::make_unique<HybridAutomatonContext<T>>(num_systems);

    // Add each constituent system to the Context.
    for (int i = 0; i < num_systems; ++i) {
      const System<T>* const sys = sorted_systems_[i];
      auto subcontext = sys->CreateDefaultContext();
      auto suboutput = sys->AllocateOutput(*subcontext);
      context->AddSystem(i, std::move(subcontext), std::move(suboutput));
    }

    // Wire up the HybridAutomaton-internal inputs and outputs.
    for (const auto& connection : dependency_graph_) {
      const ModeIdentifier& src = connection.second;
      const ModeIdentifier& dest = connection.first;
      context->Connect(ConvertToContextPortIdentifier(src),
                       ConvertToContextPortIdentifier(dest));
    }

    // Declare the HybridAutomaton-external inputs.
    for (const PortIdentifier& id : input_port_ids_) {
      context->ExportInput(ConvertToContextPortIdentifier(id));
    }

    context->MakeState();
    return std::unique_ptr<Context<T>>(context.release());
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const override {

  }

  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override {

  }

  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    std::vector<std::unique_ptr<ContinuousState<T>>> sub_derivatives;
    for (const System<T>* const system : sorted_systems_) {

    }

  void EvalTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const override {

  }

 protected:
  /// Constructs an uninitialized HybridAutomaton. Subclasses that use this
  /// constructor are obligated to call HybridAutomatonBuilder::BuildInto(this).
  HybridAutomaton() {}

  void DoPublish(const Context<T>& context) const override {

  }

 private:
  // A structural outline of a HybridAutomaton, produced by
  // HybridAutomatonBuilder.
  struct StateMachine {
    // The ordered subsystem ports that are inputs to the entire hybrid
    // automaton.
    std::vector<ModeIdentifier> modal_subsystem_id;
    // The ordered subsystem ports that are outputs of the entire hybrid
    // automaton.
    std::vector<ModeIdentifier> output_port_ids;
    // A map from the input ports of constituent systems to the output ports
    // on which they depend. This graph is possibly cyclic, but must not
    // contain an algebraic loop.
    std::map<ModeIdentifier, ModeIdentifier> dependency_graph;
    // A list of the systems in the dependency graph in a valid, sorted
    // execution order, such that if EvalOutput is called on each system in
    // succession, every system will have valid inputs by the time its turn
    // comes.
    std::vector<const System<T>*> sorted_systems;
  };

  // Constructs a HybridAutomaton from the StateMachine that a
  // HybridAutomatonBuilder produces.
  // This constructor is private because only HybridAutomatonBuilder calls it.
  explicit HybridAutomaton(const StateMachine& state_machine) {
    Initialize(state_machine);
  }

  // Validates the given @p state_machine and sets up the HybridAutomaton
  //accordingly.
  void Initialize(const StateMachine& state_machine) {
    // The HybridAutomaton must not already be initialized.
    DRAKE_DEMAND(sorted_systems_.empty());
    // The initialization must be nontrivial.
    DRAKE_DEMAND(!state_machine.sorted_systems.empty());

    // Copy the data from the state_machine into private member variables.
    dependency_graph_ = state_machine.dependency_graph;
    sorted_systems_ = state_machine.sorted_systems;
    input_port_ids_ = state_machine.input_port_ids;
    output_port_ids_ = state_machine.output_port_ids;

    // Generate a map from the System pointer to its index in the sort order.
    for (int i = 0; i < static_cast<int>(sorted_systems_.size()); ++i) {
      sorted_systems_map_[sorted_systems_[i]] = i;
    }

    // Every system must appear in the sort order exactly once.
    DRAKE_DEMAND(sorted_systems_.size() == sorted_systems_map_.size());
    // Every port named in the dependency_graph_ must actually exist.
    DRAKE_ASSERT(PortsAreValid());
    // The sort order must square with the dependency_graph_.
    DRAKE_ASSERT(SortOrderIsCorrect());

    // Add the inputs to the HybridAutomaton topology, and check their
    // invariants.
    for (const PortIdentifier& id : input_port_ids_) {
      ExportInput(id);
    }
    for (const PortIdentifier& id : output_port_ids_) {
      ExportOutput(id);
    }
  }

  // Takes ownership of the @p registered_systems from HybridAutomatonBuilder.
  void Own(std::vector<std::unique_ptr<System<T>>> registered_systems) {
    // We must be given something to own.
    DRAKE_DEMAND(!registered_systems.empty());
    // We must not already own any subsystems.
    DRAKE_DEMAND(registered_systems_.empty());
    // The subsystems we are being given to own must be exactly the set of
    // subsystems for which we have an execution order.
    DRAKE_DEMAND(registered_systems.size() == sorted_systems_.size());
    for (const auto& system : registered_systems) {
      const auto it = sorted_systems_map_.find(system.get());
      DRAKE_DEMAND(it != sorted_systems_map_.end());
    }
    // All of those checks having passed, take ownership of the subsystems.
    registered_systems_ = std::move(registered_systems);
    // Inform the constituent system it's bound to this HybridAutomaton.
    for (auto& system : registered_systems_) {
      system->set_parent(this);
    }
  }

  // Returns the index of the given @p sys in the sorted order of this hybrid
  // automaton, or aborts if @p sys is not a member of the hybrid automaton.
  int GetSystemIndexOrAbort(const System<T>* sys) const {
    auto it = sorted_systems_map_.find(sys);
    DRAKE_DEMAND(it != sorted_systems_map_.end());
    return it->second;
  }

  // HybridAutomaton objects are neither copyable nor moveable.
  HybridAutomaton(const HybridAutomaton<T>& other) = delete;
  HybridAutomaton& operator=(const HybridAutomaton<T>& other) = delete;
  HybridAutomaton(HybridAutomaton<T>&& other) = delete;
  HybridAutomaton& operator=(HybridAutomaton<T>&& other) = delete;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<ModeIdentifier, ModeIdentifier> dependency_graph_;

  // The Systems in this HybridAutomaton, which are owned by this
  // HybridAutomaton, in the order they were registered.
  std::vector<std::unique_ptr<System<T>>> registered_systems_;

  // TODO(jadecastro): Throw some exception if the number of outputs do not
  // completely align among subsystems. Also require that their names also
  // match?

};

}  // namespace systems
}  // namespace drake
