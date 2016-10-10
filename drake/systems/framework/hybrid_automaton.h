#pragma once

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/hybrid_context.h"

namespace drake {
namespace systems {

/// NB: This implents a DTHS which is the easier case to simulate, that is, it
/// doesn't require any machinery above and beyond that needed for DT systems.

/// HybridAutomaton is a System composed of one or more constituent Systems,
/// arranged in a directed graph where the vertices are the constituent Systems,
/// and the edges connect various Systems.
template <typename T>
class HybridAutomaton : public System<T> {
 public:
  typedef int ModeId;
  typedef typename std::pair<std::unique_ptr<System<T>>, ModeId> ModalSubsystem;
  typedef typename std::pair<const ModalSubsystem<T>*,
    const ModalSubsystem<T>*> ModalSubsystemPair;

  ~HybridAutomaton() override {}

  /// Returns true if any output of the HybridAutomaton might have direct-
  /// feedthrough from any input.
  bool has_any_direct_feedthrough() const override {
    // For each system, see whether it has direct feedthrough.
    return false;
  }

  // NB: the idea is to store the contexts for each of the individual systems
  // and then rationalize about them at runtime, and when writing down problem
  // instances.  Uses diagram_context as is.
  // TODO: We'll need to store all the contexts from the system, and then
  //    multiplex them according to the discrete mode (also part of the
  //    context).
  //    ... Since we need to add discrete mode to the context, we will need
  //    a different hybrid_context as well (though it might derive from
  //    diagram_context).
  // TODO: modify PerformReset.
  // TODO: reset should check whether or not the pre/post contexts are
  // consistent.
  std::unique_ptr<Context<T>> CreateDefaultContext() const override {
    const int num_subsystems = static_cast<int>(modes_.size());

    // Reserve inputs as specified during HybridAutomaton initialization.
    auto context = std::make_unique<HybridContext<T>>(num_subsystems);

    // Add each constituent system to the Context.
    for (int i = 0; i < num_subsystems; ++i) {
      const System<T>* const sys = sorted_systems_[i];
      auto subcontext = sys->CreateDefaultContext();
      auto suboutput = sys->AllocateOutput(*subcontext);
      context->AddSystem(i, std::move(subcontext), std::move(suboutput));
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
      sub_derivatives.push_back(system->AllocateTimeDerivatives());
    }
    return std::unique_ptr<ContinuousState<T>>(sub_derivatives);
  }

  void EvalTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const override {
    auto diagram_context = dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    auto diagram_derivatives =
        dynamic_cast<DiagramContinuousState<T>*>(derivatives);
    DRAKE_DEMAND(diagram_derivatives != nullptr);
    const int n = diagram_derivatives->get_num_substates();
    DRAKE_DEMAND(static_cast<int>(sorted_systems_.size()) == n);

    // Evaluate the derivatives of each constituent system.
    for (int i = 0; i < n; ++i) {
      const Context<T>* subcontext = diagram_context->GetSubsystemContext(i);
      ContinuousState<T>* subderivatives =
          diagram_derivatives->get_mutable_substate(i);
      sorted_systems_[i]->EvalTimeDerivatives(*subcontext, subderivatives);
    }
  }

  // Assign an invariant set of states to a given mode.
  // (default is the infinite state space)
  void HybridAutomaton::AddInvariant(const Mode& mode) {

  }

  // Assign a set of initial conditions to a given mode.
  // (default is the infinite state space).
  void HybridAutomaton::AddInitialCondition(const Mode& mode) {

  }

  // Add a guard condition (a symbolic expression over context) to a given edge.
  void HybridAutomaton::AddGuard(const Edge& edge) {

  }

  // Add a reset map (a transformation acting on the context) to a given edge.
  void HybridAutomaton::AddReset(const Edge& edge) {

  }

  // Evaluates the value of the guard condition at a particular context.
  virtual T HybridAutomaton::EvalGuard(const ModeTransition& mode_trans,
                                       const Context& context) {

  }

  // Evaluates the reset function, instantly changing the context.
  virtual void HybridAutomaton::PerformReset(const Edge& edge,
                                             Context* context) {

  }

  // TODO: add a getter to expose the whole automaton to anyone interested?

 protected:
  /// Constructs an uninitialized HybridAutomaton. Subclasses that use this
  /// constructor are obligated to call HybridAutomatonBuilder::BuildInto(this).
  HybridAutomaton() {}

  void DoPublish(const Context<T>& context) const override {

  }

 private:
  // TODO: check -- is this all that's needed to register a difference variable?
  template <typename T>
  std::unique_ptr<DifferenceState<T>> AllocateDifferenceState() const {
    // Introduce a difference state defining the discrete mode that is active
    // at any given moment.
    std::vector<std::unique_ptr<BasicVector<T>>> xd;
    // The discrete mode is a singleton; we allocate a size-one vector here.
    xd.push_back(std::make_unique<BasicVector<T>>(1));
    return std::make_unique<DifferenceState<T>>(std::move(xd));
  }

  // A structural outline of a HybridAutomaton, produced by
  // HybridAutomatonBuilder.
  struct ModeTransition {
    ModalSubsystemPair<T> modal_subsystem_pair;
    std::vector<SymbolicExpression<T>*> guard;
    std::vector<Context<T>> reset;
    // TODO: We should no longer be mutating a single context, as was done
    // previously.
  };

  struct StateMachine {
    // The modes for the entire hybrid automaton.
    // TODO: do we even need to store the modes??
    // TODO: should this be a map? If so, we lose the label (number) for each
    // instantiated system. If not, we can't use keys.
    std::vector<ModalSubsystem> modes;
    // A map from the input ports of constituent systems to the output ports
    // on which they depend. This graph is possibly cyclic, but must not
    // contain an algebraic loop.
    // NB: (from note above) here it seems to make more sense as a map.
    std::map<ModeTransition, int> mode_transitions;
  };

  // Constructs a HybridAutomaton from the StateMachine that a
  // HybridAutomatonBuilder produces.
  // This constructor is private because only HybridAutomatonBuilder calls it.
  explicit HybridAutomaton(const StateMachine& state_machine) {
    Initialize(state_machine);
  }

  // Validates the given @p state_machine and sets up the HybridAutomaton
  // accordingly.
  void Initialize(const StateMachine& state_machine) {
    // The HybridAutomaton must be noninitialized and nonempty.
    DRAKE_DEMAND(sorted_systems_.empty());
    DRAKE_DEMAND(!state_machine.sorted_systems.empty());

    // Copy the data from the state_machine into private member variables.
    auto mode_transitions_ = state_machine.mode_transitions;
    auto modes_ = state_machine.modes;

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

  // Returns the index of the given @p sys in the sorted order of this hybrid
  // automaton, or aborts if @p sys is not a member of the hybrid automaton.
  int GetSystemIndexOrAbort(std::unique_ptr<System<T>> sys) const {
    auto it = state_machine.modes.find(sys);
    DRAKE_DEMAND(it != state_machine.modes.end());
    return it->second;
  }

  // HybridAutomaton objects are neither copyable nor moveable.
  HybridAutomaton(const HybridAutomaton<T>& other) = delete;
  HybridAutomaton& operator=(const HybridAutomaton<T>& other) = delete;
  HybridAutomaton(HybridAutomaton<T>&& other) = delete;
  HybridAutomaton& operator=(HybridAutomaton<T>&& other) = delete;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<ModeId, ModeId> mode_transitions_;

  // The Systems in this HybridAutomaton, which are owned by this
  // HybridAutomaton, in the order they were registered.
  std::vector<std::unique_ptr<ModalSubsystem<T>>> modal_subsystems_;

  // TODO(jadecastro): Throw some exception if the number of outputs do not
  // completely align among subsystems. Also require that their names also
  // match?

};

}  // namespace systems
}  // namespace drake
