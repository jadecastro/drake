#pragma once

#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/hybrid_automaton.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

  // TODO(jadecastro): Prune modal subsystem list if any are isolated from the
  // tree.
  //  - First pass: discard any modes that are either not initial nor immediate
  // successors.
  //  - Second pass: do a BFS to detect any that are not reachable from the
  // initial modes.

  // TODO(jadecastro): Verify that formulas given are valid.


/// HybridAutomatonBuilder is a factory class for HybridAutomaton. It collects
/// the dependency graph of constituent systems, and topologically sorts
/// them. It is single use: after calling Build or BuildInto,
/// HybridAutomatonBuilder gives up ownership of the constituent systems, and
/// should therefore be discarded.
///
/// A system must be added to the HybridAutomatonBuilder with AddSystem before
/// it can be wired up in any way.
template <typename T>
class HybridAutomatonBuilder {
 public:
  HybridAutomatonBuilder() {}
  virtual ~HybridAutomatonBuilder() {}

  typedef int ModeId;

  /// Takes ownership of @p system and adds it to the builder. Returns a bare
  /// pointer to the System, which will remain valid for the lifetime of the
  /// HybridAutomaton built by this builder.
  ///
  /// @code
  ///   HybridAutomatonBuilder<T> builder;
  ///   auto foo = builder.AddSystem(std::make_unique<Foo<T>>());
  /// @endcode
  ///
  /// @tparam S The type of system to add.
  template <class S>
      ModalSubsystem<S>* AddModalSubsystem(std::unique_ptr<S> system,
                                           const ModeId mode_id) {
    // Initialize the invariant to True.
    // TODO: check if conjunctions can be done via std::vector.
    // TODO: Specialize the type for symbolic::Formula as a container
    // class to ensure that the variables used are consistent with the
    // underlying state vector for a given subsystem.
    std::vector<symbolic::Formula> invariant;
    invariant.push_back(symbolic::Formula::True());
    // Initialize the intial conditions to True.
    std::vector<symbolic::Formula> init;
    init.push_back(symbolic::Formula::True());
    // Populate a ModalSubsystem
    ModalSubsystem<S> modal_subsystem =
        ModalSubsystem<S>(mode_id, system.get(), invariant, init);
    modal_subsystems_->emplace_back(modal_subsystem);
    // return std::move(sys);  // TODO: How does ownership work here?
    return &modal_subsystem;
    // TODO: fix warnings^
  }
  /// Constructs a new system with the given @p args, and adds it to the
  /// builder, which retains ownership. Returns a bare pointer to the System,
  /// which will remain valid for the lifetime of the HybridAutomaton built by
  /// this builder.
  ///
  /// @code
  ///   HybridAutomatonBuilder<double> builder;
  ///   auto foo = builder.AddSystem<Foo<double>>("name", 3.14);
  /// @endcode
  ///
  /// note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   HybridAutomatonBuilder<T> builder;
  ///   auto foo = builder.template AddSystem<Foo<T>>("name", 3.14);
  /// @endcode
  ///
  /// You may prefer the `unique_ptr` variant instead.
  ///
  /// @tparam S The type of System to construct. Must subclass System<T>.
  template <class S, typename... Args>
  std::unique_ptr<S> AddModalSubsystem(Args&&... args) {
    return AddModalSubsystem(std::make_unique<S>(std::forward<Args>(args)...));
  }

  /// Constructs a new system with the given @p args, and adds it to the
  /// builder, which retains ownership. Returns a bare pointer to the System,
  /// which will remain valid for the lifetime of the HybridAutomaton built by
  /// this builder.
  ///
  /// @code
  ///   HybridAutomatonBuilder<double> builder;
  ///   // Foo must be a template.
  ///   auto foo = builder.AddSystem<Foo>("name", 3.14);
  /// @endcode
  ///
  /// Note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   HybridAutomatonBuilder<T> builder;
  ///   auto foo = builder.template AddSystem<Foo>("name", 3.14);
  /// @endcode
  ///
  /// You may prefer the `unique_ptr` variant instead.
  ///
  /// @tparam S A template for the type of System to construct. The template
  /// will be specialized on the scalar type T of this builder.
  template <template <typename Scalar> class S, typename... Args>
  std::unique_ptr<S<T>> AddModalSubsystem(Args&&... args) {
    return AddModalSubsystem(
        std::make_unique<S<T>>(std::forward<Args>(args)...));
  }

  ModeTransition<T> AddModeTransition(ModalSubsystem<T>& sys_pre,
                                       ModalSubsystem<T>& sys_post) {
    // TODO: some validation checks.
    std::pair<ModalSubsystem<T>*, ModalSubsystem<T>*> edge;
    edge.first = &sys_pre;
    edge.second = &sys_post;
    // Define the default guard.
    std::vector<symbolic::Formula> guard;
    guard.push_back(symbolic::Formula::True());
    // Define the default reset map.
    std::vector<symbolic::Formula> reset;
    reset.push_back(symbolic::Formula::True());
    // TODO(jadecastro): Initialize the reset map to True.
    // std::vector<symbolic::Formula> identity_reset;
    ModeTransition<T> mode_transition =
        ModeTransition<T>(edge, &guard, &reset);
    mode_transitions_.push_back(&mode_transition);
    return mode_transition;
  }

  // A helper for self-transitions.
  ModeTransition<T> AddModeTransition(ModalSubsystem<T>& sys) {
    return this->AddModeTransition(sys, sys);
  }

  // Getter for the ordered list of mode transitions.
  std::vector<ModeTransition<T>*> get_mode_transitions() const {
    return mode_transitions_;
  };

  // TODO: We want something that will yield modalsubsys.PushBackInvariant(...).
  void AddInvariant(ModalSubsystem<T>* modal_subsystem,
                    symbolic::Formula& new_invariant) const {
    DRAKE_ASSERT(modal_subsystem != nullptr);
    // TODO: validate, like in context.
    // DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(*context));

    // Define a pointer to the continuous state in the context.
    // TODO: cleanup.
    (*modal_subsystem->get_mutable_invariant()).push_back(new_invariant);
  }

  /// Returns the list of contained Systems.
  std::vector<systems::System<T>*> GetMutableSystems() {
    std::vector<systems::System<T>*> result;
    result.reserve(modal_subsystems_.size());
    for (const auto& mss : modal_subsystems_) {
      result.push_back(mss->get_system());
    }
    return result;
  }

  /// Builds the HybridAutomaton that has been described by the calls to
  /// Connect, ExportInput, and ExportOutput. Throws std::logic_error if the
  /// graph is not buildable.
  std::unique_ptr<HybridAutomaton<T>> Build() {
    std::unique_ptr<HybridAutomaton<T>> hybrid_automaton(
        new HybridAutomaton<T>(Compile()));
    hybrid_automaton->DumpInto(modal_subsystems_, mode_transitions_);
    return std::move(hybrid_automaton);
  }

  /// Configures @p target to have the topology that has been described by
  /// the calls to Connect, ExportInput, and ExportOutput. Throws
  /// std::logic_error if the graph is not buildable.
  ///
  /// Only HybridAutomaton subclasses should call this method. The target must
  /// not already be initialized.
  void BuildInto(HybridAutomaton<T>* target) {
    target->Initialize(Compile());
    target->DumpInto(modal_subsystems_, mode_transitions_);
  }

 private:
  /*
  void ThrowIfSystemNotRegistered(const ModalSubsystem<T>* modal_subsystem)
      const {
    DRAKE_THROW_UNLESS(
        modal_subsystems_.find(modal_subsystem) != modal_subsystems_.end());
  }
  */

  /// Produces the StateMachine that has been described by the calls to
  /// Connect, ExportInput, and ExportOutput. Throws std::logic_error if the
  /// graph is not buildable.
  typename HybridAutomaton<T>::StateMachine Compile() const {
    if (modal_subsystems_.size() == 0) {
      throw std::logic_error("Cannot compile an empty HybridAutomatonBuilder.");
    }
    typename HybridAutomaton<T>::StateMachine state_machine;
    state_machine.modal_subsystems = modal_subsystems_;
    state_machine.mode_transitions = mode_transitions_;
    return state_machine;
  }

  // HybridAutomatonBuilder objects are neither copyable nor moveable.
  HybridAutomatonBuilder(const HybridAutomatonBuilder<T>& other) = delete;
  HybridAutomatonBuilder& operator=(const HybridAutomatonBuilder<T>& other) =
      delete;
  HybridAutomatonBuilder(HybridAutomatonBuilder<T>&& other) = delete;
  HybridAutomatonBuilder& operator=(HybridAutomatonBuilder<T>&& other) = delete;

  // TODO(jadecastro): map or multimap?
  std::vector<ModalSubsystem<T>*> modal_subsystems_;
  std::vector<ModeTransition<T>*> mode_transitions_;
};

}  // namespace systems
}  // namespace drake
