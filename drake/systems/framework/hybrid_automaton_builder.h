#pragma once

#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>
#include <iostream>  //needed?
#include <tuple>  //needed?

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/hybrid_automaton.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

  // TODO: Validate that the subsystem dimensions match wrt. the reset map,
  //       and that their inputs match.
  // TODO: Context stitching!! Also, time must be consistent across jumps.
  // TODO: Validate the state machine (all registered systems are connected).
  // Note: Simulator should be a secondary priority.

/// DiagramBuilder is a factory class for Diagram. It collects the dependency
/// graph of constituent systems, and topologically sorts them. It is single
/// use: after calling Build or BuildInto, DiagramBuilder gives up ownership
/// of the constituent systems, and should therefore be discarded.
///
/// A system must be added to the DiagramBuilder with AddSystem before it can
/// be wired up in any way.
template <typename T>
class HybridAutomatonBuilder {
 public:
  HybridAutomatonBuilder() {}
  virtual ~HybridAutomatonBuilder() {}

  // TODO: Is it okay for this to be public?
  typedef typename HybridAutomaton<T>::ModalSubsystem ModalSubsystem;
  typedef typename HybridAutomaton<T>::ModeTransition ModeTransition;

  /// Takes ownership of @p system and adds it to the builder. Returns a bare
  /// pointer to the System, which will remain valid for the lifetime of the
  /// Diagram built by this builder.
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto foo = builder.AddSystem(std::make_unique<Foo<T>>());
  /// @endcode
  ///
  /// @tparam S The type of system to add.
  template<class S>
  //std::unique_ptr<S> AddModalSubsystem(std::unique_ptr<S> sys) {
  ModalSubsystem* AddModalSubsystem(std::unique_ptr<S> sys) {
    // Initialize the invariant to True.
    // TODO: check if conjunctions can be done via std::vector.
    // TODO: Specialize the type for symbolic::Formula as a container
    // class to ensure that the variables used are consistent with the
    // underlying state vector for a given subsystem.
    std::vector<symbolic::Formula> invariant_function;
    invariant_function[0] = symbolic::Formula::True();
    // Initialize the intial conditions to True.
    std::vector<symbolic::Formula> init_function;
    init_function[0] = symbolic::Formula::True();
    // Populate a ModalSubsystem
    ModalSubsystem modal_subsystem
      = std::make_tuple(sys.get(), &invariant_function, &init_function, 0);
    modal_subsystems_->push_back(modal_subsystem);
    //return std::move(sys);  // TODO: How does ownership work here?
    return &modal_subsystem;
    // TODO: fix warnings^
  }
  /// Constructs a new system with the given @p args, and adds it to the
  /// builder, which retains ownership. Returns a bare pointer to the System,
  /// which will remain valid for the lifetime of the Diagram built by this
  /// builder.
  ///
  /// @code
  ///   DiagramBuilder<double> builder;
  ///   auto foo = builder.AddSystem<Foo<double>>("name", 3.14);
  /// @endcode
  ///
  /// note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto foo = builder.template AddSystem<Foo<T>>("name", 3.14);
  /// @endcode
  ///
  /// You may prefer the `unique_ptr` variant instead.
  ///
  ///
  /// @tparam S The type of System to construct. Must subclass System<T>.
  template<class S, typename... Args>
  std::unique_ptr<S> AddModalSubsystem(Args&&... args) {
    return AddModalSubsystem(std::make_unique<S>(std::forward<Args>(args)...));
  }

  /// Constructs a new system with the given @p args, and adds it to the
  /// builder, which retains ownership. Returns a bare pointer to the System,
  /// which will remain valid for the lifetime of the Diagram built by this
  /// builder.
  ///
  /// @code
  ///   DiagramBuilder<double> builder;
  ///   // Foo must be a template.
  ///   auto foo = builder.AddSystem<Foo>("name", 3.14);
  /// @endcode
  ///
  /// Note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   DiagramBuilder<T> builder;
  ///   auto foo = builder.template AddSystem<Foo>("name", 3.14);
  /// @endcode
  ///
  /// You may prefer the `unique_ptr` variant instead.
  ///
  /// @tparam S A template for the type of System to construct. The template
  /// will be specialized on the scalar type T of this builder.
  template<template<typename Scalar> class S, typename... Args>
  std::unique_ptr<S<T>> AddModalSubsystem(Args&&... args) {
    return AddModalSubsystem(std::make_unique<S<T>>(
                                         std::forward<Args>(args)...));
  }

  ModeTransition*
  AddModeTransition(ModalSubsystem& sys_pre, ModalSubsystem& sys_post) {
    // TODO: some validation checks.
    std::pair<ModalSubsystem*,ModalSubsystem*> pair;
    pair.first = &sys_pre;
    pair.second = &sys_post;
    std::vector<symbolic::Formula> guard;
    guard[0] = symbolic::Formula::True();
    // Initialize the reset map to True.
    //std::vector<symbolic::Formula> identity_reset;
    //true_reset[0] = symbolic::Formula::True();
    ModeTransition mode_transition = std::make_tuple(&pair, &guard);
    mode_transitions_->push_back(mode_transition);
    return &mode_transition;  // TODO: should we make better use of smart ptrs?
    // TODO: fix warnings^
  }

  // A helper for self-transitions.
  ModeTransition* AddModeTransition(ModalSubsystem& sys) {
    return this->AddModeTransition(sys, sys);
  }

  // Getter for the ordered list of mode transitions.
  std::vector<ModeTransition>& get_mode_transitions() const {
    return *mode_transitions_;
  };

  // TODO: We want something that will yield modalsubsys.PushBackInvariant(...).
  void AddInvariant(ModalSubsystem* modal_subsystem,
  symbolic::Formula& new_invariant) const {
    DRAKE_ASSERT(modal_subsystem != nullptr);
    // TODO: validate, like in context.
    //DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(*context));

    // Define a pointer to the continuous state in the context.
    // TODO: cleanup.
    auto mss = *modal_subsystem;
    auto mss1 = *std::get<1>(mss);
    mss1.push_back(new_invariant);
  }

  /// Returns the list of contained Systems.
  std::vector<systems::System<T>*> GetMutableSystems() {
    std::vector<systems::System<T>*> result;
    result.reserve(registered_systems_.size());
    for (const auto& system : registered_systems_) {
      result.push_back(system.get());
    }
    return result;
  }

  /// Builds the Diagram that has been described by the calls to Connect,
  /// ExportInput, and ExportOutput. Throws std::logic_error if the graph is
  /// not buildable.
  std::unique_ptr<HybridAutomaton<T>> Build() {
    std::unique_ptr<HybridAutomaton<T>>
      hybrid_automaton(new HybridAutomaton<T>(Compile()));
    hybrid_automaton->Own(std::move(registered_systems_));
    return std::move(hybrid_automaton);
  }

  /// Configures @p target to have the topology that has been described by
  /// the calls to Connect, ExportInput, and ExportOutput. Throws
  /// std::logic_error if the graph is not buildable.
  ///
  /// Only Diagram subclasses should call this method. The target must not
  /// already be initialized.
  void BuildInto(HybridAutomaton<T>* target) {
    target->Initialize(Compile());
    target->Own(std::move(registered_systems_));
  }

 private:
  // TODO: leaving these two commented out for now.
  //typedef typename HybridAutomaton<T>::ModalSubsystem ModalSubsystem;
  //typedef typename HybridAutomaton<T>::ModeTransition ModeTransition;
  typedef typename HybridAutomaton<T>::PortIdentifier PortIdentifier;

  void ThrowIfSystemNotRegistered(const System<T>* system) const {
    DRAKE_THROW_UNLESS(systems_.find(system) != systems_.end());
  }

  /// Produces the StateMachine that has been described by the calls to
  /// Connect, ExportInput, and ExportOutput. Throws std::logic_error if the
  /// graph is not buildable.
  typename HybridAutomaton<T>::StateMachine Compile() const {
    if (registered_systems_.size() == 0) {
      throw std::logic_error("Cannot Compile an empty DiagramBuilder.");
    }
    typename HybridAutomaton<T>::StateMachine state_machine;
    state_machine.input_port_ids = input_port_ids_;
    state_machine.output_port_ids = output_port_ids_;
    state_machine.modal_subsystems = *modal_subsystems_;
    state_machine.mode_transitions = *mode_transitions_;
    return state_machine;
  }

  // DiagramBuilder objects are neither copyable nor moveable.
  HybridAutomatonBuilder(const HybridAutomatonBuilder<T>& other) = delete;
  HybridAutomatonBuilder& operator=(const HybridAutomatonBuilder<T>& other)
    = delete;
  HybridAutomatonBuilder(HybridAutomatonBuilder<T>&& other) = delete;
  HybridAutomatonBuilder& operator=(HybridAutomatonBuilder<T>&& other) = delete;

  // The ordered inputs and outputs of the Diagram to be built.
  std::vector<PortIdentifier> input_port_ids_;
  std::vector<PortIdentifier> output_port_ids_;

  // For fast membership queries: has this input port already been declared?
  std::set<PortIdentifier> diagram_input_set_;

  // The unsorted set of Systems in this DiagramBuilder. Used for fast
  // membership queries.
  std::set<const System<T>*> systems_;
  // The Systems in this DiagramBuilder, in the order they were registered.
  std::vector<std::unique_ptr<System<T>>> registered_systems_;

  // TODO: map?
  std::unique_ptr<std::vector<ModalSubsystem>> modal_subsystems_;
  // TODO: map?
  std::unique_ptr<std::vector<ModeTransition>> mode_transitions_;
};

}  // namespace systems
}  // namespace drake
