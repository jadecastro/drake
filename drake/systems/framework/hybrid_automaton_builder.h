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

/// HybridAutomatonBuilder is a factory class for HybridAutomaton. It collects
/// the dependency graph of constituent systems, and topologically sorts them.
/// It is single use: after calling Build or BuildInto, HybridAutomatonBuilder
/// gives up ownership of the constituent systems, and should therefore be
/// discarded.
///
/// A system must be added to the HybridAutomatonBuilder with AddSystem before
/// it can be wired up in any way.
///
/// TODO: determine when it is okay/not okay to use identity as the default
/// reset map. A dumb way would be to require the user to specify it when the
/// pre/post state dimensions match, but we should really be smarter about it.
template <typename T>
class HybridAutomatonBuilder {
 public:
  HybridAutomatonBuilder() {}
  virtual ~HybridAutomatonBuilder() {}

  // Add a mode to the automaton.
  //std::unique_ptr<> AddModalSubsystem(const System<T>& sys) const {
  //  state_machine.modes[size++] = sys;
  //}

  // Connect a "pre" mode to a "post" mode.
  //std::unique_ptr<> AddModeTransition(const ModalSubsystem<T>& pre,
  //                                    const ModalSubsystem<T>& post) const {
  //  const ModeId pre_id = pre.second;
  //  const ModeId post_id = post.second;
  //  mode_transition_[pre_id] = post_id;
  //  TODO: is the best ordering pre->post?
  //}

  /// Add a mode to the automaton. Returns a bare
  /// pointer to the System, which will remain valid for the lifetime of the
  /// HybridAutomaton built by this builder.
  ///
  /// @code
  ///   HybridAutomatonBuilder<T> builder;
  ///   auto foo = builder.AddSystem(std::make_unique<Foo<T>>());
  /// @endcode
  ///
  /// @tparam S The type of system to add.
  template<class S>
  std::unique_ptr<S> AddModalSubsystem(std::unique_ptr<S> sys) {
    state_machine.modes.insert(sys);
    return sys;  // TODO: does the smart pointer work?
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
  /// Note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   HybridAutomatonBuilder<T> builder;
  ///   auto foo = builder.template AddSystem<Foo<T>>("name", 3.14);
  /// @endcode
  ///
  /// @tparam S The type of System to construct. Must subclass System<T>.
  template<class S, typename... Args>
  std::unique_ptr<S> AddModalSubsystem(Args&&... args) {
    return AddSystem(std::make_unique<S>(std::forward<Args>(args)...));
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
  /// @tparam S A template for the type of System to construct. The template
  /// will be specialized on the scalar type T of this builder.
  template<template<typename Scalar> class S, typename... Args>
  unique_ptr::<S<T>> AddModalSubsystem(Args&&... args) {
    return AddSystem(std::make_unique<S<T>>(std::forward<Args>(args)...));
  }

  /// Add a mode to the automaton. Returns a bare
  /// pointer to the System, which will remain valid for the lifetime of the
  /// HybridAutomaton built by this builder.
  ///
  /// @code
  ///   HybridAutomatonBuilder<T> builder;
  ///   auto foo = builder.AddSystem(std::make_unique<Foo<T>>());
  /// @endcode
  ///
  /// @tparam S The type of system to add.
  template<class S>
  std::unique_ptr<S> AddModeTransition(std::unique_ptr<S1> pre,
                                       std::unique_ptr<S2> post) {
    const ModeId pre_id = pre.second;
    const ModeId post_id = post.second;
    return raw_sys_ptr;  // TODO: does the smart pointer work?
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
  /// Note that for dependent names you must use the template keyword:
  ///
  /// @code
  ///   HybridAutomatonBuilder<T> builder;
  ///   auto foo = builder.template AddSystem<Foo<T>>("name", 3.14);
  /// @endcode
  ///
  /// @tparam S The type of System to construct. Must subclass System<T>.
  template<class S, typename... Args>
  S* AddSystem(Args&&... args) {
    return AddModeTransition(std::make_unique<S>(std::forward<Args>(args)...));
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
  /// @tparam S A template for the type of System to construct. The template
  /// will be specialized on the scalar type T of this builder.
  template<template<typename Scalar> class S, typename... Args>
  S<T>* AddModeTransition(Args&&... args) {
    return AddSystem(std::make_unique<S<T>>(std::forward<Args>(args)...));
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

  /// Builds the HybridAutomaton that has been described by the calls to
  /// Connect, ExportInput, and ExportOutput. Throws std::logic_error if the
  /// graph is not buildable.
  std::unique_ptr<HybridAutomaton<T>> Build() {
    std::unique_ptr<HybridAutomaton<T>>
      diagram(new HybridAutomaton<T>(Compile()));
    diagram->Own(std::move(registered_systems_));
    return std::move(diagram);
  }

 private:
  /// Produces a state machine that has been described by the calls to
  /// Connect, ExportInput, and ExportOutput. Throws std::logic_error if the
  /// graph is not buildable.
  /*
  typename HybridAutomaton<T>::StateMachine Compile() const {
    if (modal_subsystems_.size() == 0) {
      throw std::logic_error("Cannot compile an empty HybridAutomatonBuilder.");
    }
    typename HybridAutomaton<T>::StateMachine state_machine;
    blueprint.mode_transitions = mode_transitions_;
    blueprint.modal_subsystems = modal_subsystems_;
    return state_machine;
  }
  */

  // HybridAutomatonBuilder objects are neither copyable nor moveable.
  HybridAutomatonBuilder(const HybridAutomatonBuilder<T>& other) = delete;
  HybridAutomatonBuilder& operator=(const HybridAutomatonBuilder<T>& other)
    = delete;
  HybridAutomatonBuilder(HybridAutomatonBuilder<T>&& other) = delete;
  HybridAutomatonBuilder& operator=(HybridAutomatonBuilder<T>&& other) = delete;

  // A map from the input ports of constituent systems, to the output ports of
  // the systems on which they depend.
  std::map<ModalSubsystemPair, int> mode_transitions_;

  // The Systems in this HybridAutomatonBuilder, in the order they were
  // registered.
  std::vector<std::unique_ptr<ModalSubsystem<T>>> modal_subsystems_;
};

}  // namespace systems
}  // namespace drake
