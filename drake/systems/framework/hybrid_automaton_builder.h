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

using std::unique_ptr;

  // TODO(jadecastro): Prune modal subsystem list if any are isolated from the
  // tree.
  //  - First pass: discard any modes that are either not initial nor immediate
  // successors.
  //  - Second pass: do a BFS to detect any that are not reachable from the
  // initial modes.

  // TODO(jadecastro): Remove any transitions whose guards are disjoint from the
  // intersection of the invariants of the pre and post modes.

  // TODO(jadecastro): Verify that formulas given are valid.

  // TODO(jadecastro): Verify the ports.

  // TODO(jadecastro): Verify the resets given will yield unique initial
  // conditions following the mode transition event.  Should live in
  // AddModeTransition.


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
  typedef int PortId;

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
  template <template <typename Scalar> class S>
  ModalSubsystem<T> AddModalSubsystem(
      unique_ptr<S<T>> system, std::vector<PortId>& inport_ids,
      std::vector<PortId>& outport_ids, const ModeId mode_id) {
    // Initialize the invariant to True.

    DRAKE_DEMAND(system != nullptr);

    for (auto mss : modal_subsystems_) {
      // Throw if the proposed mode_id exists.
      DRAKE_ASSERT(mss->get_mode_id() != mode_id);
    }
    // TODO(jadecastro): Is std::vector the best data container?  Ultimately
    // want set operations on elements (e.g. unions, intersections).

    // TODO(jadecastro): Make sure that the variables used are consistent with
    // the underlying continuous state.
    std::vector<symbolic::Formula> invariant;

    // Initialize the intial conditions to True.
    std::vector<symbolic::Formula> init;

    DRAKE_DEMAND(system.get() != nullptr);
    // Populate a ModalSubsystem
    ModalSubsystem<T> modal_subsystem =
        ModalSubsystem<T>(mode_id, system.get(), invariant, init,
                          inport_ids, outport_ids);
    modal_subsystems_.emplace_back(&modal_subsystem);

    return modal_subsystem;
  }

  ModeTransition<T> AddModeTransition(ModalSubsystem<T>& sys_pre,
                                      ModalSubsystem<T>& sys_post) {
    // TODO(jadecastro): Throw if pre and post have disjoint invariants.
    std::pair<ModalSubsystem<T>*, ModalSubsystem<T>*> edge =
        std::make_pair(&sys_pre, &sys_post);
    // Define an empty guard.
    std::vector<symbolic::Formula> guard;
    // Define an empty reset mapping.
    std::vector<symbolic::Formula> reset;

    ModeTransition<T> mode_transition = ModeTransition<T>(edge, guard, reset);
    mode_transitions_.insert(std::make_pair(mode_transitions_.size(),
                                            &mode_transition));

    return mode_transition;
  }

  // TODO(jadecastro): These three functions are kinda dumb.
  void set_mode_id_init(const ModeId mode_id) { mode_id_init_ = mode_id; }
  void set_num_expected_input_ports(const int num_inports) {
    num_inports_ = num_inports;
  }
  void set_num_expected_output_ports(const int num_outports) {
    num_outports_ = num_outports;
  }

  // A helper for creating transitions with a self-loop.
  ModeTransition<T> AddModeTransition(ModalSubsystem<T>& sys) {
    return this->AddModeTransition(sys, sys);
  }

  // Getter for the ordered list of mode transitions.
  std::multimap<ModeId, ModeTransition<T>*> get_mode_transitions() const {
    return mode_transitions_;
  };

  // Adds an invariant formula for the specified ModalSubsystem.
  void AddInvariant(ModalSubsystem<T>* modal_subsystem,
                    const symbolic::Formula& invariant) const {
    DRAKE_ASSERT(modal_subsystem != nullptr);
    // TODO: validate, like in context.
    // DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(*context));

    (*modal_subsystem->get_mutable_invariant()).push_back(invariant);

    // Check that the formula doesn't falsify the conjunction of the
    // expressions.
    // TODO(jadecastro): Implement this check.
    // bool result = EvalInvariant();
    // DRAKE_DEMAND(!result);
  }

  // Adds an initial condition formula for the specified ModalSubsystem.
  void AddInitialCondition(ModalSubsystem<T>* modal_subsystem,
                            const symbolic::Formula& init) const {
    DRAKE_ASSERT(modal_subsystem != nullptr);
    // TODO: validate, like in context.
    // DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(*context));

    (*modal_subsystem->get_mutable_initial_conditions()).push_back(init);

    // Check that the formula doesn't falsify the conjunction of the
    // expressions.
    // TODO(jadecastro): Implement this check.
    // bool result = EvalInvariant();
    // DRAKE_DEMAND(!result);
  }

  // Adds a guard formula for the specified ModeTransition.
  void AddGuard(ModeTransition<T>* mode_transition,
                const symbolic::Formula& guard) const {
    DRAKE_ASSERT(mode_transition != nullptr);
    // TODO: validate, like in context.
    // DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(*context));

    // Define a pointer to the continuous state in the context.
    (*mode_transition->get_mutable_guard()).push_back(guard);

    // Check that the formula doesn't falsify the conjunction of the guard
    // expression.
    // TODO(jadecastro): Implement this check.
    // bool result = EvalInvariant();
    // DRAKE_DEMAND(!result);
  }

  // Adds a vector of reset formulas to the specified ModeTransition. Because
  // alignment with the continuous state vector is important, we require reset
  // to be supplied as a vector of appropriate dimension.
  void AddReset(ModeTransition<T>* mode_transition,
                std::vector<symbolic::Formula>& reset) {
    // NB: symbolic::Formula::True() implements a 'fast' version of the identity
    // mapping.

    // TODO(jadecastro): Throw if non-trivial resets are not algebraic. Do
    // something like AddConstraint in MathematicalProgram?
    if (reset[0].EqualTo(symbolic::Formula::True())) {
      // Ensure that the continuous dimensions all match.
      unique_ptr<Context<T>> context_pre =
          mode_transition->get_predecessor()->get_system()->
          CreateDefaultContext();
      unique_ptr<Context<T>> context_post =
          mode_transition->get_predecessor()->get_system()->
          CreateDefaultContext();
      const ContinuousState<T>& xc_pre = *context_pre->get_continuous_state();
      const ContinuousState<T>& xc_post = *context_post->get_continuous_state();
      DRAKE_DEMAND(xc_pre.get_generalized_position().size() ==
                   xc_post.get_generalized_position().size());
      DRAKE_DEMAND(xc_pre.get_generalized_velocity().size() ==
                   xc_post.get_generalized_velocity().size());
      DRAKE_DEMAND(xc_pre.get_misc_continuous_state().size() ==
                   xc_post.get_misc_continuous_state().size());
    }

    mode_transition->set_reset_throw_if_incompatible(reset);
  }

  // Adds the identity reset to the specified ModeTransition.
  void AddReset(ModeTransition<T>* mode_transition) {
    std::vector<symbolic::Formula> reset{symbolic::Formula::True()};
    AddReset(mode_transition, reset);
  }

  // Adds a set of initial modes to the HA, replacing everything there already.
  void AddInitialModes(std::set<ModeId> initial_modes) {
    DRAKE_DEMAND(initial_modes_.empty());
    DRAKE_DEMAND(!modal_subsystems_.empty());
    DRAKE_DEMAND(initial_modes.size() < modal_subsystems_.size());

    // Require that all elements are valid.
    // TODO(jadecastro): Do something other than exhaustive search here.
    for (auto mss : modal_subsystems_) {
      DRAKE_DEMAND(initial_modes.find(mss->get_mode_id()) !=
                   initial_modes.end());  // TODO: <--- Obviously wrong. Revisit
                                          // it.
    }

    initial_modes_ = initial_modes;
  }

  /// Returns the list of contained Systems.
  std::vector<systems::System<T>*> GetMutableSystems() {
    std::vector<systems::System<T>*> result;
    result.reserve(modal_subsystems_.size());
    for (const auto& mss : modal_subsystems_) {
      result.emplace_back(mss->get_system());
    }
    return result;
  }

  /// Builds the HybridAutomaton that has been described by the calls to
  /// Connect, ExportInput, and ExportOutput. Throws std::logic_error if the
  /// graph is not buildable.
  unique_ptr<HybridAutomaton<T>> Build() {
    // TODO(jadecastro): Need some extensive verification here.
    Finalize();
    unique_ptr<HybridAutomaton<T>> hybrid_automaton(
        new HybridAutomaton<T>());
    hybrid_automaton->Initialize(Compile());;
    return std::move(hybrid_automaton);
  }

  /// Configures @p target to have the topology that has been described by
  /// the calls to Connect, ExportInput, and ExportOutput. Throws
  /// std::logic_error if the graph is not buildable.
  ///
  /// Only HybridAutomaton subclasses should call this method. The target must
  /// not already be initialized.
  void BuildInto(HybridAutomaton<T>* target) {
    Finalize();
    target->Initialize(Compile());
  }

  // **********Call this something else!!
  void Finalize() {
    for (auto modal_subsystem : modal_subsystems_) {
      if (modal_subsystem->get_invariant().size() == 0) {
        auto invariant = modal_subsystem->get_mutable_invariant();
        (*invariant).emplace_back(symbolic::Formula::True());
      }
      if (modal_subsystem->get_initial_conditions().size() == 0) {
        auto init = modal_subsystem->get_mutable_initial_conditions();
        (*init).emplace_back(symbolic::Formula::True());
      }
    }
    for (auto it : mode_transitions_) {
      ModeTransition<T>* mode_transition = it.second;
      if (mode_transition->get_guard().size() == 0) {
        auto guard = mode_transition->get_mutable_guard();
        (*guard).emplace_back(symbolic::Formula::True());
      }
      // NB: We require resets to be explicitly specified by the user, since
      // context consistency checks are needed upon finalizing the HA.
    }
  }

 private:
  /*
  void ThrowIfSystemNotRegistered(const ModalSubsystem<T>* modal_subsystem)
      const {
    DRAKE_THROW_UNLESS(
        modal_subsystems_.find(modal_subsystem) != modal_subsystems_.end());
  }
  */

  /// Produces the state machine corresponding to the modal subsystems and mode
  /// transitions created using the HybridAutomatonBuilder.
  typename HybridAutomaton<T>::StateMachine Compile() const {
    if (modal_subsystems_.size() == 0 || mode_transitions_.size() == 0) {
      throw std::logic_error(
          "Cannot create an empty or unconnected HybridAutomatonBuilder.");
    }

    typename HybridAutomaton<T>::StateMachine state_machine;
    // TODO(jadecastro): The loop is here to deal with the const conversion. Is
    // there a cleaner way?
    for (auto& mss : modal_subsystems_) {
      state_machine.modal_subsystems.emplace_back(mss);
    }
    state_machine.mode_transitions = mode_transitions_;
    state_machine.initial_modes = initial_modes_;
    state_machine.mode_id_init = mode_id_init_;
    state_machine.num_inports = num_inports_;
    state_machine.num_outports = num_outports_;
    return state_machine;
  }

  // HybridAutomatonBuilder objects are neither copyable nor moveable.
  HybridAutomatonBuilder(const HybridAutomatonBuilder<T>& other) = delete;
  HybridAutomatonBuilder& operator=(const HybridAutomatonBuilder<T>& other) =
      delete;
  HybridAutomatonBuilder(HybridAutomatonBuilder<T>&& other) = delete;
  HybridAutomatonBuilder& operator=(HybridAutomatonBuilder<T>&& other) = delete;

  // TODO(jadecastro): map?
  std::vector<ModalSubsystem<T>*> modal_subsystems_;
  std::multimap<ModeId, ModeTransition<T>*> mode_transitions_;
  std::set<ModeId> initial_modes_;

  // Fixed input/output port dimensions for the HA.
  ModeId mode_id_init_{0};
  int num_inports_;
  int num_outports_;
};

}  // namespace systems
}  // namespace drake
