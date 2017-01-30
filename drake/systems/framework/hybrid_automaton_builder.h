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
using std::shared_ptr;

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
/// A ModalSubsystem must be added to the HybridAutomatonBuilder via
/// AddModalSubsystem, then ModeTransitions may then be added to define the
/// transitionable subsystems.  
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
  /// @tparam S The type of system to add.
  template <template <typename Scalar> class S>
  ModalSubsystem<T> AddModalSubsystem(
      unique_ptr<S<T>> system, std::vector<PortId>& inport_ids,
      std::vector<PortId>& outport_ids, const ModeId mode_id) {
    //DRAKE_DEMAND(system != nullptr);

    for (auto mss : modal_subsystems_) {
      // Throw if the proposed mode_id exists.
      // TODO(jadecastro): Make use of find.
      DRAKE_ASSERT(mss->get_mode_id() != mode_id);
    }

    // Fail if the system is stateless or if the inputs and output traits
    // disagree.
    auto context0 = system->CreateDefaultContext();
    auto output0 = system->AllocateOutput(*context0);
    DRAKE_DEMAND(!context0->is_stateless());

    if (modal_subsystems_.size() > 0) {
      DRAKE_DEMAND(modal_subsystems_[0]->get_num_input_ports() ==
                   static_cast<int>(inport_ids.size()));
      DRAKE_DEMAND(modal_subsystems_[0]->get_num_output_ports() ==
                   static_cast<int>(outport_ids.size()));
    }

    // Populate a ModalSubsystem
    ModalSubsystem<T> modal_subsystem =
        ModalSubsystem<T>(mode_id, shared_ptr<System<T>>(std::move(system)),
                          inport_ids, outport_ids);
    modal_subsystems_.emplace_back(&modal_subsystem);

    return modal_subsystem;
  }

  /// Implicitly sets the input and output ports to system defaults.
  template <template <typename Scalar> class S>
  ModalSubsystem<T> AddModalSubsystem(unique_ptr<S<T>> system,
                                      const ModeId mode_id) {
    //DRAKE_DEMAND(system != nullptr);

    for (auto mss : modal_subsystems_) {
      // Throw if the proposed mode_id exists.
      // TODO(jadecastro): Make use of find.
      DRAKE_ASSERT(mss->get_mode_id() != mode_id);
    }

    // Fail if the system is stateless or if the inputs and output traits
    // disagree.
    auto context0 = system->CreateDefaultContext();
    auto output0 = system->AllocateOutput(*context0);
    DRAKE_DEMAND(!context0->is_stateless());

    if (modal_subsystems_.size() > 0) {
      auto& system1 = modal_subsystems_.front->get_system();
      auto context1 = system1->CreateDefaultContext();
      auto output1 = system1->AllocateOutput(*context1);
      DRAKE_DEMAND(modal_subsystems_[0]->get_num_input_ports() ==
                   context1->get_num_input_ports());
      DRAKE_DEMAND(modal_subsystems_[0]->get_num_output_ports() ==
                   output1->get_num_ports());
    }

    // Populate a ModalSubsystem
    ModalSubsystem<T> modal_subsystem =
        ModalSubsystem<T>(mode_id, shared_ptr<System<T>>(std::move(system)));
    modal_subsystems_.emplace_back(&modal_subsystem);

    return modal_subsystem;
  }

  ModeTransition<T> AddModeTransition(ModalSubsystem<T>& sys_pre,
                                      ModalSubsystem<T>& sys_post) {
    // TODO(jadecastro): Throw if pre and post have disjoint invariants.
    std::pair<ModeId, ModeId> edge = std::make_pair(sys_pre.get_mode_id(),
                                                    sys_post.get_mode_id());

    ModeTransition<T> mode_transition = ModeTransition<T>(edge);
    //mode_transitions_.insert(std::make_pair(mode_transitions_.size(),
    //                                        &mode_transition));
    mode_transitions_.push_back(&mode_transition);

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
  //std::multimap<int, ModeTransition<T>*> get_mode_transitions() const {
  //  return mode_transitions_;
  //};

  // Adds an invariant formula for the specified ModalSubsystem.
  void AddInvariant(ModalSubsystem<T>* modal_subsystem,
                    const symbolic::Expression& invariant) const {
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
                           const symbolic::Expression& init) const {
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
                const symbolic::Expression& guard) const {
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
  // to be supplied as a vector of appropriate dimension.  An empty Expression
  // corresponds to the identity mapping.
  void AddReset(ModeTransition<T>* mode_transition,
                std::vector<symbolic::Expression>& reset) {

    // TODO(jadecastro): Throw if non-trivial resets are not algebraic. Do
    // something like AddConstraint in MathematicalProgram?
    if (reset.size() == 0) {
      // Ensure that the continuous dimensions all match.
      const ModalSubsystem<T>* mss_pre =
          modal_subsystems_[mode_transition->get_predecessor()];
      unique_ptr<Context<T>> context_pre =
          mss_pre->get_system()->CreateDefaultContext();
      const ModalSubsystem<T>* mss_post =
          modal_subsystems_[mode_transition->get_successor()];
      unique_ptr<Context<T>> context_post =
          mss_post->get_system()->CreateDefaultContext();
      const ContinuousState<T>& xc_pre = *context_pre->get_continuous_state();
      const ContinuousState<T>& xc_post = *context_post->get_continuous_state();
      DRAKE_DEMAND(xc_pre.get_generalized_position().size() ==
                   xc_post.get_generalized_position().size());
      DRAKE_DEMAND(xc_pre.get_generalized_velocity().size() ==
                   xc_post.get_generalized_velocity().size());
      DRAKE_DEMAND(xc_pre.get_misc_continuous_state().size() ==
                   xc_post.get_misc_continuous_state().size());
      int num_xd_pre = context_pre->get_num_discrete_state_groups();
      int num_xd_post = context_post->get_num_discrete_state_groups();
      DRAKE_DEMAND(num_xd_pre == num_xd_post);
    }

    mode_transition->set_reset_throw_if_incompatible(reset);
  }

  // Adds the identity reset (empty vector) to the specified ModeTransition.
  // TODO(jadecastro): How do we ensure that the user either calls AddReset or
  // AddIdentityReset?
  void AddIdentityReset(ModeTransition<T>* mode_transition) {
    std::vector<symbolic::Expression> reset;
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
    //Finalize();
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
    //Finalize();
    target->Initialize(Compile());
  }

 private:

  // void ThrowIfSystemNotRegistered(const ModalSubsystem<T>* modal_subsystem)
  //     const {
  //   DRAKE_THROW_UNLESS(
  //       modal_subsystems_.find(modal_subsystem) != modal_subsystems_.end());
  // }

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
    for (auto& mt : mode_transitions_) {
      state_machine.mode_transitions.emplace_back(mt);
    }
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
  std::vector<ModeTransition<T>*> mode_transitions_;
  //std::multimap<int, ModeTransition<T>*> mode_transitions_;
  std::set<ModeId> initial_modes_;

  // Fixed input/output port dimensions for the HA.
  ModeId mode_id_init_{0};
  int num_inports_;
  int num_outports_;
};

}  // namespace systems
}  // namespace drake
