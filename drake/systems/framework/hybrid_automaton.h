#pragma once

// TODO: triage this list.
#include <algorithm>
#include <functional>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>
#include <fstream>

// TODO: triage this list.
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/abstract_state.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/hybrid_automaton_context.h"
//#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

template <typename T>
class HybridAutomatonBuilder;

namespace internal {


  // TODO: the following two should be removed.

/// DiagramOutput is an implementation of SystemOutput that holds
/// unowned OutputPort pointers. It is used to expose the outputs of
/// constituent systems as outputs of a Diagram.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
class HybridAutomatonOutput: public SystemOutput<T> {
 public:

  // Required virtual member functions.
  int get_num_ports() const override { return static_cast<int>(ports_.size()); }

  OutputPort* get_mutable_port(int index) override {
    DRAKE_DEMAND(index >= 0 && index < get_num_ports());
    return ports_[index];
  }

  const OutputPort& get_port(int index) const override {
    DRAKE_DEMAND(index >= 0 && index < get_num_ports());
    return *ports_[index];
  }

  std::vector<OutputPort*>* get_mutable_ports() { return &ports_; }

 protected:
  // Returns a clone that has the same number of output ports, set to nullptr.
  HybridAutomatonOutput<T>* DoClone() const override {
    HybridAutomatonOutput<T>* clone = new HybridAutomatonOutput<T>();
    clone->ports_.resize(get_num_ports());
    return clone;
  }

 private:
  std::vector<OutputPort*> ports_;
};

template <typename T>
class DiagramTimeDerivatives : public DiagramContinuousState<T> {
 public:
  explicit DiagramTimeDerivatives(
      std::vector<std::unique_ptr<ContinuousState<T>>> substates)
      : DiagramContinuousState<T>(Unpack(substates)),
        substates_(std::move(substates)) {}

  ~DiagramTimeDerivatives() override {}

 private:
  template <typename U>
  std::vector<U*> Unpack(const std::vector<std::unique_ptr<U>>& in) {
    std::vector<U*> out(in.size());
    std::transform(in.begin(), in.end(), out.begin(),
                   [](auto& p) { return p.get(); });
    return out;
  }

  std::vector<std::unique_ptr<ContinuousState<T>>> substates_;
};


}  // namespace internal

/*
template <typename T>
class ModalSubsystem : public System<T> {
 public:
 private:
    typedef typename std::tuple<
      // System model.
      const System<T>*,
      // Formula representing the invariant for this mode.
      const std::vector<symbolic::Formula>*,  // TODO: Eigen??
      // Formula representing the initial conditions for this mode.
      const std::vector<symbolic::Formula>*,  // TODO: Eigen??
      // Index for this mode.
      ModeId> ModalSubsystem;
};
*/

/// Hybrid Automaton.

/// Limitations:
///  1) modal_subsystems can be Diagrams or (Leaf)Systems, but cannot themselves
/// be HybridAutomata.
/// TODO: Is there a way to mark systems as being "finalized" such that no
/// other system type can inherit from it?
///  2) modal_subsystems do not have to have consistent state dimensions,
/// input dimensions, nor output dimensions. (really a non-limitation)
/// TODO: Can we accommodate this most general case in the first spiral?
///  3) Only one initial modal_state is allowed.
///  4) The state dimensions can change over time (~check~), but the number of
/// input and output ports must be invariant to protect upstream producers and
/// downstream consumers.
/// TODO: Check the viability of a dynamically-sized state vector within
/// the hybrid automaton context.
///  5) ....
template <typename T>
class HybridAutomaton : public System<T> {
 public:
  typedef int ModeId;

  // TODO: Should we have fields of ModalSubsystem in System<T> instead?
  // Pro: Tighter coupling between System and its attributes.
  // Con: These attributes are really a property of hybrid automata and
  //   are (mostly) meaningless on a standalone basis.
  typedef typename std::tuple<
    // System model.
    const System<T>*,
    // Formula representing the invariant for this mode.
    const std::vector<symbolic::Formula>*,  // TODO: Eigen??
    // Formula representing the initial conditions for this mode.
    const std::vector<symbolic::Formula>*,  // TODO: Eigen??
    // Index for this mode.
    ModeId> ModalSubsystem;

  typedef typename std::tuple<
    // Modal subsystem pair.
    const std::pair<ModalSubsystem*, ModalSubsystem*>*,  //TODO: nested ptrs?
    // Formula representing the guard for this transition.
    const std::vector<symbolic::Formula>*> ModeTransition;  // TODO: Eigen??
  // Formula representing the reset map for this transition.
  // TODO: Do we want a formula with limited symantics here, or something?
  //const std::vector<symbolic::Formula>*

  //typedef typename std::pair<const ModalSubsystem*,
  //const ModalSubsystem*> ModalSubsystemPair;
  typedef typename std::pair<const System<T>*, int> PortIdentifier;

  ~HybridAutomaton() override {}

  ///
  //symbolic::Formula* get_mutable_invariant(int index) {
  //  DRAKE_ASSERT(index >= 0 && index < size());
  //
  //  return data_[index];
  //}

  /// Returns the list of modal subsystems.
  std::vector<const systems::System<T>*> GetSystems() const {
    std::vector<const systems::System<T>*> result;
    result.reserve(registered_systems_.size());
    for (const auto& system : registered_systems_) {
      result.push_back(system.get());
    }
    return result;
  }

  /// Returns true if any modal subsystem has direct feedthrough.
  bool has_any_direct_feedthrough() const override {
    for (const auto& modal_subsystem : modal_subsystems_) {
      auto system = get_subsystem(modal_subsystem);
      if (!system->has_any_direct_feedthrough()) {
        return false;
      }
    }
    return true;
  }

  std::unique_ptr<Context<T>> CreateDefaultContext() const override {
    const int num_systems = static_cast<int>(modal_subsystems_.size());
    // Reserve inputs as specified during initialization.
    auto context = std::make_unique<HybridAutomatonContext<T>>(num_systems);

    // Add each constituent system to the Context in the correct order.
    for (ModeId i = 0; i < num_systems; ++i) {
      auto system = get_subsystem(modal_subsystems_[i]);
      auto modal_context = system->CreateDefaultContext();
      auto modal_output = system->AllocateOutput(*modal_context);
      context->AddModalSubsystem(i,
                          std::move(modal_context), std::move(modal_output));
    }

    return std::unique_ptr<Context<T>>(context.release());
  }

  std::unique_ptr<BasicVector<T>> AllocateOutputVector(
      const SystemPortDescriptor<T>& descriptor) const {
    return std::make_unique<BasicVector<T>>(descriptor.get_size());
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const override {

    auto hybrid_context
      = dynamic_cast<const HybridAutomatonContext<T>*>(&context);

    DRAKE_DEMAND(hybrid_context != nullptr);
    ModeId id = hybrid_context->get_mode_id(context);
    const System<T>* subsystem = get_subsystem(modal_subsystems_[id]);

    //auto output = std::make_unique<internal::HybridAutomatonOutput<T>*>(
    //                               new internal::HybridAutomatonOutput<T>);
    auto output = std::make_unique<internal::HybridAutomatonOutput<T>>();

    for (const auto& descriptor : subsystem->get_output_ports()) {
      output->get_mutable_ports()->
        emplace_back(new OutputPort(AllocateOutputVector(descriptor)));
    }

    //return std::move(output);
    return std::unique_ptr<SystemOutput<T>>(output.release());
  }

  /// ======= Hybrid Automaton Execution Methods ======
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override {
    // Down-cast the context and output to HybridAutomatonContext and
    // DiagramOutput.
    auto hybrid_context
      = dynamic_cast<const HybridAutomatonContext<T>*>(&context);
    DRAKE_DEMAND(hybrid_context != nullptr);
    auto hybrid_output
      = dynamic_cast<internal::HybridAutomatonOutput<T>*>(output);
    DRAKE_DEMAND(hybrid_output != nullptr);

    // Populate the output with pointers to the appropriate subsystem
    // outputs in the HybridAutomatonContext. We do this on every call
    // to EvalOutput, so that the diagram_context and diagram_output
    // are not tightly coupled.
    ExposeSubsystemOutputs(*hybrid_context, hybrid_output);

    // Since the diagram output now contains pointers to the subsystem
    // outputs, all we need to do is ask those subsystem outputs to
    // evaluate themselves.  They will recursively evaluate any
    // intermediate inputs that they need.  for (const ModalSubsystem&
    // modal_subsystem : modal_subsystems_) {
    // EvaluateOutputPort(*diagram_context, modal_subsystem); }
  }

  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    std::vector<std::unique_ptr<ContinuousState<T>>> sub_derivatives;
    for (const ModalSubsystem modal_subsystem : modal_subsystems_) {
      auto system = get_subsystem(modal_subsystem);
      sub_derivatives.push_back(system->AllocateTimeDerivatives());
    }
    return std::unique_ptr<ContinuousState<T>>(
        new internal::DiagramTimeDerivatives<T>(std::move(sub_derivatives)));
  }

  void EvalTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const override {
    auto diagram_context
      = dynamic_cast<const HybridAutomatonContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);

    auto diagram_derivatives =
        dynamic_cast<DiagramContinuousState<T>*>(derivatives);
    DRAKE_DEMAND(diagram_derivatives != nullptr);
    const int n = diagram_derivatives->get_num_substates();

    // Evaluate the derivatives of each constituent system.
    for (int i = 0; i < n; ++i) {
      const Context<T>* subcontext
        = diagram_context->GetSubsystemContext(context);
      ContinuousState<T>* subderivatives =
          diagram_derivatives->get_mutable_substate(i);
      auto system = get_subsystem(modal_subsystems_[i]);
      system->EvalTimeDerivatives(*subcontext, subderivatives);
    }
  }

  // ====== Modal State Processing Functions ======

  /// Evaluate the guard at the current valuation of the state vector.
  T EvalInvariant(const systems::HybridAutomatonContext<T>& context) const {
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
    // TODO: add ^this check to all other context-consuming methods as well.

    // Evaluate the guard function.
    const systems::VectorBase<T>& state = context.get_continuous_state_vector();
    symbolic::Environment state_env;
    for (int i = 0; i < state.size(); i++) {
      //if ( state[i] == double ) {
      std::ostringstream key;
      key  << "x" << i;
      symbolic::Variable state_var{ key.str() };
      state_env.insert(state_var, state[i]);
      //} else {
      // throw std::runtime_error("types other than double not implemented.");
      //}
    }

    ModeId id = context.get_mode_id(context);
    auto invariant_formula
      = *std::get<1>(modal_subsystems_[id]);  // invariant (TODO: make a getter)
    // The guard is satisfied (returns a non-positive value) when
    // the ball's position is less than or equal to zero and its
    // velocity is non-positive.
    return (invariant_formula.at(0)).Evaluate(state_env);
  }

  // ====== Accessors ======

  /// Returns the subcontext that corresponds to the moda_subsystem.
  /// Classes inheriting from HybridAutomaton need access to this
  /// method in order to pass their constituent subsystems the
  /// apropriate subcontext.

  // TODO: need?

  Context<T>* GetMutableSubsystemContext(Context<T>* context,
                               const ModalSubsystem* modal_subsystem) const {
    DRAKE_DEMAND(context != nullptr);
    DRAKE_DEMAND(modal_subsystem != nullptr);
    auto modal_context
      = dynamic_cast<HybridAutomatonContext<T>*>(context);
    DRAKE_DEMAND(modal_context != nullptr);
    // const ModeId id = get_mode_id(modal_subsystem);
    return modal_context->GetMutableSubsystemContext(*context);
    // TODO: isn't dereferencing args bad practice? -- check.
  }

  /// Retrieves the state for a particular modal_subsystem.

  // TODO: need?
  State<T>* GetMutableSubsystemState(Context<T>* context,
                           const ModalSubsystem* modal_subsystem) const {
    Context<T>* subcontext
      = GetMutableSubsystemContext(context, modal_subsystem);
    return subcontext->get_mutable_state();
  }

  // Returns the index of the given @p ModalSubsystem.
  ModeId get_mode_id(const ModalSubsystem* sys) const {
    auto pos = find(modal_subsystems_.begin(), modal_subsystems_.end(), *sys);
    DRAKE_DEMAND(pos != modal_subsystems_.end());
    return std::distance(modal_subsystems_.begin(), pos);
  }

  // Returns the system of the given @p ModalSubsystem.
  const System<T>* get_subsystem(const ModalSubsystem& modal_subsystem) const {
    return std::get<0>(modal_subsystem);
  }

 protected:

  HybridAutomaton() {}

  void DoPublish(const Context<T>& context) const override {
    auto hybrid_context
      = dynamic_cast<const HybridAutomatonContext<T>*>(&context);
    DRAKE_DEMAND(hybrid_context != nullptr);

    for (const ModalSubsystem modal_subsystem : modal_subsystems_) {
      auto system = get_subsystem(modal_subsystem);
      system->Publish(*hybrid_context->GetSubsystemContext(context));
    }
  }

 private:
  // The finite-state machine (automaton) whose modes are the modal
  // subsystems and whose edges are the mode transitions.
  struct StateMachine {
    // The ordered subsystem ports that are inputs.
    std::vector<PortIdentifier> input_port_ids;
    // The ordered subsystem ports that are outputs.
    std::vector<PortIdentifier> output_port_ids;

    std::vector<ModalSubsystem> modal_subsystems;
    std::vector<ModeTransition> mode_transitions;

  };

  // Constructor for the HybridAutomaton.
  explicit HybridAutomaton(const StateMachine& state_machine) {
    Initialize(state_machine);
  }

  // Validates the given @p state_machine and general set-up.

  // TODO: can we roll this up more cleanly, possibly within the
  // calling function?
  void Initialize(const StateMachine& state_machine) {
    // Expect the modal subsytems to be empty (this suffices to
    // conclude that the list of mode transitions is also empty).
    DRAKE_DEMAND(modal_subsystems_.empty());

    // Copy the data from the state_machine into private member variables.
    input_port_ids_ = state_machine.input_port_ids;
    output_port_ids_ = state_machine.output_port_ids;
    //modal_subsystems_ = state_machine.modal_subsystems;
  }

  // Takes ownership of the modal subsystems from HybridAutomatonBuilder.

  // TODO: need?
  void Own(std::vector<std::unique_ptr<System<T>>> registered_systems) {
    // We must be given something to own.
    DRAKE_DEMAND(!registered_systems.empty());
    // We must not already own any subsystems.
    DRAKE_DEMAND(registered_systems_.empty());
    // All of those checks having passed, take ownership of the subsystems.
    registered_systems_ = std::move(registered_systems);

    // TODO: decide whether or not the systems need to know who their parent is.
    //for (auto& system : registered_systems_) {
    //  system->set_parent(this);
    //}
  }

  // Sets up the OutputPort pointers in @p output to point to the subsystem
  // outputs, found in @p context.
  void ExposeSubsystemOutputs(const HybridAutomatonContext<T>& context,
                           internal::HybridAutomatonOutput<T>* output) const {
    // The number of output ports of this diagram must equal the number of
    // ports in the provided DiagramOutput.
    const int num_ports = static_cast<int>(output_port_ids_.size());
    DRAKE_DEMAND(output->get_num_ports() == num_ports);

    for (int i = 0; i < num_ports; ++i) {
      const PortIdentifier& id = output_port_ids_[i];
      // For each configured output port ID, obtain from the
      // HybridAutomatonContext the actual OutputPort that produces
      // it.
      const int port_index = id.second;
      SystemOutput<T>* subsystem_output = context.GetSubsystemOutput(context);
      OutputPort* output_port = subsystem_output->get_mutable_port(port_index);

      // Make a pointer to the ith OutputPort.
      (*output->get_mutable_ports())[i] = output_port;
    }
  }

  // HybridAutomaton objects are neither copyable nor moveable.
  HybridAutomaton(const HybridAutomaton<T>& other) = delete;
  HybridAutomaton& operator=(const HybridAutomaton<T>& other) = delete;
  HybridAutomaton(HybridAutomaton<T>&& other) = delete;
  HybridAutomaton& operator=(HybridAutomaton<T>&& other) = delete;

  // TODO: deprecate.
  std::vector<std::unique_ptr<System<T>>> registered_systems_;

  // The ordered inputs and outputs.
  std::vector<PortIdentifier> input_port_ids_;
  std::vector<PortIdentifier> output_port_ids_;

  // The hybrid automaton data.
  // TODO: simplify the vector type declarations like what's done in VectorBase?
  std::vector<ModalSubsystem> modal_subsystems_;
  std::vector<ModeTransition> mode_transitions_;

  friend class HybridAutomatonBuilder<T>;
};

}  // namespace systems
}  // namespace drake
