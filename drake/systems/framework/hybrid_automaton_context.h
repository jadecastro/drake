#pragma once

#include <map>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/hybrid_automaton_continuous_state.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

using std::make_unique;
using std::move;
using std::shared_ptr;
using std::unique_ptr;

// TODO(jadecastro): Some comments are in order.
template <typename T>
class ModalSubsystem {
 public:
  typedef int ModeId;
  typedef int PortId;

  explicit ModalSubsystem(
      ModeId mode_id, shared_ptr<System<T>> system,
      std::vector<symbolic::Expression> invariant,
      std::vector<symbolic::Expression> initial_conditions,
      std::vector<PortId> input_port_ids, std::vector<PortId> output_port_ids)
      : mode_id_(mode_id), system_(move(system)), invariant_(invariant),
        initial_conditions_(initial_conditions),
        input_port_ids_(input_port_ids), output_port_ids_(output_port_ids) {
    CreateSymbolicStatesAndInputs();
  }

  explicit ModalSubsystem(
      ModeId mode_id, shared_ptr<System<T>> system,
      std::vector<PortId> input_port_ids, std::vector<PortId> output_port_ids)
      : mode_id_(mode_id), system_(move(system)),
        input_port_ids_(input_port_ids), output_port_ids_(output_port_ids) {
    CreateSymbolicStatesAndInputs();
  }

  explicit ModalSubsystem(
      ModeId mode_id, shared_ptr<System<T>> system)
      : mode_id_(mode_id), system_(move(system)) {
    CreateSymbolicStatesAndInputs();
    PopulateDefaultPorts();
  }

  /// Accessors for the underlying data.
  ModeId get_mode_id() const { return mode_id_; }
  System<T>* get_system() const {return system_.get(); }
  const std::vector<PortId> get_input_port_ids() const {
    return input_port_ids_;
  }
  std::vector<PortId>* get_mutable_input_port_ids() {
    return &input_port_ids_;
  }
  PortId get_input_port_id(const int index) const {
    DRAKE_DEMAND(index >=0 && index < static_cast<int>(input_port_ids_.size()));
    return input_port_ids_[index];
  }
  int get_num_input_ports() const {
    return static_cast<int>(input_port_ids_.size());
  }
  const std::vector<PortId> get_output_port_ids() const {
    return output_port_ids_;
  }
  std::vector<PortId>* get_mutable_output_port_ids() {
    return &output_port_ids_;
  }
  PortId get_output_port_id(const int index) const {
    DRAKE_DEMAND(index >=0 &&
                 index < static_cast<int>(output_port_ids_.size()));
    return output_port_ids_[index];
  }
  int get_num_output_ports() const {
    return static_cast<int>(output_port_ids_.size());
  }

  /// Accessors for the symbolic::Expressions for the invariants and initial
  /// condition state sets for this ModalSubsystem. Their defining sets are
  /// semialgebraic: a state assignment is within the set iff it evaluates to a
  /// non-negative value.
  const std::vector<symbolic::Expression> get_invariant() const {
    return invariant_;
  }
  std::vector<symbolic::Expression>* get_mutable_invariant() {
    return &invariant_;
  }
  const std::vector<symbolic::Expression> get_initial_conditions() const {
    return initial_conditions_;
  }
  std::vector<symbolic::Expression>* get_mutable_initial_conditions() {
    return &initial_conditions_;
  }

  // TODO(jadecastro): Check for consistency of any incoming invariants or
  // initial condition formulas with the given symbolic_state_.
  const std::vector<symbolic::Variable>& get_symbolic_continuous_states()
      const {
    return symbolic_variables_.at("xc")[0];
  };
  const std::vector<symbolic::Variable>& get_symbolic_discrete_states_at(
      const int index) const {
    return symbolic_variables_.at("xd")[index];
  };
  int get_num_symbolic_discrete_states() const {
    return symbolic_variables_.at("xd").size();
  };

  /// Returns a clone that includes a deep copy of all the underlying data.
  unique_ptr<ModalSubsystem<T>> Clone() const {
    DRAKE_DEMAND(system_ != nullptr);
    shared_ptr<System<T>> sys = system_;
    ModalSubsystem<T>* clone =
        new ModalSubsystem<T>(mode_id_, sys, invariant_, initial_conditions_,
                              input_port_ids_, output_port_ids_);
    DRAKE_DEMAND(clone != nullptr);
    return unique_ptr<ModalSubsystem<T>>(clone);
  }

 private:
  // Create symbolic variables based on the expected context for this subsystem.
  void CreateSymbolicStatesAndInputs() {
    // Create a temporary context to extract the needed dimensions.
    unique_ptr<Context<T>> context = system_->AllocateContext();

    // Create symbolic variables for the continuous and discrete states.
    CreateSymbolicVariables(
        "xc", context->get_continuous_state_vector().size());
    for (int i = 0; i < context->get_num_discrete_state_groups(); ++i) {
      CreateSymbolicVariables("xd", context->get_discrete_state(i)->size());
    }
    context.reset();
  }

  // Creates symbolic variables according to the variable_type key word.
  void CreateSymbolicVariables(const std::string variable_type,
                               const int size) {
    std::vector<std::vector<symbolic::Variable>> sym{};
    if (symbolic_variables_.find(variable_type) != symbolic_variables_.end()) {
      sym = symbolic_variables_.at(variable_type);
    }
    std::vector<symbolic::Variable> row;
    for (int i = 0; i < size; ++i) {
      std::ostringstream key{};
      key << variable_type << i;
      symbolic::Variable var{key.str()};
      row.emplace_back(var);
    }
    sym.emplace_back(row);
    symbolic_variables_.insert(std::make_pair(variable_type, sym));
  }

  // Populates the input and output ports with the full complement of system
  // inputs and outputs.
  void PopulateDefaultPorts() {
    input_port_ids_.resize(system_->get_num_input_ports());
    std::iota (std::begin(input_port_ids_), std::end(input_port_ids_), 0);
    output_port_ids_.resize(system_->get_num_output_ports());
    std::iota (std::begin(output_port_ids_), std::end(output_port_ids_), 0);
  }

  // An identifier for this mode.
  // TODO(jadecastro): Allow ModeId to take on a descriptor in place of the int.
  ModeId mode_id_;
  // The system model.
  shared_ptr<System<T>> system_;
  // Expression representing the invariant for this mode.
  std::vector<symbolic::Expression> invariant_;
  // Expression representing the initial conditions for this mode.
  std::vector<symbolic::Expression> initial_conditions_;
  // Index set of the input and output ports.
  std::vector<PortId> input_port_ids_;
  std::vector<PortId> output_port_ids_;
  // A vector of symbolic variables for each of the continuous states in the
  // system.
  std::map<std::string, std::vector<std::vector<symbolic::Variable>>>
      symbolic_variables_;
};


/// HybridAutomatonState is a State that is annotated with pointers to the
/// current active subsystem.
template <typename T>
class HybridAutomatonState : public State<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HybridAutomatonState)

  /// Constructs a DiagramState consisting of @p size substates.
  explicit HybridAutomatonState<T>() : State<T>(), state_(), owned_state_() {}

  /// Sets the @p mode_id for the current active subsystem.
  void set_mode_id(int mode_id) { mode_id_ = mode_id; }

  /// Sets the state of the HA to @p substate, but does not take ownership
  /// of @p substate.
  void set_substate(State<T>* state) { state_ = state; }

  /// Sets the state of the HA to @p substate, taking ownership of @p substate.
  void set_and_own_substate(unique_ptr<State<T>> state) {
    state_ = state.get();
    owned_state_ = move(state);
  }

  /// Returns the stored substate.
  State<T>* get_mutable_substate() { return state_; }

  /// Finalizes the state, augmenting any abstract data with the current @p
  /// ModeId.
  void Finalize() {
    DRAKE_DEMAND(!finalized_);
    finalized_ = true;

    // Create the continuous and discrete states.
    DRAKE_DEMAND(state_->get_continuous_state() != nullptr);
    ContinuousState<T>* xc = state_->get_mutable_continuous_state();
    VectorBase<T>* x = xc->get_mutable_vector();
    int num_q = xc->get_generalized_position().size();
    int num_v = xc->get_generalized_velocity().size();
    int num_z = xc->get_misc_continuous_state().size();
    this->set_continuous_state(
        make_unique<internal::HybridAutomatonContinuousState<T>>(
            x, num_q, num_v, num_z));

    const std::vector<BasicVector<T>*>& xd =
        state_->get_mutable_discrete_state()->get_data();
    this->set_discrete_state(make_unique<DiscreteState<T>>(xd));

    // Combine the current abstract state with the active modal subsystem state.
    AbstractState* subsystem_xm = state_->get_mutable_abstract_state();
    std::vector<unique_ptr<AbstractValue>> hybrid_xm;
    for (int i = 0; i < subsystem_xm->size(); ++i) {
      hybrid_xm.push_back(subsystem_xm->get_mutable_abstract_state(i).Clone());
    }
    hybrid_xm.push_back(unique_ptr<AbstractValue>(new Value<int>(mode_id_)));

    // The wrapper states do not own the subsystem state.
    this->set_abstract_state(make_unique<AbstractState>(move(hybrid_xm)));
  }

 private:
  bool finalized_{false};
  State<T>* state_;
  unique_ptr<State<T>> owned_state_;
  int mode_id_;
};


/// The HybridAutomatonContext contains the context and output for the active
/// ModalSubsystem, as chosen by HybridAutomaton. In addition, it augments the
/// subsystem contexts with an abstract state designating the active mode.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class HybridAutomatonContext : public Context<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HybridAutomatonContext)

  typedef int ModeId;
  typedef int PortId;

  /// Constructs a HybridAutomatonContext.
  explicit HybridAutomatonContext() {}

  /// Registers the ModalSubsystem by passing ownership of its constituent data
  /// structures into this context.  If the object is already initialized, the
  /// existing data is discarded.
  void RegisterSubsystem(unique_ptr<ModalSubsystem<T>> modal_subsystem,
                         unique_ptr<Context<T>> subcontext,
                         unique_ptr<SystemOutput<T>> suboutput) {
    subcontext->set_parent(this);

    context_.swap(subcontext);
    output_.swap(suboutput);
    modal_subsystem_.swap(modal_subsystem);

    // TODO(jadecastro): Need to involve the simulator too.
    //   `reset_context` along with `Initialize` and `release_context`?
  }

  /// Declares that a particular input port of the active subsystem is an input
  /// to the allocating HybridAutomaton.
  ///
  /// Note: User code should not call this method. It is for use during context
  /// allocation only.
  void ExportInput(const PortId& port_id) {
    modal_subsystem_->get_mutable_input_port_ids()->emplace_back(port_id);
  }

  void ExportInput(const std::vector<PortId>& port_ids) {
    for (auto& port_id : port_ids) { this->ExportInput(port_id); }
  }

  /// Declares that a particular input port of the active subsystem is an input
  /// to the allocating HybridAutomaton.
  ///
  /// Note: User code should not call this method. It is for use during context
  /// allocation only.
  void ExportOutput(const PortId& port_id) {
    modal_subsystem_->get_mutable_output_port_ids()->emplace_back(port_id);
  }

  void ExportOutput(const std::vector<PortId>& port_ids) {
    for (auto& port_id : port_ids) { this->ExportOutput(port_id); }
  }

  /// Generates the state vector for the allocating HybridAutomaton.
  ///
  /// Note: User code should not call this method. It is for use during context
  /// allocation only.
  void MakeState() {
    auto hybrid_state = make_unique<HybridAutomatonState<T>>();
    Context<T>* subcontext = context_.get();
    DRAKE_DEMAND(subcontext != nullptr);
    hybrid_state->set_substate(subcontext->get_mutable_state());

    hybrid_state->set_mode_id(modal_subsystem_->get_mode_id());
    hybrid_state->Finalize();

    state_ = move(hybrid_state);
  }

  /// Returns the output structure for the active ModalSubsystem.
  SystemOutput<T>* GetSubsystemOutput() const { return output_.get(); }

  /// Returns the context structure for the active ModalSubsystem.
  const Context<T>* GetSubsystemContext() const { return context_.get(); }

  /// Returns the mutable context structure for the active ModalSubsystem.
  Context<T>* GetMutableSubsystemContext() { return context_.get(); }

  /// Szets the time in both this context and the active subcontext.
  void set_time(const T& time_sec) override {
    Context<T>::set_time(time_sec);
    Context<T>* subcontext = GetMutableSubsystemContext();
    if (subcontext != nullptr) {
      subcontext->set_time(time_sec);
    }
  }

  /// Retrieves a pointer to the current active ModalSubsystem.
  ModalSubsystem<T>* GetModalSubsystem() const {
    return modal_subsystem_.get();
  }

  /// Retrieves the identity of the current modal subsystem.
  ModeId get_mode_id() const {
    const ModalSubsystem<T>* modal_subsystem = GetModalSubsystem();
    DRAKE_DEMAND(modal_subsystem != nullptr);
    return modal_subsystem->get_mode_id();
  }

  /// Updates the abstract value with the current active ModalSubsystem.
  void SetModalState() {
    auto abstract_state = this->get_mutable_abstract_state();
    DRAKE_DEMAND(abstract_state != nullptr);

    //ModalSubsystem<T> mss = this->
    //    template get_mutable_abstract_state<ModalSubsystem<T>>(mss_index);
    // TODO(jadecastro): int vs. ModalSubsystem<T>?
    this->template get_mutable_abstract_state<int>(abstract_state->size()-1) =
        GetModalSubsystem()->get_mode_id();
  }

  /// Returns a pointer to the continuous symbolic state vector.
  const std::vector<symbolic::Variable>& get_symbolic_continuous_states()
      const {
    ModalSubsystem<T>* modal_subsystem = GetModalSubsystem();
    DRAKE_DEMAND(modal_subsystem != nullptr);
    return modal_subsystem->get_symbolic_continuous_states();
  };

  /// Returns a pointer to the discrete symbolic state vector at @p index.
  const std::vector<symbolic::Variable>& get_symbolic_discrete_states_at(
      const int index) const {
    ModalSubsystem<T>* modal_subsystem = GetModalSubsystem();
    DRAKE_DEMAND(modal_subsystem != nullptr);
    return modal_subsystem->get_symbolic_discrete_states_at(index);
  };

  /// Returns a reference to the state vector.
  const State<T>& get_state() const override { return *state_; }

  /// Returns a pointer to the state vector.
  State<T>* get_mutable_state() override { return state_.get(); }

  /// Retrieves the input ports for the current active ModalSubsystem.
  int get_num_input_ports() const override {
    const ModalSubsystem<T>* modal_subsystem = GetModalSubsystem();
    DRAKE_DEMAND(modal_subsystem != nullptr);
    return modal_subsystem->get_num_input_ports();
  }

  /// Retrieves the output ports for the current active ModalSubsystem.
  void SetInputPort(int port_index, unique_ptr<InputPort> port) override {
    const ModalSubsystem<T>* modal_subsystem = GetModalSubsystem();
    DRAKE_DEMAND(modal_subsystem != nullptr);
    PortId subsystem_port_id = modal_subsystem->get_input_port_id(port_index);
    GetMutableSubsystemContext()
        ->SetInputPort(subsystem_port_id, move(port));
  }

 protected:
  HybridAutomatonContext<T>* DoClone() const override {
    HybridAutomatonContext<T>* clone = new HybridAutomatonContext();

    // Clone all the subsystem contexts and outputs.
    clone->RegisterSubsystem(modal_subsystem_->Clone(), context_->Clone(),
                             output_->Clone());

    // Build the state for the initially-activated subsystem in the HA.
    clone->MakeState();

    // Make deep copies of everything else using the default copy constructors.
    *clone->get_mutable_step_info() = this->get_step_info();

    return clone;
  }

  /// Clones the states. The caller owns the returned memory.
  State<T>* DoCloneState() const override {
    HybridAutomatonState<T>* clone = new HybridAutomatonState<T>();

    Context<T>* context = context_.get();
    clone->set_and_own_substate(context->CloneState());
    clone->set_mode_id(get_mode_id());
    clone->Finalize();
    return clone;
  }

  /// Returns the input port at the given @p index, whose subsystem is derived
  /// from the context.
  const InputPort* GetInputPort(int port_index) const override {
    // TODO(jadecastro): DependentInputPort? (i.e. eval some output to get the
    // input?)
    const ModalSubsystem<T>* modal_subsystem = GetModalSubsystem();
    DRAKE_DEMAND(modal_subsystem != nullptr);
    PortId subsystem_port_id = modal_subsystem->get_input_port_id(port_index);
    return Context<T>::GetInputPort(*GetSubsystemContext(), subsystem_port_id);
  }

 private:
  // The internal state of the HybridAutomaton.
  unique_ptr<HybridAutomatonState<T>> state_;

  // The data structures for the current active subsystem.
  unique_ptr<SystemOutput<T>> output_;
  unique_ptr<Context<T>> context_;
  unique_ptr<ModalSubsystem<T>> modal_subsystem_;
};

}  // namespace systems
}  // namespace drake
