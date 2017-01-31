#pragma once

// TODO: triage this list.
//#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

// TODO: triage this list.
#include "drake/common/symbolic_formula.h"
//#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
//#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system.h"
//#include "drake/systems/framework/system_input.h"
//#include "drake/systems/framework/system_output.h"

// Debugging.
//#include "drake/systems/framework/test_utilities/pack_value.h"

namespace drake {
namespace systems {

using std::unique_ptr;
using std::make_unique;
using std::shared_ptr;
using std::cerr;
using std::endl;

// TODO(jadecastro): Test capability to capture hybrid system-of-hybrid-systems
// functionality.

// TODO(jadecastro): Some comments are in order.
template <typename T>
class ModalSubsystem {
 public:
  typedef int ModeId;
  typedef int PortId;

  // TODO(jadecastro): Use setters instead, like in RigidBody.
  explicit ModalSubsystem(
      ModeId mode_id, shared_ptr<System<T>> system,
      std::vector<symbolic::Expression> invariant,
      std::vector<symbolic::Expression> initial_conditions,
      std::vector<PortId> input_port_ids, std::vector<PortId> output_port_ids)
      : mode_id_(mode_id), system_(std::move(system)), invariant_(invariant),
        initial_conditions_(initial_conditions),
        input_port_ids_(input_port_ids), output_port_ids_(output_port_ids) {
    CreateSymbolicVariables();
  }

  explicit ModalSubsystem(
      ModeId mode_id, shared_ptr<System<T>> system,
      std::vector<symbolic::Expression> invariant,
      std::vector<symbolic::Expression> initial_conditions)
      : mode_id_(mode_id), system_(std::move(system)), invariant_(invariant),
        initial_conditions_(initial_conditions) {
    CreateSymbolicVariables();
    PopulateDefaultPorts();
  }

  explicit ModalSubsystem(
      ModeId mode_id, shared_ptr<System<T>> system,
      std::vector<PortId> input_port_ids, std::vector<PortId> output_port_ids)
      : mode_id_(mode_id), system_(std::move(system)),
        input_port_ids_(input_port_ids), output_port_ids_(output_port_ids) {
    CreateSymbolicVariables();
  }

  explicit ModalSubsystem(
      ModeId mode_id, shared_ptr<System<T>> system)
      : mode_id_(mode_id), system_(std::move(system)) {
    CreateSymbolicVariables();
    PopulateDefaultPorts();
  }

  ModeId get_mode_id() const { return mode_id_; }
  System<T>* get_system() const {return system_.get(); }
  int get_num_input_ports() const {
    return static_cast<int>(input_port_ids_.size());
  }
  int get_num_output_ports() const {
    return static_cast<int>(output_port_ids_.size());
  }

  const std::vector<PortId> get_input_port_ids() const {
    return input_port_ids_;
  }
  std::vector<PortId>* get_mutable_input_port_ids() {
    return &input_port_ids_;
  }
  const std::vector<PortId> get_output_port_ids() const {
    return output_port_ids_;
  }
  std::vector<PortId>* get_mutable_output_port_ids() {
    return &output_port_ids_;
  }
  const std::vector<symbolic::Expression> get_invariant() const {
    return invariant_;
  }
  std::vector<symbolic::Expression>* get_mutable_invariant() {
    return &invariant_;
  }
  void set_invariant(std::vector<symbolic::Expression>& invariant) {
    invariant_ = invariant;
  }
  const std::vector<symbolic::Expression> get_initial_conditions() const {
    return initial_conditions_;
  }
  std::vector<symbolic::Expression>* get_mutable_initial_conditions() {
    return &initial_conditions_;
  }
  void set_intial_conditions(std::vector<symbolic::Expression>& init) {
    initial_conditions_ = init;
  }
  PortId get_input_port_id(const int index) const {
    DRAKE_DEMAND(index >=0 && index < static_cast<int>(input_port_ids_.size()));
    return input_port_ids_[index];
  }
  PortId get_output_port_id(const int index) const {
    DRAKE_DEMAND(index >=0 &&
                 index < static_cast<int>(output_port_ids_.size()));
    return output_port_ids_[index];
  }

  // TODO(jadecastro): Check for consistency of any incoming invariants or
  // initial condition formulas with the given symbolic_state_.
  const std::vector<symbolic::Variable>& get_symbolic_continuous_states()
      const {
    return xc_symbolic_;
  };
  const std::vector<symbolic::Variable>& get_symbolic_discrete_states() const {
    return xd_symbolic_;
  };

  /// Returns a clone that includes a deep copy of all the output ports.
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
  void CreateSymbolicVariables() {
    // TODO(jadecastro): Either we need to modify the system API to allow us
    // access to the underlying state dimensions without creating a throwaway
    // context *or* we just allocate the context here and output it along with
    // the ModalSubsystem.
    std::unique_ptr<Context<T>> context = system_->AllocateContext();

    // TODO(jadecastro): Implement this to handle both continuous state @p x and
    // input @p u.
    const int num_xc = context->get_continuous_state_vector().size();
    for (int i = 0; i < num_xc; ++i) {
      std::ostringstream key;
      key << "xc" << i;
      symbolic::Variable state_var{key.str()};
      xc_symbolic_.emplace_back(state_var);
    }
    const int num_xd = context->get_num_discrete_state_groups();
    for (int i = 0; i < num_xd; ++i) {
      std::ostringstream key;
      key << "xd" << i;
      symbolic::Variable state_var{key.str()};
      xd_symbolic_.emplace_back(state_var);
    }
  }

  // Populate the input and output ports with the full complement of system
  // inputs and outputs.
  void PopulateDefaultPorts() {
    input_port_ids_.resize(system_->get_num_input_ports());
    std::iota (std::begin(input_port_ids_), std::end(input_port_ids_), 0);
    output_port_ids_.resize(system_->get_num_output_ports());
    std::iota (std::begin(output_port_ids_), std::end(output_port_ids_), 0);
  }

  // Index for this mode.
  ModeId mode_id_;
  // TODO(jadecastro): Allow ModeId to take on an `enum` here in place of the
  // `int`?
  // The system model.
  shared_ptr<System<T>> system_;
  // Expression representing the invariant for this mode.
  std::vector<symbolic::Expression> invariant_;  // TODO: Eigen??
  // Expression representing the initial conditions for this mode.
  std::vector<symbolic::Expression> initial_conditions_;  // TODO: Eigen??
  // Index set of the input and output ports.
  std::vector<PortId> input_port_ids_;
  std::vector<PortId> output_port_ids_;
  // A vector of symbolic variables for each of the continuous states in the
  // system.
  std::vector<symbolic::Variable> xc_symbolic_;
  std::vector<symbolic::Variable> xd_symbolic_;
  // TODO(jadecastro): Store symbolic versions of the inputs also.
};


/// HybridAutomatonState is a State that is annotated with pointers to the
/// current active subsystem.
template <typename T>
class HybridAutomatonState : public State<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HybridAutomatonState)

  /// Constructs a DiagramState consisting of @p size substates.
  explicit HybridAutomatonState<T>() :
      State<T>(),
      substate_(),
      owned_substate_() {}

  /// Sets the @p mode_id for the current active subsystem.
  void set_mode_id(int mode_id) { mode_id_ = mode_id; }

  /// Sets the state of the HA to @p substate, but does not take ownership
  /// of @p substate.
  void set_substate(State<T>* substate) { substate_ = substate; }

  /// Sets the state of the HA to @p substate, taking ownership of @p substate.
  void set_and_own_substate(std::unique_ptr<State<T>> substate) {
    set_substate(substate.get());
    owned_substate_ = std::move(substate);
  }

  /// Returns the stored substate.
  State<T>* get_mutable_substate() { return substate_; }

  /// Finalizes the state.
  void Finalize() {
    DRAKE_DEMAND(!finalized_);
    finalized_ = true;

    // Create the continuous and discrete states.
    if (substate_->get_continuous_state() != nullptr) {
      const ContinuousState<T>& xc = *substate_->get_continuous_state();
      const int num_q = xc.get_generalized_position().size();
      const int num_v = xc.get_generalized_velocity().size();
      const int num_z = xc.get_misc_continuous_state().size();
      const BasicVector<T>& xc_vector =
          dynamic_cast<const BasicVector<T>&>(xc.get_vector());
      this->set_continuous_state(std::make_unique<ContinuousState<T>>(
          xc_vector.Clone(), num_q, num_v, num_z));
    }
    this->set_discrete_state(substate_->get_discrete_state()->Clone());

    // Combine the current abstract state with the active modal subsystem state.
    AbstractState* subsystem_xm = substate_->get_mutable_abstract_state();
    std::vector<unique_ptr<AbstractValue>> hybrid_xm;
    for (int i_xm = 0; i_xm < subsystem_xm->size(); ++i_xm) {
      hybrid_xm.push_back(
          subsystem_xm->get_mutable_abstract_state(i_xm).Clone());
    }
    hybrid_xm.push_back(unique_ptr<AbstractValue>(new Value<int>(mode_id_)));

    // The wrapper states do not own the subsystem state.
    this->set_abstract_state(std::make_unique<AbstractState>(
        std::move(hybrid_xm)));

    //std::cerr << " MakeState: Abstract state: "
    //          << this->template get_abstract_state<int>(0) << std::endl;
  }

 private:
  bool finalized_{false};
  State<T>* substate_;
  std::unique_ptr<State<T>> owned_substate_;
  int mode_id_;
};


/// The HybridAutomatonContext contains the context and output for the active
/// ModalSubsystem, as chosen by HybridAutomaton. In addition, it augments the
/// subsystem contexts with an abstract state designating the active mode.
///
/// In general, users should not need to interact with a HybridAutomatonContext
/// directly. Use the accessors on Hybrid Automaton instead.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class HybridAutomatonContext : public Context<T> {
 public:
  typedef int ModeId;
  typedef int PortId;

  /// Constructs a HybridAutomatonContext with a fixed number @p num_subsystems
  /// in a way that allows for dynamic re-sizing during discrete events.
  explicit HybridAutomatonContext() {}

  // ADD COMMENTS
  void RegisterSubsystem(unique_ptr<ModalSubsystem<T>> modal_subsystem,
                         unique_ptr<Context<T>> subcontext,
                         unique_ptr<SystemOutput<T>> suboutput) {
    subcontext->set_parent(this);

    context_ = std::move(subcontext);
    output_ = std::move(suboutput);
    modal_subsystem_ = std::move(modal_subsystem);

    // TODO(jadecastro): Need to involve the simulator too.
    //   `reset_context` along with `Initialize` and `release_context`?
  }

  // ADD COMMENTS
  void DeRegisterSubsystem() {
    context_.release();
    output_.release();
    //modal_subsystem_.release();
  }

  /// Declares that a particular input port of a particular subsystem is an
  /// input to the entire HA that allocates this Context.
  ///
  /// User code should not call this method. It is for use during HA context
  /// allocation only.
  void ExportInput(const PortId& port_id) {
    modal_subsystem_->get_mutable_input_port_ids()->emplace_back(port_id);
  }

  void ExportInput(const std::vector<PortId>& port_ids) {
    // Debugging.....
    //DRAKE_DEMAND(context_ != nullptr);
    //DRAKE_DEMAND(output_ != nullptr);
    DRAKE_DEMAND(modal_subsystem_ != nullptr);

    for (auto& port_id : port_ids) { this->ExportInput(port_id); }
  }

  /// Declares that a particular input port of a particular subsystem is an
  /// input to the entire HA that allocates this Context.
  ///
  /// User code should not call this method. It is for use during HA context
  /// allocation only.
  void ExportOutput(const PortId& port_id) {
    modal_subsystem_->get_mutable_output_port_ids()->emplace_back(port_id);
  }

  void ExportOutput(const std::vector<PortId>& port_ids) {
    for (auto& port_id : port_ids) { this->ExportOutput(port_id); }
  }

  /// Generates the state vector for the HA.
  ///
  /// User code should not call this method. It is for use during
  /// HybridAutomaton context allocation only.
  //
  // TODO(jadecastro): This and DoClone has changed a bit in master... we'll
  // have to refactor it slightly.
  void MakeState() {
    auto hybrid_state = std::make_unique<HybridAutomatonState<T>>();
    Context<T>* subcontext = context_.get();
    DRAKE_DEMAND(subcontext != nullptr);
    hybrid_state->set_substate(subcontext->get_mutable_state());

    hybrid_state->set_mode_id(modal_subsystem_->get_mode_id());
    hybrid_state->Finalize();

    state_ = hybrid_state.get();
  }
  // TODO(jadecastro): Likely a temporary function 'till we wrangle with the API
  // updates to Context<T>.

  /// Returns the output structure for a given constituent system at @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// HybridAutomatonContext at that index.
  SystemOutput<T>* GetSubsystemOutput() const { return output_.get(); }

  /// Returns the context structure for a given constituent system @p index.
  /// Aborts if @p index is out of bounds, or if no system has been added to the
  /// HybridAutomatonContext at that index.
  const Context<T>* GetSubsystemContext() const { return context_.get(); }

  /// Returns the context structure for a given subsystem @p index.  Aborts if
  /// @p index is out of bounds, or if no system has been added to the
  /// HybridAutomatonContext at that index.
  Context<T>* GetMutableSubsystemContext() { return context_.get(); }

  /// Recursively sets the time on this context and all subcontexts.
  // TODO: should we only advance time for the current active subsystem?
  void set_time(const T& time_sec) override {
    Context<T>::set_time(time_sec);
    Context<T>* subcontext = GetMutableSubsystemContext();
    if (subcontext != nullptr) {
      subcontext->set_time(time_sec);
    }
  }

  int get_num_input_ports() const override {
    const ModalSubsystem<T>* modal_subsystem = GetModalSubsystem();
    DRAKE_DEMAND(modal_subsystem != nullptr);
    return modal_subsystem->get_num_input_ports();
  }

  void SetInputPort(int port_index, unique_ptr<InputPort> port) override {
    // TODO(jadecastro): DependentInputPort? (i.e. eval some output to get the
    // input?)
    const ModalSubsystem<T>* modal_subsystem = GetModalSubsystem();
    DRAKE_DEMAND(modal_subsystem != nullptr);
    PortId subsystem_port_id = modal_subsystem->get_input_port_id(port_index);
    GetMutableSubsystemContext()
        ->SetInputPort(subsystem_port_id, std::move(port));
  }

  ModalSubsystem<T>* GetModalSubsystem() const {
    return modal_subsystem_.get();
  }

  ModeId get_mode_id() const {
    const ModalSubsystem<T>* modal_subsystem = GetModalSubsystem();
    DRAKE_DEMAND(modal_subsystem != nullptr);
    return modal_subsystem->get_mode_id();
  }

  // Updates the abstract value with the current active ModalSubsystem.
  void SetModalState() {
    const int mss_index = this->get_mutable_abstract_state()->size() - 1;
    //std::cerr << " mss_index: " << mss_index << std::endl;
    //ModalSubsystem<T> mss = this->
    //    template get_mutable_abstract_state<ModalSubsystem<T>>(mss_index);
    // TODO(jadecastro): int vs. ModalSubsystem<T>?
    this->template get_mutable_abstract_state<int>(mss_index) =
        GetModalSubsystem()->get_mode_id();
  }

  /// Returns a reference to the continuous symbolic state vector.
  //const std::vector<symbolic::Variable>& get_symbolic_continuous_states()
  //    const {
  //  ModalSubsystem<T>* modal_subsystem = GetModalSubsystem();
  //  return modal_subsystem->get_symbolic_continuous_states();
  //};

  /// Returns a reference to the discrete symbolic state vector.
  //const std::vector<symbolic::Variable>& get_symbolic_discrete_states() const {
  //   ModalSubsystem<T>* modal_subsystem = GetModalSubsystem();
  // return modal_subsystem->get_symbolic_discrete_states();
  //};

  /// @name Mandatory overrides.
  /// Returns a reference to the state vector.
  const State<T>& get_state() const override { return *state_; }

  /// Returns a pointer to the state vector.
  State<T>* get_mutable_state() override { return state_; }

 protected:
  HybridAutomatonContext<T>* DoClone() const override {
    HybridAutomatonContext<T>* clone = new HybridAutomatonContext();

    // Clone all the subsystem contexts and outputs.  This basically repeats
    // everything in CreateDefaultContext.
    clone->RegisterSubsystem(modal_subsystem_->Clone(), context_->Clone(),
                             output_->Clone());

    // Build the state for the initially-activated subsystem in the HA.
    clone->MakeState();

    // Make deep copies of everything else using the default copy constructors.
    *clone->get_mutable_step_info() = this->get_step_info();

    return clone;
  }

  /// The caller owns the returned memory.
  State<T>* DoCloneState() const override {
    HybridAutomatonState<T>* clone = new HybridAutomatonState<T>();

    Context<T>* context = context_.get();
    clone->set_and_own_substate(context->CloneState());

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
    return Context<T>::GetInputPort(*GetSubsystemContext(),
                                    subsystem_port_id);
  }

 private:
  // The internal state of the HA System.
  HybridAutomatonState<T>* state_ = nullptr;

  // Data for the current active subsystem.
  unique_ptr<SystemOutput<T>> output_;
  unique_ptr<Context<T>> context_;
  unique_ptr<ModalSubsystem<T>> modal_subsystem_;
};

}  // namespace systems
}  // namespace drake
