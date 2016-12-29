#pragma once

// TODO: triage this list.
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

// TODO: triage this list.
#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

// TODO(jadecastro): How to concatenate abstract states from subsystems to the
// parent hybrid system.
//
// TODO(jadecastro): Better use of unique pointers for memory management and
// ownership hierarchy.  See, e.g.:
// https://github.com/ToyotaResearchInstitute/drake-maliput/commit/ ...
//    cc29ae0c2ea265150a4d37554c4e63bf0751d30e
//
// TODO(jadecastro): Implement and test capability to capture hybrid
// system-of-hybrid-systems functionality.
//
// TODO(jadecastro): Consistent naming modalsubsystems vs. modal_subsystems, id
// vs. mode_id, ....

/// HybridAutomatonState is a State, annotated with un-owned pointers
/// to all the mutable subsystem states that it spans.
template <typename T>
  class HybridAutomatonState : public State<T> {
 public:
  /// Constructs a HybridAutomatonSubsystemState consisting of @p size
  /// substates.
  explicit HybridAutomatonState<T>(int size) : State<T>(),
      modal_substates_(size) {}

  /// Returns the substate at @p index.
  State<T>* get_mutable_substate(const int index) {
    DRAKE_DEMAND(index >= 0 && index < (int)modal_substates_.size());
    return modal_substates_[index];
  }

  /// Sets the substate at @p index to @p substate, or aborts if @p index is out
  /// of bounds.
  void set_substate(const int index, State<T>* modal_substate) {
    DRAKE_DEMAND(index >= 0 && index < (int)modal_substates_.size());
    modal_substates_[index] = modal_substate;
  }

 private:
  std::vector<State<T>*> modal_substates_;
};


// TODO(jadecastro): Some comments are in order.
template <typename T>
class ModalSubsystem {
 public:
  typedef int ModeId;
  typedef int PortIdentifier;

  // TODO(jadecastro): Use setters instead, like in RigidBody.
  explicit ModalSubsystem(
      const ModeId mode_id, const System<T>* system,
      std::vector<symbolic::Formula>* invariant,
      std::vector<symbolic::Formula>* initial_conditions,
      std::vector<PortIdentifier> input_port_ids,
      std::vector<PortIdentifier> output_port_ids)
      : mode_id_(mode_id), system_(system), invariant_(invariant),
        initial_conditions_(initial_conditions),
        input_port_ids_(input_port_ids), output_port_ids_(output_port_ids) {}

  explicit ModalSubsystem(
      const ModeId mode_id, const System<T>* system,
      std::vector<symbolic::Formula>* invariant,
      std::vector<symbolic::Formula>* initial_conditions)
      : mode_id_(mode_id), system_(system), invariant_(invariant),
        initial_conditions_(initial_conditions) { SetDefaultPortIds(); }

  explicit ModalSubsystem(
      const ModeId mode_id, const System<T>* system,
      std::vector<PortIdentifier> input_port_ids,
      std::vector<PortIdentifier> output_port_ids)
      : mode_id_(mode_id), system_(system), input_port_ids_(input_port_ids),
        output_port_ids_(output_port_ids){ SetDefaultPortIds(); }

  explicit ModalSubsystem(
      const ModeId mode_id, const System<T>* system)
      : mode_id_(mode_id), system_(system) { SetDefaultPortIds(); }

  ModeId get_mode_id() const { return mode_id_; }
  System<T>* get_system() const {return system_; }
  int get_num_input_ports() const {
    return static_cast<int>(input_port_ids_.size());
  }

  const std::vector<PortIdentifier> get_input_port_ids() const {
    return input_port_ids_;
  }
  std::vector<PortIdentifier>* get_mutable_input_port_ids() {
    return &input_port_ids_;
  }
  const std::vector<PortIdentifier> get_output_port_ids() const {
    return output_port_ids_;
  }
  // TODO(jadecastro): Do we really need the following two getters?
  // TODO: const?
  PortIdentifier get_input_port_id(const int index) const {
    DRAKE_DEMAND(index >=0 && index < (int)input_port_ids_.size());
    return input_port_ids_[index];
  }
  PortIdentifier get_output_port_id(const int index) const {
    DRAKE_DEMAND(index >=0 && index < (int)output_port_ids_.size());
    return output_port_ids_[index];
  }

  /// Returns a clone that includes a deep copy of all the output ports.
  // TODO(jadecastro): Decide whether or not we actually need ModalSubsystems to
  // be unique_ptr, and hence need this function.
  std::unique_ptr<ModalSubsystem<T>> Clone() const {
    ModalSubsystem<T>* clone =
        new ModalSubsystem<T>(mode_id_, system_,
                              invariant_, initial_conditions_,
                              input_port_ids_, output_port_ids_);
    return std::unique_ptr<ModalSubsystem<T>>(clone);
  }

 private:
  // Index for this mode.
  ModeId mode_id_;
  // TODO(jadecastro): Allow ModeId to take on an `enum` here in place of the
  // `int`.
  // The system model.
  const System<T>* system_;
  // Formula representing the invariant for this mode.
  std::vector<symbolic::Formula>* invariant_;  // TODO: Eigen??
  // Formula representing the initial conditions for this mode.
  std::vector<symbolic::Formula>* initial_conditions_;  // TODO: Eigen??
  // Index set of the input and output ports.
  std::vector<PortIdentifier> input_port_ids_;
  std::vector<PortIdentifier> output_port_ids_;

  void SetDefaultPortIds() {
    if (input_port_ids_.empty()) {
      for (int id = 0; id < system_->get_num_input_ports(); ++id) {
        input_port_ids_.emplace_back(id);
      }
    }
    if (output_port_ids_.empty()) {
      for (int id = 0; id < system_->get_num_output_ports(); ++id) {
        output_port_ids_.emplace_back(id);
      }
    }
    // TODO(jadecastro): Address segfaulting in symbolic::Formula.
    //if (invariant_->empty()) {
    //invariant_->push_back(symbolic::Formula::False());
    //}
    //if (initial_conditions_->empty()) {
    //initial_conditions_->push_back(symbolic::Formula::True());
    //}
  }
};

/// The HybridAutomatonContext is a container for all of the data necessary to
/// uniquely determine the computations performed by a HybridAutomaton (HA).
/// Specifically, a HybridAutomatonContext contains contexts and outputs for all
/// modal subsystems in the finite-state machine. In addition, it augments the
/// subsystem contexts with an abstract (modal) state designating the current
/// active ModalSubsystem.
///
/// In the context, the size of the state vector may change, but the inputs and
/// outputs must be of fixed size.
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
  typedef int PortIdentifier;

  /// Constructs a HybridAutomatonContext with a fixed number @p num_subsystems
  /// in a way that allows for dynamic re-sizing during discrete events.
  explicit HybridAutomatonContext(const int num_subsystems)
      : state_(num_subsystems) {}

  // COMMENT
  void RegisterSubsystem(std::unique_ptr<ModalSubsystem<T>> modal_subsystem,
                         std::unique_ptr<Context<T>> subcontext,
                         std::unique_ptr<SystemOutput<T>> suboutput) {
    subcontext->set_parent(this);

    context_ = std::move(subcontext);
    output_ = std::move(suboutput);
    modal_subsystem_ = std::move(modal_subsystem);
  }

  /// Declares that a particular input port of a particular subsystem is an
  /// input to the entire Diagram that allocates this Context. Aborts if the
  /// subsystem has not been added to the DiagramContext.
  ///
  /// User code should not call this method. It is for use during Diagram
  /// context allocation only.
  void ExportInput(const PortIdentifier& port_id) {
    modal_subsystem_->
        get_mutable_input_port_ids()->emplace_back(port_id);
  }

  /// Generates the state vector for the entire diagram by wrapping the states
  /// of all the constituent diagrams.
  ///
  /// User code should not call this method. It is for use during
  /// HybridAutomaton context allocation only.
  //
  // TODO(jadecastro): This and DoClone has changed a bit in master... we'll
  // have to refactor it slightly.
  void MakeState() {
    //const int num_subsystems = static_cast<int>(contexts_.size());
    std::vector<AbstractValue*> hybrid_xm;

    Context<T>* subcontext = context_.get();
    state_ = subcontext->get_mutable_state();
    // Create discrete state.
    const std::vector<BasicVector<T>*>& hybrid_xd =
        subcontext->get_mutable_discrete_state()->get_data();
    // Create abstract state.
    AbstractState* subsystem_xm = subcontext->get_mutable_abstract_state();
    for (int i_xm = 0; i_xm < subsystem_xm->size(); ++i_xm) {
      hybrid_xm.push_back(&subsystem_xm->get_mutable_abstract_state(i_xm));
    }
    // Append another, final element corresponding to the modal subsystem of the
    // HA.
    const auto mss = std::unique_ptr<AbstractValue>(
        new Value<ModalSubsystem<T>>(modal_subsystem_.get()));
    hybrid_xm.push_back(mss.get());
    // TODO(jadecastro): Should we store this paricular index?

    // The wrapper states do not own the constituent state.
    this->set_discrete_state(std::make_unique<DiscreteState<T>>(hybrid_xd));
    this->set_abstract_state(std::make_unique<AbstractState>(hybrid_xm));

    // Create continuous state.
    if (subcontext->get_continuous_state() != nullptr) {
      const ContinuousState<T>& xc = *subcontext->get_continuous_state();
      const int num_q = xc.get_generalized_position().size();
      const int num_v = xc.get_generalized_velocity().size();
      const int num_z = xc.get_misc_continuous_state().size();
      const BasicVector<T>& xc_vector =
          dynamic_cast<const BasicVector<T>&>(xc.get_vector());
      this->set_continuous_state(std::make_unique<ContinuousState<T>>(
          xc_vector.Clone(), num_q, num_v, num_z));
    }
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

  /*
  // TODO(jadecastro): make the naming consistent.
  /// Returns the substate at @p index.
  State<T>* get_mutable_subsystem_state() {
    // TODO(jadecastro): Ensure contexts_ has the same cardinality as
    // substates_.
    const ModeId id = this->get_mode_id();
    return substates_[index];
  }
  */

  /// Recursively sets the time on this context and all subcontexts.
  // TODO: should we only advance time for the current active subsystem?
  void set_time(const T& time_sec) override {
    Context<T>::set_time(time_sec);
    const Context<T>* subcontext = GetMutableSubsystemContext();
    if (subcontext != nullptr) {
      subcontext->set_time(time_sec);
    }
  }

  int get_num_input_ports() const override {
    const ModalSubsystem<T> modal_subsystem = this->get_modal_subsystem();
    return modal_subsystems.get_num_input_ports();
  }

  void SetInputPort(int port_index, std::unique_ptr<InputPort> port) override {
    // TODO(jadecastro): DependentInputPort? (i.e. eval some output to get the
    // input?)
    const ModalSubsystem<T>* modal_subsystem = this->get_modal_subsystem();
    PortIdentifier subsystem_port_id =
        modal_subsystem->get_input_port_id(port_index);
    GetMutableSubsystemContext()
        ->SetInputPort(subsystem_port_id, std::move(port));
  }

  const ModalSubsystem<T>* get_modal_subsystem() const {
    //const int num_xm = state_.get_abstract_state()->size();
    //const AbstractState* xm = this->get_state().get_abstract_state();
    //const int size_xm = xm->size(); // TODO(jadecastro): Seems a bit
                                     //silly. Use back() instead?
    std::cerr << " continuous state: "
              << this->template
        get_continuous_state()->get_vector().GetAtIndex(0) << std::endl;
    std::cerr << " abstract state: "
              << this->template get_abstract_state<
                 ModalSubsystem<double>>(0).get_mode_id() << std::endl;
    return this->template get_abstract_state<ModalSubsystem<T>>(0);
  }

  ModeId get_mode_id() const {
    const ModalSubsystem<T> modal_subsystem = this->get_modal_subsystem();
    //DRAKE_DEMAND(modal_subsystem != nullptr);
    return modal_subsystem.get_mode_id();
  }

  // Mandatory overrides.
  const State<T>& get_state() const override { return state_; }

  State<T>* get_mutable_state() override { return &state_; }

 protected:
  HybridAutomatonContext<T>* DoClone() const override {
    HybridAutomatonContext<T>* clone
      = new HybridAutomatonContext(num_subsystems);

    // Clone all the subsystem contexts and outputs.  This basically repeats
    // everything in CreateDefaultContext.
    clone->RegisterSubsystem(
        modal_subsystem_->Clone(), context_->Clone(), output_->Clone());

    // Clone the external input structure.
    const ModalSubsystem<T>* modal_subsystem = get_modal_subsystem();
    for (int i = 0; i < modal_subsystem->get_num_input_ports(); ++i) {
      clone->ExportInput(modal_subsystem.get_input_port_id(i));
    }

    // Build the state for the initially-activated subsystem in the HA.
    clone->MakeState();

    // Make deep copies of everything else using the default copy constructors.
    *clone->get_mutable_step_info() = this->get_step_info();

    return clone;
  }

  /// Returns the input port at the given @p index, whose subsystem is derived
  /// from the context.
  const InputPort* GetInputPort(int port_index) const override {
    // TODO(jadecastro): DependentInputPort? (i.e. eval some output to get the
    // input?)
    const ModalSubsystem<T>* modal_subsystem = get_modal_subsystem();
    PortIdentifier subsystem_port_id =
        modal_subsystem->get_input_port_id(port_index);
    return Context<T>::GetInputPort(*GetSubsystemContext(),
                                    subsystem_port_id);
  }

 private:
  // TODO(jadecastro): Implement this.
  /*
  std::unique_ptr<symbolic::Variable>
  MakeSymbolicVariableFromState() {

  }
  */

  // The internal state of the System.
  State<T> state_;

  std::unique_ptr<SystemOutput<T>> outputs_;
  std::unique_ptr<Context<T>> contexts_;
  std::unique_ptr<ModalSubsystem<T>> modal_subsystem_;  // TODO(jadecastro):
                                                        // Need?
};

}  // namespace systems
}  // namespace drake
