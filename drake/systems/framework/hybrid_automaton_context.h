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

// Debugging.
#include "drake/systems/framework/test_utilities/pack_value.h"

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
        initial_conditions_(initial_conditions) {}

  explicit ModalSubsystem(
      const ModeId mode_id, const System<T>* system,
      std::vector<PortIdentifier> input_port_ids,
      std::vector<PortIdentifier> output_port_ids)
      : mode_id_(mode_id), system_(system), input_port_ids_(input_port_ids),
        output_port_ids_(output_port_ids){}

  explicit ModalSubsystem(
      const ModeId mode_id, const System<T>* system)
      : mode_id_(mode_id), system_(system) {}

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
  std::vector<PortIdentifier>* get_mutable_output_port_ids() {
    return &output_port_ids_;
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
    DRAKE_DEMAND(clone != nullptr);
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
};

/// The HybridAutomatonContext is a container for all of the data necessary to
/// uniquely determine the computations performed by a HybridAutomaton (HA).
/// Specifically, a HybridAutomatonContext contains contexts and outputs for all
/// modal subsystems in the finite-state machine. In addition, it augments the
/// subsystem contexts with an abstract (modal) state designating the current
/// active ModalSubsystem.
///
/// In the context, the size of the state vector may change, but the inputs and
/// outputs must be of fixed size.  If ModalSubsystems have inconsistent
/// input/output dimensions, this is reconciled via ExportInput and
/// ExportOutput.
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
  explicit HybridAutomatonContext() {}

  // ADD COMMENTS
  // ***** Call this something else; registered implies we only do it once.. we
  // actually call this every time a new mode is triggered.
  void RegisterSubsystem(std::unique_ptr<ModalSubsystem<T>> modal_subsystem,
                         std::unique_ptr<Context<T>> subcontext,
                         std::unique_ptr<SystemOutput<T>> suboutput) {
    subcontext->set_parent(this);

    context_ = std::move(subcontext);
    output_ = std::move(suboutput);
    modal_subsystem_ = std::move(modal_subsystem);
  }

  void DeRegisterSubsystem() {
    context_.release();
    output_.release();
    modal_subsystem_.release();
  }

  /// Declares that a particular input port of a particular subsystem is an
  /// input to the entire HA that allocates this Context.
  ///
  /// User code should not call this method. It is for use during HA context
  /// allocation only.
  void ExportInput(const PortIdentifier& port_id) {
    modal_subsystem_->get_mutable_input_port_ids()->emplace_back(port_id);
  }

  /// Declares that a particular input port of a particular subsystem is an
  /// input to the entire HA that allocates this Context.
  ///
  /// User code should not call this method. It is for use during HA context
  /// allocation only.
  void ExportOutput(const PortIdentifier& port_id) {
    modal_subsystem_->get_mutable_output_port_ids()->emplace_back(port_id);
  }

  /// Generates the state vector for the HA.
  ///
  /// User code should not call this method. It is for use during
  /// HybridAutomaton context allocation only.
  //
  // TODO(jadecastro): This and DoClone has changed a bit in master... we'll
  // have to refactor it slightly.
  void MakeState() {
    Context<T>* subcontext = context_.get();
    DRAKE_DEMAND(subcontext != nullptr);
    state_ = subcontext->get_mutable_state();

    // TODO(jadecastro): Do we need these, or does SetFrom do all of this for
    // us?
    // Create discrete state.
    const std::vector<BasicVector<T>*>& hybrid_xd =
        subcontext->get_mutable_discrete_state()->get_data();
    // Create abstract state.
    AbstractState* subsystem_xm = subcontext->get_mutable_abstract_state();
    std::vector<std::unique_ptr<AbstractValue>> hybrid_xm;
    for (int i_xm = 0; i_xm < subsystem_xm->size(); ++i_xm) {
      hybrid_xm.push_back(
          subsystem_xm->get_mutable_abstract_state(i_xm).Clone());
    }
    hybrid_xm.push_back(std::unique_ptr<AbstractValue>(new Value<int>(
        modal_subsystem_->get_mode_id())));

    // The wrapper states do not own the constituent state.
    this->set_discrete_state(std::make_unique<DiscreteState<T>>(hybrid_xd));
    this->set_abstract_state(std::make_unique<AbstractState>(
        std::move(hybrid_xm)));

    // Create the continuous state.
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
    //std::cerr << " MakeState: Abstract state: "
    //          << this->template get_abstract_state<int>(0) << std::endl;
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

  void SetInputPort(int port_index, std::unique_ptr<InputPort> port) override {
    // TODO(jadecastro): DependentInputPort? (i.e. eval some output to get the
    // input?)
    const ModalSubsystem<T>* modal_subsystem = GetModalSubsystem();
    DRAKE_DEMAND(modal_subsystem != nullptr);
    PortIdentifier subsystem_port_id =
        modal_subsystem->get_input_port_id(port_index);
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
    int mss = this->template get_mutable_abstract_state<int>(mss_index);
    mss = GetModalSubsystem()->get_mode_id();
  }

  // Mandatory overrides.
  const State<T>& get_state() const override { return *state_; }

  State<T>* get_mutable_state() override { return state_; }

 protected:
  HybridAutomatonContext<T>* DoClone() const override {
    HybridAutomatonContext<T>* clone = new HybridAutomatonContext();

    // Clone all the subsystem contexts and outputs.  This basically repeats
    // everything in CreateDefaultContext.
    clone->RegisterSubsystem(
        modal_subsystem_->Clone(), context_->Clone(), output_->Clone());

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
    const ModalSubsystem<T>* modal_subsystem = GetModalSubsystem();
    DRAKE_DEMAND(modal_subsystem != nullptr);
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
  State<T>* state_ = nullptr;

  std::unique_ptr<SystemOutput<T>> output_;
  std::unique_ptr<Context<T>> context_;
  std::unique_ptr<ModalSubsystem<T>> modal_subsystem_;  // TODO(jadecastro):
                                                        // Need?
};

}  // namespace systems
}  // namespace drake
