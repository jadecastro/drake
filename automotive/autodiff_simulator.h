#pragma once

#include "drake/automotive/lane_direction.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/math/autodiff.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/value.h"

namespace drake{
namespace automotive {

namespace {

// Helper that populates the state vector derivatives to the identity matrix of
// appropriate dimension, akin to math::InitializeAutoDiff.
// TODO(jadecastro) Graduate this to Drake master, if it proves useful.
void InitializeAutoDiffContext(systems::DiagramContext<AutoDiffXd>* context) {
  const auto& states = context->get_continuous_state_vector();
  const int num_states = states.size();
  Eigen::VectorXd context_vector(num_states + 1);  // time + states.
  for (int i{0}; i < num_states; ++i) {
    context_vector(i) = states.GetAtIndex(i).value();
  }
  context_vector(num_states) = 0.;  // initial time

  const auto autodiff_context_vector = math::initializeAutoDiff(context_vector);

  const AutoDiffXd time_autodiff = autodiff_context_vector(num_states);
  context->set_time(time_autodiff);
  const auto states_autodiff = autodiff_context_vector.segment(0, num_states);
  context->get_mutable_continuous_state().SetFromVector(states_autodiff);
}

const systems::Diagram<AutoDiffXd>& ToDiagram(
    const systems::System<AutoDiffXd>& system) {
  return dynamic_cast<const systems::Diagram<AutoDiffXd>&>(system);
}

systems::DiagramContext<AutoDiffXd>& ToDiagramContext(
    systems::Context<AutoDiffXd>& context) {
  return dynamic_cast<systems::DiagramContext<AutoDiffXd>&>(context);
}

}  // namespace


// AutoDiffXd curtainwall for simulating pybind systems defined in C++ but
// instantiated in python with AutoDiffXd.  Transforms the Diagram into one of
// type AutoDiffXd, instantiates a Simulator, and exposes StepTo() as well as
// setters and getters for the continuous state vector for the diagram and
// subsystems.
class AutodiffSimulator {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AutodiffSimulator)

  AutodiffSimulator(const systems::Diagram<double>& system)
      : system_(system.ToAutoDiffXd()),
        simulator_(std::make_unique<systems::Simulator<AutoDiffXd>>(
            ToDiagram(*system_), ToDiagram(*system_).CreateDefaultContext())),
    // Defer context initialization.
        context_(ToDiagramContext(simulator_->get_mutable_context())) {
    // Declare the partial derivatives for all of the context members.
    InitializeAutoDiffContext(&context_);

    // Initialize the Simulator.
    const double kSimulatorTimeStep = 0.1;
    simulator_->set_target_realtime_rate(0.);
    //    simulator_->get_mutable_integrator()->set_requested_minimum_step_size(
    //    kSimulatorTimeStep);
    simulator_->get_mutable_integrator()->set_maximum_step_size(
        kSimulatorTimeStep);
    simulator_->get_mutable_integrator()->set_fixed_step_mode(true);
    simulator_->Initialize();
  }

  void SetSubsystemState(int index, const std::vector<AutoDiffXd>& values) {
    const systems::Diagram<AutoDiffXd>& diagram = ToDiagram(*system_);
    const systems::System<AutoDiffXd>& subsystem = *diagram.GetSystems()[index];
    systems::VectorBase<AutoDiffXd>& state =
        diagram.GetMutableSubsystemContext(subsystem, &context_)
        .get_mutable_continuous_state_vector();
    DRAKE_DEMAND(static_cast<int>(values.size()) == state.size());
    for (int i{0}; i < state.size(); i++) {
      state.GetAtIndex(i) = values[i];
    }
  }

  void SetState(const std::vector<AutoDiffXd>& values) {
    systems::VectorBase<AutoDiffXd>& state =
        context_.get_mutable_continuous_state_vector();
    DRAKE_DEMAND(static_cast<int>(values.size()) == state.size());
    for (int i{0}; i < state.size(); i++) {
      state.GetAtIndex(i) = values[i];
    }
  }

  const std::vector<AutoDiffXd> GetSubsystemState(int index) const {
    const systems::Diagram<AutoDiffXd>& diagram = ToDiagram(*system_);
    const systems::System<AutoDiffXd>& subsystem = *diagram.GetSystems()[index];
    const systems::VectorBase<AutoDiffXd>& state =
        diagram.GetSubsystemContext(subsystem, context_)
        .get_continuous_state_vector();
    std::vector<AutoDiffXd> result(state.size());
    // TODO: Faster way of doing this?
    for (int i{0}; i < state.size(); i++) {
      result[i] = state.GetAtIndex(i);
    }
    return result;
  }

  const std::vector<AutoDiffXd> GetState() const {
    const systems::VectorBase<AutoDiffXd>& state =
        context_.get_continuous_state_vector();
    std::vector<AutoDiffXd> result(state.size());
    // TODO: Faster way of doing this?
    for (int i{0}; i < state.size(); i++) {
      result[i] = state.GetAtIndex(i);
    }
    return result;
  }

  void FixInputPort(
      int index, std::unique_ptr<systems::BasicVector<AutoDiffXd>> vector) {
    context_.FixInputPort(index, std::move(vector));
  }

  void FixInputPort(int index, std::unique_ptr<systems::AbstractValue> value) {
    context_.FixInputPort(index, std::move(value));
  }

  void StepTo(double time_step) {
    simulator_->StepTo(time_step);
  }

private:
  std::unique_ptr<systems::System<AutoDiffXd>> system_;
  std::unique_ptr<systems::Simulator<AutoDiffXd>> simulator_;
  systems::DiagramContext<AutoDiffXd>& context_;
};

}  // namespace automotive
}  // namespace drake
