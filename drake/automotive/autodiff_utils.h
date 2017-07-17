#pragma once

#include "drake/common/eigen_autodiff_types.h"
#include "drake/math/autodiff.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace automotive {
namespace autodiff {

/// Sets the partial derivatives of @p x to match the dimensions of @p
/// model_value, but whose settings are all zeros.
void SetZeroPartials(const AutoDiffXd& model_value, AutoDiffXd* x) {
  const int num_partials = model_value.derivatives().size();
  auto& derivs = (*x).derivatives();
  if (derivs.size() == 0) {
    derivs.resize(num_partials);
    derivs.setZero();
  }
}

void InitializeAutoDiffContext(systems::Context<AutoDiffXd>* context) {
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
  context->get_mutable_continuous_state()->SetFromVector(states_autodiff);
}

}  // namespace autodiff
}  // namespace automotive
}  // namespace drake
