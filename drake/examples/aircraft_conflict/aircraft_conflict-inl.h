#pragma once

/// @file
/// Template method implementations for bouncing_ball.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/examples/bouncing_ball/bouncing_ball.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace aircraft_conflict {

template <typename T>
AircraftConflictResolution<T>::AircraftConflictResolution() {
  // TODO: explain the plant model (two aircraft with same controller, etc.)

  // Register two modes: cruise and avoid.
  auto cruise_mode_ = hybrid_automaton.AddMode<RelativeDubins>(1.0);
  cruise_mode_.add_invariant(SymbolicExpression& invar_c);  // invariant is
  // infinite, unless specified with the add_invariant call.
  cruise_mode_.add_initial_states(SymbolicExpression& init_c);  // initial
  // states are infinite unless specified with the add_intial_states call.

  auto avoid_mode_ = hybrid_automaton.AddMode<RelativeDubins>(0.0);
  avoid_mode_.add_invariant(SymbolicExpression& invar_a);  // invariant is
  // infinite, unless specified with the add_invariant call.
  avoid_mode_.add_initial_states(SymbolicExpression& init_a);  // initial
  // states are infinite unless specified with the add_intial_states call.

  // Add two transitions between the registered modes.
  auto cruise_avoid_transition_ =
    hybrid_automaton.AddModeTransition(cruise_mode_, avoid_mode_);
  cruise_avoid_transition_.set_guard(SymbolicExpression& guard_c_a);
  cruise_avoid_transition_.set_reset(SymbolicExpression& reset_c_a);

  auto avoid_cruise_transition_ =
    hybrid_automaton.AddModeTransition(avoid_mode_, cruise_mode_);
  avoid_cruise_transition_.set_guard(SymbolicExpression& guard_a_c);
  avoid_cruise_transition_.set_reset(SymbolicExpression& reset_a_c);
}

template <typename T>
  T AircraftConflictResolution<T>::EvalGuard(
std::pair<const systems::System<T>, const systems::System<T>> edge,
                                             const systems::Context<T>& context)
  const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Evaluate the guard function.
  const systems::VectorBase<T>& state =
    context.get_continuous_state()->get_state();

  // The guard is satisfied (returns a non-positive value) when
  // the ball's position is less than or equal to zero and its
  // velocity is non-positive.
  return std::max(state.GetAtIndex(0), state.GetAtIndex(1));
}

template <typename T>
void AircraftConflictResolution<T>::PerformReset(systems::Context<T>* context)
  const {
  DRAKE_ASSERT(context != nullptr);
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(*context));

  // Define a pointer to the continuous state in the context.
  const auto result =
    context->get_mutable_continuous_state()->get_mutable_state();

  // Perform the reset: map the position to itself and negate the
  // velocity and attenuate by the coefficient of restitution.
  auto state = context->get_mutable_continuous_state()->get_mutable_state();
  result->SetAtIndex(1,
     -1.0 * this->restitution_coef_ * state->GetAtIndex(1));
}

}  // namespace aircraft_conflict
}  // namespace drake
