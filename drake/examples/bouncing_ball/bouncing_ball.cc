#include "drake/examples/bouncing_ball/bouncing_ball.h"
#include "drake/systems/framework/hybrid_automaton.h"
#include "drake/systems/framework/hybrid_automaton_builder.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace bouncing_ball {

template <typename T>
//BouncingBall<T>::BouncingBall(const ModalSubsystem& mss) :
//  systems::HybridAutomaton<T>(), ball_subsystem_(mss) {
BouncingBall<T>::BouncingBall() : systems::HybridAutomaton<T>() {
//BouncingBall<T>::BouncingBall(const ModalSubsystem& mss) :
//  ball_subsystem_(mss) {

  systems::HybridAutomatonBuilder<T> builder;

  ball_subsystem_ = builder.AddModalSubsystem(
    std::unique_ptr<Ball<T>>());
  //symbolic::Expression x = get_symbolic_state_vector(ball_subsystem_);
  symbolic::Formula invariant_formula_ball = symbolic::Formula::True();
  builder.AddInvariant(ball_subsystem_, invariant_formula_ball);

  ball_to_ball_ = builder.AddModeTransition(*ball_subsystem_);
  //symbolic::Formula guard_formula_bounce = symbolic::Formula::True();
  // TODO: add the guard.
  // TODO: add the reset map.

}

template <typename T>
bool BouncingBall<T>::has_any_direct_feedthrough() const {
  return false;
}

template class BouncingBall<double>;
template class BouncingBall<AutoDiffXd>;

}  // namespace bouncing_ball
}  // namespace drake
