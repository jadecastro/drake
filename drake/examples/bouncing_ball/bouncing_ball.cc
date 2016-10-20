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

  systems::ModalSubsystemBuilder<T> modal_subsystem_builder;

  ball_subsystem_ = modal_subsystem_builder.AddModalSubsystem(
    std::unique_ptr<Ball<T>>());
  //ball_.add_invariant(int 5);  // test the invariant-addition approach by
  // simply pushing back an int.

  systems::HybridAutomatonBuilder<T> hybrid_automaton_builder;

}

template <typename T>
bool BouncingBall<T>::has_any_direct_feedthrough() const {
  return false;
}

template class BouncingBall<double>;
template class BouncingBall<AutoDiffXd>;

}  // namespace bouncing_ball
}  // namespace drake
