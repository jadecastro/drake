#include "drake/examples/bouncing_ball/bouncing_ball.h"
#include "drake/systems/framework/hybrid_test.h"
#include "drake/systems/framework/hybrid_builder_test.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace bouncing_ball {

template <typename T>
BouncingBall<T>::BouncingBall() : systems::HybridAutomaton<T>() {

  systems::HybridAutomatonBuilder<T> builder;

  ball_ = builder.template AddModalSubsystem<Ball>();
}

template <typename T>
bool BouncingBall<T>::has_any_direct_feedthrough() const {
  return false;
}

template class BouncingBall<double>;
template class BouncingBall<AutoDiffXd>;

}  // namespace bouncing_ball
}  // namespace drake
