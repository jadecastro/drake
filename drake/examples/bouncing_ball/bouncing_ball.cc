#include "drake/examples/bouncing_ball/bouncing_ball.h"
#include "drake/systems/framework/diagram_tester.h"
#include "drake/systems/framework/diagram_builder_tester.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace bouncing_ball {

template <typename T>
BouncingBall<T>::BouncingBall() : systems::DiagramTester<T>() {

  systems::DiagramBuilderTester<T> builder;

  ball_ = builder.template AddModalSubsystem<Ball>();
  //ball_.add_invariant(int 5);  // test the invariant-addition approach by
  // simply pushing back an int.

}

template <typename T>
bool BouncingBall<T>::has_any_direct_feedthrough() const {
  return false;
}

template class BouncingBall<double>;
template class BouncingBall<AutoDiffXd>;

}  // namespace bouncing_ball
}  // namespace drake
