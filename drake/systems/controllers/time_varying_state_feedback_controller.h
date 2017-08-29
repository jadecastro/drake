#pragma once

#include "drake/systems/primitives/affine_system.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"

namespace drake {
namespace systems {

/// A SISO system implementation of the time-varying state-feedback controller:
///
/// @f[ u(t) = u0(t) - K(t)(x(t) - x0(t)) @f]
///
// TODO(jadecastro) Generalize to schedule against one of the state variables or
// an auxilliary 1D signal brought in via an additional input port rather than
// time.  This might involve deriving from something other than
// TimeVaryingAffineSystem.
template <typename T>
class TimeVaryingStateFeedbackController : public TimeVaryingAffineSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimeVaryingStateFeedbackController)

  // TODO(jadecastro) Implement a discrete-time version of trajectories.
  TimeVaryingStateFeedbackController(
      const PiecewisePolynomialTrajectory& K,
      const PiecewisePolynomialTrajectory& x0 =
      PiecewisePolynomialTrajectory(PiecewisePolynomial<double>()),
      const PiecewisePolynomialTrajectory& u0 =
      PiecewisePolynomialTrajectory(PiecewisePolynomial<double>()),
      double time_step = 0.)
  : TimeVaryingAffineSystem(0, x0.rows(), u0.cols(), time_step) {
    if (x0.rows() > 0) {
      DRAKE_DEMAND(x0.cols() == 1);
      DRAKE_DEMAND(u0.rows() > 0 && u0.cols() == 1);
      DRAKE_DEMAND(K.cols() == x0.rows() && K.rows() == u0.rows());
      DRAKE_DEMAND(K.get_start_time() == u0.get_start_time() &&
                   K.get_start_time() == x0.get_start_time());
      DRAKE_DEMAND(K.get_end_time() == u0.get_end_time() &&
                   K.get_end_time() == x0.get_end_time());
    }
    K_ = K;
    x0_ = x0;
    u0_ = u0;

    // TODO(jadecastro) Warn if we're feeding in a time_step that's inconsistent
    // with the breaks.
  }

  ~TimeVaryingStateFeedbackController() override {}

  MatrixX<T> A(const T& t) const override {
    return Eigen::Matrix<T, 0, 0>::Zero();
  }
  MatrixX<T> B(const T& t) const override {
    return Eigen::MatrixXd::Zero(0, this->num_inputs());
  }
  VectorX<T> f0(const T& t) const override {
    return Eigen::Matrix<T, 0, 1>::Zero();
  }
  MatrixX<T> C(const T& t) const override {
    return Eigen::MatrixXd::Zero(this->num_outputs(), 0);
  }
  MatrixX<T> D(const T& t) const override {
    DRAKE_DEMAND(t >= K_.get_start_time() && t <= K_.get_end_time());
    return -K_.value(t);
  }
  MatrixX<T> y0(const T& t) const override {
    DRAKE_DEMAND(t >= K_.get_start_time() && t <= K_.get_end_time());
    return u0_.value(t) + K_.value(t) * x0_.value(t);
  }

 private:
  PiecewisePolynomialTrajectory K_;
  PiecewisePolynomialTrajectory x0_;
  PiecewisePolynomialTrajectory u0_;
};

// TODO(jadecastro) If AutoDiffXd is needed (I cannot think of a use case)
// PiecewisePolynomialTrajectory is the bottleneck here -- the fix is to open it
// up to additional scalar types or go with another container.
template class TimeVaryingStateFeedbackController<double>;

}  // namespace systems
}  // namespace drake
