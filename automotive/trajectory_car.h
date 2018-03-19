#pragma once

#include "drake/automotive/gen/simple_car_state.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/vector_base.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// TrajectoryCar models a car that follows a pre-established trajectory given
/// by a PiecewisePolynomial on PoseVector and FrameVelocity.
///
/// output port 0:
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
///   heading is defined around the +z axis, so positive-turn-left
/// * velocity
///   (OutputPort getter: raw_pose_output())
///
/// output port 1: A PoseVector containing X_WC, where C is the car frame.
///   (OutputPort getter: pose_output())
///
/// output port 2: A FrameVelocity containing Xdot_WC, where C is the car frame.
///   (OutputPort getter: velocity_output())
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_plants
template <typename T>
class TrajectoryCar final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryCar)

  /// Constructs a TrajectoryCar system that follows a given trajectory of
  /// recorded poses and velocities.
  explicit TrajectoryCar(const trajectories::Trajectory<double>& trajectory);

  /// See class description for details about the following ports.
  /// @{
  const systems::OutputPort<T>& state_output() const {
    return this->get_output_port(0);
  }
  const systems::OutputPort<T>& pose_output() const {
    return this->get_output_port(1);
  }
  const systems::OutputPort<T>& velocity_output() const {
    return this->get_output_port(2);
  }
  /// @}

 private:
  void CalcStateOutput(const systems::Context<T>& context,
                       SimpleCarState<T>* output_vector) const;

  void CalcPoseOutput(const systems::Context<T>& context,
                      systems::rendering::PoseVector<T>* pose) const;
  void CalcVelocityOutput(
      const systems::Context<T>& context,
      systems::rendering::FrameVelocity<T>* velocity) const;

  // Eigen::Ref here??
  void ImplCalcStateOutput(const MatrixX<T>& raw_pose,
                           SimpleCarState<T>* output) const;

  const trajectories::Trajectory<T> trajectory_;
};

}  // namespace automotive
}  // namespace drake
