#include "drake/automotive/trajectory_car.h"

namespace drake {
namespace automotive {

using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;
using trajectories::Trajectory;

template <typename T>
TrajectoryCar<T>::TrajectoryCar(
    const trajectories::Trajectory<double>& trajectory)
    : trajectory_(trajectory) {
  DRAKE_DEMAND(trajectory.cols() == 1);
  this->DeclareVectorOutputPort(&TrajectoryCar::CalcStateOutput);
  this->DeclareVectorOutputPort(&TrajectoryCar::CalcPoseOutput);
  this->DeclareVectorOutputPort(&TrajectoryCar::CalcVelocityOutput);
}

template <typename T>
void TrajectoryCar<T>::CalcStateOutput(const systems::Context<T>& context,
                                       SimpleCarState<T>* output_vector) const {
  const auto& raw_pose = trajectory_->value(context.get_time());
  ImplCalcStateOutput(raw_pose, output_vector);
}

template <typename T>
void TrajectoryCar<T>::CalcPoseOutput(
    const systems::Context<T>& context, PoseVector<T>* pose) const {
  const auto& raw_pose = trajectory_->value(context.get_time());
  pose->set_translation(raw_pose.head(3));
  pose->set_rotation(raw_pose.segment(3, PoseVector::kSize - 3));
}

template <typename T>
void TrajectoryCar<T>::CalcVelocityOutput(const systems::Context<T>& context,
                                          FrameVelocity<T>* velocity) const {
  const auto& raw_pose = trajectory_->value(context.get_time());
  velocity->set_velocity(raw_pose.tail(FrameVelocity::kSize));
}

template <typename T>
void TrajectoryCar<T>::ImplCalcStateOutput(const MatrixX<T>& raw_pose,
                                           SimpleCarState<T>* output) const {
  // Convert raw pose to output type.
  output->set_x(raw_pose[0]);
  output->set_y(raw_pose[1]);
  output->set_heading(raw_pose[2]);
  output->set_velocity(raw_pose[5]);
}

}  // namespace automotive
}  // namespace drake

template<>
class ::drake::automotive::TrajectoryCar<double>;
