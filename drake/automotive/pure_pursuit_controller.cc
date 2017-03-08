#include "drake/automotive/pure_pursuit_controller.h"

#include <cmath>

#include "drake/common/drake_assert.h"

namespace drake {

using systems::rendering::PoseVector;

namespace automotive {

static constexpr int kGoalPositionSize{2};

template <typename T>
PurePursuitController<T>::PurePursuitController() {
  // Declare the input port for the DrivingCommand request.
  this->DeclareInputPort(systems::kVectorValued,
                          DrivingCommandIndices::kNumCoordinates);
  // Declare the input port for the Cartesian goal position.
  this->DeclareInputPort(systems::kVectorValued, kGoalPositionSize);
  // Declare the input port for the ego car PoseVector.
  this->DeclareInputPort(systems::kVectorValued, PoseVector<T>::kSize);
  // Declare the DrivingCommand output port.
  this->DeclareOutputPort(systems::kVectorValued,
                          DrivingCommandIndices::kNumCoordinates);
}

template <typename T>
PurePursuitController<T>::~PurePursuitController() {}

template <typename T>
const systems::InputPortDescriptor<T>&
PurePursuitController<T>::driving_command_input() const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::InputPortDescriptor<T>&
PurePursuitController<T>::goal_position_input() const {
  return systems::System<T>::get_input_port(1);
}

template <typename T>
const systems::InputPortDescriptor<T>&
PurePursuitController<T>::ego_pose_input() const {
  return systems::System<T>::get_input_port(2);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
PurePursuitController<T>::driving_command_output() const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
void PurePursuitController<T>::DoCalcOutput(
    const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
  // Obtain the input/output data structures.
  const DrivingCommand<T>* const input_command =
      this->template EvalVectorInput<DrivingCommand>(
          context, this->driving_command_input().get_index());
  DRAKE_ASSERT(input_command != nullptr);

  const systems::BasicVector<T>* const goal_position =
      this->template EvalVectorInput<systems::BasicVector>(
          context, this->goal_position_input().get_index());
  DRAKE_ASSERT(goal_position != nullptr);

  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(
          context, this->ego_pose_input().get_index());
  DRAKE_ASSERT(ego_pose != nullptr);

  systems::BasicVector<T>* const command_output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(command_output_vector != nullptr);
  DrivingCommand<T>* const output_command =
      dynamic_cast<DrivingCommand<T>*>(command_output_vector);
  DRAKE_ASSERT(output_command != nullptr);

  ImplDoCalcOutput(*input_command, *goal_position, *ego_pose, output_command);
}

template <typename T>
void PurePursuitController<T>::ImplDoCalcOutput(
    const DrivingCommand<T>& input_command,
    const systems::BasicVector<T>& goal_position, const PoseVector<T>& ego_pose,
    DrivingCommand<T>* output_command) const {
  // Carrot-on-stick method:
  // Steering command is the change in heading angle with respect to the line
  // from the current position to the goal point.
  // TODO(jadecastro): Implement Pure-Pursuit.
  const T x = ego_pose.get_translation().translation().x();
  const T y = ego_pose.get_translation().translation().y();
  const T heading = ego_pose.get_rotation().z();
  const T steering_angle = std::atan2(goal_position[1] - y,
                                      goal_position[0] - x) - heading;

  // Pass the commanded throttle and acceleration through to the output,
  // applying the required steering angle.
  output_command->set_throttle(
      cond(input_command.throttle() < T(0.), T(0.), input_command.throttle()));
  output_command->set_brake(
      cond(input_command.brake() >= T(0.), T(0.), input_command.brake()));
  output_command->set_steering_angle(steering_angle);
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
PurePursuitController<T>::AllocateOutputVector(
    const systems::OutputPortDescriptor<T>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() == 0);
  return std::make_unique<DrivingCommand<T>>();
}

// These instantiations must match the API documentation in
// idm_planner.h.
template class PurePursuitController<double>;

}  // namespace automotive
}  // namespace drake
