#include "drake/automotive/pure_pursuit_controller.h"

#include <cmath>

#include "drake/common/drake_assert.h"
#include "drake/automotive/simple_car.h"

namespace drake {

using systems::rendering::PoseVector;

namespace automotive {

static constexpr int kGoalPositionSize{2};

template <typename T>
PurePursuitController<T>::PurePursuitController() {
  // Declare the input port for the DrivingCommand request.
  command_input_index_ = this->DeclareInputPort(
      systems::kVectorValued, DrivingCommandIndices::kNumCoordinates)
      .get_index();
  // Declare the input port for the desired LaneDirection.
  lane_index_ = this->DeclareAbstractInputPort().get_index();
  // Declare the input port for the ego car PoseVector.
  ego_pose_index_ =
      this->DeclareInputPort(systems::kVectorValued, PoseVector<T>::kSize)
      .get_index();
  // Declare the DrivingCommand output port.
  command_output_index_ =
      this->DeclareOutputPort(DrivingCommand<T>()).get_index();

  this->DeclareNumericParameter(PurePursuitControllerParams);
  this->DeclareNumericParameter(SimpleCarParams);
}

template <typename T>
PurePursuitController<T>::~PurePursuitController() {}

template <typename T>
const systems::InputPortDescriptor<T>&
PurePursuitController<T>::driving_command_input() const {
  return systems::System<T>::get_input_port(command_input_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>&
PurePursuitController<T>::lane_input() const {
  return systems::System<T>::get_input_port(lane_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>&
PurePursuitController<T>::ego_pose_input() const {
  return systems::System<T>::get_input_port(ego_pose_index_);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
PurePursuitController<T>::driving_command_output() const {
  return systems::System<T>::get_output_port(command_output_index_);
}

template <typename T>
void PurePursuitController<T>::DoCalcOutput(
    const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
  // Obtain the parameters.
  const PurePursuitControllerParams<T>& pp_params =
      this->template GetNumericParameter<PurePursuitControllerParams>(
          context, 0);
  const SimpleCarConfig<T>& car_params =
      this->template GetNumericParameter<SimpleCarParams>(context, 1);

  // Obtain the input/output data structures.
  const DrivingCommand<T>* const input_command =
      this->template EvalVectorInput<DrivingCommand>(
          context, this->driving_command_input().get_index());
  DRAKE_ASSERT(input_command != nullptr);

  const LaneDirection* const lane_direction =
       this->template EvalInputValue<LaneDirection>(
           context, this->lane_input().get_index());
  DRAKE_ASSERT(lane_direction != nullptr);

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

  ImplDoCalcOutput(pp_params, car_params, *input_command, *lane_direction,
                   *ego_pose, output_command);
}

template <typename T>
void PurePursuitController<T>::ImplDoCalcOutput(
    const PurePursuitControllerParams<T>& pp_params,
    const SimpleCarParams<T>& car_params,
    const DrivingCommand<T>& input_command,
    const LaneDirection& lane_direction, const PoseVector<T>& ego_pose,
    DrivingCommand<T>* output_command) const {
  // TODO(jadecastro): Replace with ToLanePosition?
  const T x = ego_pose.get_translation().translation().x();
  const T y = ego_pose.get_translation().translation().y();
  const T heading = ego_pose.get_rotation().z();

  const GeoPosition goal_position = ComputeGoalPoint(
      pp_params, lane_direction.lane,
      pose_selector::CalcRoadPosition(road_, ego_pose.get_isometry()));
  // Carrot-on-stick method.
  // Steering command is the change in heading angle with respect to the line
  // from the current position to the goal point.
  //const T steering_angle = std::atan2(goal_position[1] - y,
  //                                    goal_position[0] - x) - heading;

  // Pure-pursuit method.
  // TODO(jadecastro): Make a function, just like IdmPlanner.
  const T delta_y = -(goal_position[0] - x) * sin(heading) +
      (goal_position[1] - y) * cos(heading);
  const T lookahead = goal_position[0] - x;
  const T curvature = 2 * delta_y / pow(lookahead, 2.);
  const T steering_angle = atan(car_params.wheelbase() * curvature);

  // Apply the required steering angle.
  output_command->set_steering_angle(steering_angle);
  // Pass the acceleration through to the output,
  output_command->set_acceleration(input_command.acceleration());
}

template <typename T>
const GeoPosition MobilPlanner<T>::ComputeGoalPoint(
    const PurePursuitControllerParams<T>& pp_params,
    const LaneDirection* lane_direction,
    const RoadPosition& position) const {
  const LanePosition lane_position(position.pos.s + params.s_lookahead(), 0, 0);
  return lane_direction.lane->ToGeoPosition(lane_position);
}

// These instantiations must match the API documentation in
// idm_planner.h.
template class PurePursuitController<double>;

}  // namespace automotive
}  // namespace drake
