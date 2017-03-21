#include "drake/automotive/pure_pursuit_controller.h"

#include <cmath>

#include "drake/common/drake_assert.h"
#include "drake/automotive/simple_car.h"

namespace drake {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using systems::rendering::PoseVector;

namespace automotive {

static constexpr int kGoalPositionSize{2};

template <typename T>
PurePursuitController<T>::PurePursuitController(
    std::unique_ptr<RoadGeometry> road)
    : road_(std::move(road)) {
  // Declare the input port for the DrivingCommand request.
  this->DeclareInputPort(systems::kVectorValued,
                         DrivingCommandIndices::kNumCoordinates);
  // Declare the input port for the lane request.
  this->DeclareAbstractInputPort();
  // Declare the input port for the ego car PoseVector.
  this->DeclareInputPort(systems::kVectorValued, PoseVector<T>::kSize);
  // Declare the DrivingCommand output port.
  this->DeclareVectorOutputPort(DrivingCommand<T>());
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
PurePursuitController<T>::lane_input() const {
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
  // Obtain the parameters.
  const SimpleCarConfig<T>& config =
      this->template GetNumericParameter<SimpleCarConfig>(context, 0);

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

  ImplDoCalcOutput(config, *input_command, *goal_position, *ego_pose,
                   output_command);
}

template <typename T>
void PurePursuitController<T>::ImplDoCalcOutput(
    const PurePursuitControllerParameters<T>& pp_params,
    const SimpleCarConfig<T>& car_params,
    const DrivingCommand<T>& input_command,
    const LaneId& lane_id, const PoseVector<T>& ego_pose,
    DrivingCommand<T>* output_command) const {
  const T x = ego_pose.get_translation().translation().x();
  const T y = ego_pose.get_translation().translation().y();
  const T heading = ego_pose.get_rotation().z();

  const Lane* lane = road_.GetLane(lane_id);  // <--- incomplete.
  const GeoPosition goal_position =
     ComputeGoalPoint(pp_params, lane, ego_position);

  // Pure-pursuit method.
  const T s_lookahead = goal_position.x - x;
  const T delta_y = -s_lookahead * sin(heading) +
      (goal_position.y - y) * cos(heading);
  const T curvature = 2 * delta_y / pow(s_lookahead, 2.);
  const T steering_angle = atan(car_params.wheelbase() * curvature);

  // Pass the commanded throttle and acceleration through to the output,
  // applying the required steering angle.
  output_command->set_throttle(
      cond(input_command.throttle() < T(0.), T(0.), input_command.throttle()));
  output_command->set_brake(
      cond(input_command.brake() >= T(0.), T(0.), input_command.brake()));
  output_command->set_steering_angle(steering_angle);
}

template <typename T>
const GeoPosition PurePursuitController<T>::ComputeGoalPoint(
    const PurePursuitControllerParameters<T>& params, const Lane* lane,
    const RoadPosition& position) const {
  const LanePosition lane_position(position.pos.s + params.s_lookahead(), 0, 0);
  return lane->ToGeoPosition(lane_position);
}

template <typename T>
std::unique_ptr<systems::Parameters<T>>
PurePursuitController<T>::AllocateParameters() const {
  std::vector<std::unique_ptr<systems::BasicVector<T>>> params;
  params.push_back(std::make_unique<PurePursuitControllerParameters<T>>());
  params.push_back(std::make_unique<SimpleCarConfig<T>>());
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

template <typename T>
void PurePursuitController<T>::SetDefaultParameters(
    const systems::LeafContext<T>& context, systems::Parameters<T>* params)
    const {
  PurePursuitControllerParameters<T>* pp_params =
      dynamic_cast<PurePursuitControllerParameters<T>*>(
          params->get_mutable_numeric_parameter(0));
  DRAKE_DEMAND(pp_params != nullptr);

  SimpleCarConfig<T>* config = dynamic_cast<SimpleCarConfig<T>*>(
      params->get_mutable_numeric_parameter(1));
  DRAKE_DEMAND(config != nullptr);
  SimpleCar<T>::SetDefaultParameters(config);
}

// These instantiations must match the API documentation in
// idm_planner.h.
template class PurePursuitController<double>;

}  // namespace automotive
}  // namespace drake
