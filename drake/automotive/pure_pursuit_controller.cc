#include "drake/automotive/pure_pursuit_controller.h"

#include <cmath>

#include "drake/common/drake_assert.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/pose_selector.h"
#include "drake/automotive/simple_car.h"

namespace drake {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::Junction;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using maliput::api::Segment;
using systems::rendering::PoseVector;

namespace automotive {

static constexpr int kCarParamsIndex{0};
static constexpr int kPpParamsIndex{1};

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
  const PurePursuitControllerParameters<T>& pp_params =
      this->template GetNumericParameter<PurePursuitControllerParameters>(
          context, kPpParamsIndex);
  const SimpleCarConfig<T>& car_params =
      this->template GetNumericParameter<SimpleCarConfig>(context,
                                                          kCarParamsIndex);

  // Obtain the input/output data structures.
  const DrivingCommand<T>* const input_command =
      this->template EvalVectorInput<DrivingCommand>(
          context, this->driving_command_input().get_index());
  DRAKE_ASSERT(input_command != nullptr);

  const maliput::api::LaneId* lane_id =
      this->template EvalInputValue<maliput::api::LaneId>(
          context, this->lane_input().get_index());

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

  ImplDoCalcOutput(pp_params, car_params, *input_command, *lane_id, *ego_pose,
                   output_command);
}

template <typename T>
void PurePursuitController<T>::ImplDoCalcOutput(
    const PurePursuitControllerParameters<T>& pp_params,
    const SimpleCarConfig<T>& car_params,
    const DrivingCommand<T>& input_command,
    const maliput::api::LaneId& lane_id, const PoseVector<T>& ego_pose,
    DrivingCommand<T>* output_command) const {
  const T x = ego_pose.get_translation().translation().x();
  const T y = ego_pose.get_translation().translation().y();
  const T heading = ego_pose.get_rotation().z();

  const Lane* lane = GetLane(lane_id);
  DRAKE_DEMAND(lane != nullptr);
  const RoadPosition& ego_position =
      pose_selector::CalcRoadPosition(*road_, ego_pose.get_isometry());
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
const Lane* PurePursuitController<T>::GetLane(
    const maliput::api::LaneId& lane_id) const {
  // Exhaustive searching is ridiculous.
  const Lane* lane{nullptr};
  for (int i = 0; i < road_->num_junctions(); ++i) {
    const Junction* junction = road_->junction(i);
    for (int j = 0; j < junction->num_segments(); ++j) {
      const Segment* segment = junction->segment(j);
      for (int k = 0; k < segment->num_lanes(); ++k) {
        lane = segment->lane(k);
        if (lane->id().id == lane_id.id) return lane;
      }
    }
  }
  return lane;
}

template <typename T>
std::unique_ptr<systems::Parameters<T>>
PurePursuitController<T>::AllocateParameters() const {
  std::vector<std::unique_ptr<systems::BasicVector<T>>> params;
  params.insert(params.begin() + kPpParamsIndex,
                std::make_unique<PurePursuitControllerParameters<T>>());
  params.insert(params.begin() + kCarParamsIndex,
                std::make_unique<SimpleCarConfig<T>>());
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

template <typename T>
void PurePursuitController<T>::SetDefaultParameters(
    const systems::LeafContext<T>& context, systems::Parameters<T>* params)
    const {
  PurePursuitControllerParameters<T>* pp_params =
      dynamic_cast<PurePursuitControllerParameters<T>*>(
          params->get_mutable_numeric_parameter(kPpParamsIndex));
  DRAKE_DEMAND(pp_params != nullptr);
  pp_params->set_s_lookahead(T(15.));  // lookahead distance [m].

  SimpleCarConfig<T>* config = dynamic_cast<SimpleCarConfig<T>*>(
      params->get_mutable_numeric_parameter(kCarParamsIndex));
  DRAKE_DEMAND(config != nullptr);
  SimpleCar<T>::SetDefaultParameters(config);
}

// These instantiations must match the API documentation in
// pure_pursuit_controller.h.
template class PurePursuitController<double>;

}  // namespace automotive
}  // namespace drake
