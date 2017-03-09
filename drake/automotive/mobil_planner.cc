#include "drake/automotive/mobil_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/pose_selector.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"

namespace drake {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

namespace automotive {

static constexpr int kGoalPositionSize{2};
static constexpr double kLargeAcceleration{1e6};  // in units [m / s^2].
static constexpr double kEpsilonDistance{1e-2};   // in units [m].

template <typename T>
MobilPlanner<T>::MobilPlanner(const RoadGeometry* road) : road_(road) {
  // Declare the input port for the ego car PoseVector.
  this->DeclareInputPort(systems::kVectorValued, PoseVector<T>::kSize);
  // Declare the input port for the traffic car PoseBundle.
  this->DeclareAbstractInputPort();
  // Declare the DrivingCommand output port.
  this->DeclareOutputPort(systems::kVectorValued,
                          DrivingCommandIndices::kNumCoordinates);
  // Declare the output port exposing the Cartesian goal point.
  this->DeclareOutputPort(systems::kVectorValued, kGoalPositionSize);
  // TODO(jadecastro): Pass an Enum {kToLeft, kToRight, kStraight} instead as an
  // abstract port.
}

template <typename T>
MobilPlanner<T>::~MobilPlanner() {}

template <typename T>
const systems::InputPortDescriptor<T>& MobilPlanner<T>::ego_pose_input() const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::InputPortDescriptor<T>&
MobilPlanner<T>::agent_pose_bundle_input() const {
  return systems::System<T>::get_input_port(1);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
MobilPlanner<T>::driving_command_output() const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
MobilPlanner<T>::goal_position_output() const {
  return systems::System<T>::get_output_port(1);
}

template <typename T>
void MobilPlanner<T>::DoCalcOutput(const systems::Context<T>& context,
                                   systems::SystemOutput<T>* output) const {
  // Obtain the parameters.
  const int kParamsIndex = 0;
  const MobilPlannerParameters<T>& params =
      this->template GetNumericParameter<MobilPlannerParameters>(context,
                                                                 kParamsIndex);

  // Obtain the input/output data structures.
  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(
          context, this->ego_pose_input().get_index());
  DRAKE_ASSERT(ego_pose != nullptr);

  const PoseBundle<T>* const agent_poses =
      this->template EvalInputValue<PoseBundle<T>>(
          context, this->agent_pose_bundle_input().get_index());
  DRAKE_ASSERT(agent_poses != nullptr);

  systems::BasicVector<T>* const goal_output_vector =
      output->GetMutableVectorData(1);
  DRAKE_ASSERT(goal_output_vector != nullptr);

  ImplDoCalcLane(*ego_pose, *agent_poses, params, goal_output_vector);

  systems::BasicVector<T>* const command_output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(command_output_vector != nullptr);
  DrivingCommand<T>* const driving_command =
      dynamic_cast<DrivingCommand<T>*>(command_output_vector);
  DRAKE_ASSERT(driving_command != nullptr);

  ImplDoCalcOutput(*ego_pose, *agent_poses, params, driving_command);
}

template <typename T>
void MobilPlanner<T>::ImplDoCalcLane(
    const PoseVector<T>& ego_pose, const PoseBundle<T>& agent_poses,
    const MobilPlannerParameters<T>& params,
    systems::BasicVector<T>* goal_output) const {
  const RoadPosition& ego_position =
      PoseSelector<T>::GetRoadPosition(*road_, ego_pose.get_isometry());
  // Prepare a list of (possibly nullptr) Lanes to evaluate.
  std::pair<const Lane*, const Lane*> lanes =
      std::make_pair(ego_position.lane->to_left(),
                     ego_position.lane->to_right());

  const Lane* new_lane;
  if (lanes.first != nullptr || lanes.second != nullptr) {
    const std::pair<T, T> incentives =
        ComputeIncentives(lanes, params, ego_pose, agent_poses);
    // Choose the lane with the highest incentive score greater than zero.
    const T threshold = params.threshold();
    if (incentives.first >= incentives.second)
      new_lane = (incentives.first > threshold) ? lanes.first :
          ego_position.lane;
    else
      new_lane = (incentives.second > threshold) ? lanes.second :
          ego_position.lane;
  }
  // TODO(jadecastro): Changes to the goal should be an event.
  const GeoPosition goal = ComputeGoalPoint(params, new_lane, ego_position);
  (*goal_output)[0] = goal.x;
  (*goal_output)[1] = goal.y;
}

template <typename T>
void MobilPlanner<T>::ImplDoCalcOutput(const PoseVector<T>& ego_pose,
                                       const PoseBundle<T>& agent_poses,
                                       const MobilPlannerParameters<T>& params,
                                       DrivingCommand<T>* command) const {
  const RoadPosition& ego_position =
      PoseSelector<T>::GetRoadPosition(*road_, ego_pose.get_isometry());
  const RoadPosition& agent_position =
      PoseSelector<T>::SelectClosestLeadingPosition(*road_, ego_pose,
                                                    agent_poses);

  // Output the acceleration command from the IDM equation and allocate the
  // result to either the throttle or brake.
  const T command_acceleration =
      EvaluateIdm(params, ego_position, agent_position);
  command->set_throttle(
      cond(command_acceleration < T(0.), T(0.), command_acceleration));
  command->set_brake(
      cond(command_acceleration >= T(0.), T(0.), -command_acceleration));
  command->set_steering_angle(0.);
}

template <typename T>
const std::pair<T, T> MobilPlanner<T>::ComputeIncentives(
    const std::pair<const Lane*, const Lane*> lanes,
    const MobilPlannerParameters<T>& params, const PoseVector<T>& ego_pose,
    const PoseBundle<T>& agent_poses) const {
  // Initially disincentivize all lane options.
  std::pair<T, T> incentive(-kLargeAcceleration, -kLargeAcceleration);

  const RoadPosition& ego_position =
      PoseSelector<T>::GetRoadPosition(*road_, ego_pose.get_isometry());
  DRAKE_DEMAND(ego_position.lane != nullptr);
  const std::pair<RoadPosition, RoadPosition>& positions_this =
      PoseSelector<T>::SelectClosestPositions(*road_, nullptr, ego_pose,
                                              agent_poses);

  const T ego_old_accel =
      EvaluateIdm(params, ego_position, positions_this.first);
  const T trailing_this_old_accel =
      EvaluateIdm(params, positions_this.second, ego_position);
  const T trailing_this_new_accel =
      EvaluateIdm(params, positions_this.second, positions_this.first);
  // delta_trailer_this_lane?
  const T trailing_accel_this =
      trailing_this_new_accel - trailing_this_old_accel;
  for (int i = 0; i < 2; i++) {
    if ((i == 0) ? lanes.first != nullptr : lanes.second != nullptr) {
      const std::pair<RoadPosition, RoadPosition>& positions =
          PoseSelector<T>::SelectClosestPositions(
              *road_, (i == 0) ? lanes.first : lanes.second, ego_pose,
              agent_poses);
      const T ego_new_accel =
          EvaluateIdm(params, ego_position, positions.first);
      const T trailing_old_accel =
          EvaluateIdm(params, positions.second, ego_position);
      const T trailing_new_accel =
          EvaluateIdm(params, positions.second, positions.first);
      // delta_trailer_other_lane?
      const T trailing_accel_other = trailing_new_accel - trailing_old_accel;
      // delta_ego?
      const T ego_accel = ego_new_accel - ego_old_accel;

      // We wouldn't want to discomfort our trailing car-to-be!
      if (trailing_old_accel < -params.max_deceleration()) continue;

      // Compute the incentive as a weighted sum of the net accelerations for
      // the ego and each immediate neighbor.
      const T result =
          ego_accel + params.p() * (trailing_accel_other + trailing_accel_this);
      (i == 0) ? (incentive.first = result) : (incentive.second = result);
    }
  }
  return incentive;
}

template <typename T>
const T MobilPlanner<T>::EvaluateIdm(
    const MobilPlannerParameters<T>& params, const RoadPosition& ego_position,
    const RoadPosition& agent_position) const {
  const T car_length = 4.5;

  const T& s_ego = ego_position.pos.s;
  const T& s_dot_ego = 10.;  // TODO(jadecastro): Retrieve an actual velocity.
  const T& s_agent = agent_position.pos.s;
  const T& s_dot_agent = 0.;  // TODO(jadecastro): Retrieve an actual velocity.

  // Ensure that we are supplying the planner with sane parameters and input
  // values.
  const T delta = s_agent - s_ego;
  //
  const T net_distance = cond(delta > T(0.),
                              cond(delta - car_length < T(kEpsilonDistance),
                                   T(kEpsilonDistance), delta - car_length),
                              cond(delta + car_length > T(kEpsilonDistance),
                                   T(kEpsilonDistance), delta + car_length));
  DRAKE_DEMAND(std::abs(net_distance) >= kEpsilonDistance);
  const T closing_velocity = s_dot_ego - s_dot_agent;

  return IdmPlanner<T>::Evaluate(params, s_dot_ego, net_distance,
                                 closing_velocity);
}

template <typename T>
const GeoPosition MobilPlanner<T>::ComputeGoalPoint(
    const MobilPlannerParameters<T>& params, const Lane* lane,
    const RoadPosition& position) const {
  const LanePosition lane_position(position.pos.s + params.s_lookahead(), 0, 0);
  return lane->ToGeoPosition(lane_position);
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>> MobilPlanner<T>::AllocateOutputVector(
    const systems::OutputPortDescriptor<T>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() <= 1);
  return std::make_unique<DrivingCommand<T>>();
}

template <typename T>
std::unique_ptr<systems::Parameters<T>> MobilPlanner<T>::AllocateParameters()
    const {
  auto params = std::make_unique<MobilPlannerParameters<T>>();
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

template <typename T>
void MobilPlanner<T>::SetDefaultParameters(
    const systems::LeafContext<T>& context,
    systems::Parameters<T>* params) const {
  auto mobil_params = dynamic_cast<MobilPlannerParameters<T>*>(
      params->get_mutable_numeric_parameter(0));
  IdmPlanner<T>::SetDefaultParameters(mobil_params);
}

// These instantiations must match the API documentation in
// idm_planner.h.
template class MobilPlanner<double>;

}  // namespace automotive
}  // namespace drake
