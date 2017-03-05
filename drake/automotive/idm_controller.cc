#include "drake/automotive/idm_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Geometry>

#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"

namespace drake {

using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

namespace automotive {

template <typename T>
IdmController<T>::IdmController() {
  // Declare the input for the ego car.
  this->DeclareInputPort(systems::kVectorValued, PoseVector<T>::kSize);
  // Declare the inputs for the traffic cars.
  this->DeclareAbstractInputPort();
  // Declare the output port.
  this->DeclareOutputPort(systems::kVectorValued,
                          DrivingCommandIndices::kNumCoordinates);
}

template <typename T>
IdmController<T>::~IdmController() {}

template <typename T>
const systems::InputPortDescriptor<T>& IdmController<T>::ego_pose_input()
    const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::InputPortDescriptor<T>&
IdmController<T>::agent_pose_bundle_input() const {
  return systems::System<T>::get_input_port(1);
}

template <typename T>
void IdmController<T>::DoCalcOutput(const systems::Context<T>& context,
                                    systems::SystemOutput<T>* output) const {
  // Obtain the input/output data structures.
  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(
          context, this->ego_pose_input().get_index());
  DRAKE_ASSERT(ego_pose != nullptr);

  // TODO(jadecastro): Make this PoseBundle and select a certain PoseVector.
  const PoseBundle<T>* const agent_poses =
      this->template EvalInputValue<PoseBundle<T>>(
          context, this->agent_pose_bundle_input().get_index());
  DRAKE_ASSERT(agent_poses != nullptr);

  systems::BasicVector<T>* const command_output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(command_output_vector != nullptr);
  DrivingCommand<T>* const driving_command =
      dynamic_cast<DrivingCommand<T>*>(command_output_vector);
  DRAKE_ASSERT(driving_command != nullptr);

  // Obtain the parameters.
  const int kParamsIndex = 0;
  const IdmPlannerParameters<T>& params =
      this->template GetNumericParameter<IdmPlannerParameters>(context,
                                                               kParamsIndex);

  const Isometry3<T>& agent_pose =
      SelectNearestTargetAhead(*ego_pose, *agent_poses);
  ImplDoCalcOutput(*ego_pose, agent_pose, params, driving_command);
}

template <typename T>
const Isometry3<T>
IdmController<T>::SelectNearestTargetAhead(
    const PoseVector<T>& ego_pose, const PoseBundle<T>& agent_poses) const {
  const T car_length = 4.5;  // TODO(jadecastro): Car length needs to go
                             // somewhere else.

  // TODO(jadcastro): Check if in lane or not.
  Isometry3<T> pose_result = Isometry3<T>::Identity();
  pose_result.translate(Vector3<T>(
      std::numeric_limits<double>::infinity(), 0, 0));
  const T& x_ego = ego_pose.get_translation().translation().x();
  for (int i = 0; i < agent_poses.get_num_poses(); ++i) {
    const T& x_agent = agent_poses.get_pose(i).translation().x();
    if (x_agent > x_ego + car_length && x_agent < pose_result.translation().x())
      pose_result = agent_poses.get_pose(i);
  }
  return pose_result;
}

template <typename T>
void IdmController<T>::ImplDoCalcOutput(const PoseVector<T>& ego_pose,
                                        const Isometry3<T>& agent_pose,
                                        const IdmPlannerParameters<T>& params,
                                        DrivingCommand<T>* command) const {
  const T car_length = 4.5;

  // TODO(jadecastro): Convert this to Lane coordinates.
  const T& x_ego = ego_pose.get_translation().translation().x();
  const T& v_ego = 10.;  // TODO(jadecastro): Require velocity from SimpleCar.
  // const auto agent_position = agents_pose.get_translation();
  const T& x_agent = agent_pose.translation().x();
  const T& v_agent = 0.;  // TODO(jadecastro): Require velocity from
                          // TrajectoryCar.

  // Ensure that we are supplying the planner with sane parameters and input
  // values.
  const T net_distance = x_agent - x_ego - car_length;  // <---- TODO
  DRAKE_DEMAND(net_distance > 0.);
  const T closing_velocity = v_ego - v_agent;

  // Output the acceleration command.
  const T command_acceleration =
      IdmPlanner<T>::Evaluate(params, v_ego, net_distance, closing_velocity);
  command->set_throttle(
      cond(command_acceleration < T(0.), T(0.), command_acceleration));
  command->set_brake(
      cond(command_acceleration >= T(0.), T(0.), -command_acceleration));
  command->set_steering_angle(0.);
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>> IdmController<T>::AllocateOutputVector(
    const systems::OutputPortDescriptor<T>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() <= 1);
  return std::make_unique<DrivingCommand<T>>();
}

template <typename T>
std::unique_ptr<systems::Parameters<T>> IdmController<T>::AllocateParameters()
    const {
  auto params = std::make_unique<IdmPlannerParameters<T>>();
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

template <typename T>
void IdmController<T>::SetDefaultParameters(
    const systems::LeafContext<T>& context, systems::Parameters<T>* params)
    const {
  auto idm_params = dynamic_cast<IdmPlannerParameters<T>*>(
      params->get_mutable_numeric_parameter(0));
  IdmPlanner<T>::SetDefaultParameters(idm_params);
}

// These instantiations must match the API documentation in
// idm_planner.h.
template class IdmController<double>;
template class IdmController<drake::TaylorVarXd>;
template class IdmController<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
