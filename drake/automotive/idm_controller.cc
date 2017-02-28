#include "drake/automotive/idm_controller.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"

namespace drake {

using systems::rendering::PoseVector;

namespace automotive {

template <typename T>
IdmController<T>::IdmController(const T& v_ref) : v_ref_(v_ref) {
  // TODO(jadecastro): Remove v_ref from the constructor.
  // The reference velocity must be strictly positive.
  DRAKE_ASSERT(v_ref > 0);

  const int kEgoCarOutputVectorSize = 2;
  const int kAgentCarOutputVectorSize = 2;
  const int kLinearAccelerationSize = 1;
  // Declare the ego car input port.
  this->DeclareInputPort(systems::kVectorValued, kEgoCarOutputVectorSize);
  // Declare the agent car input port.
  this->DeclareInputPort(systems::kVectorValued, kAgentCarOutputVectorSize);
  // Declare the output port.
  this->DeclareOutputPort(systems::kVectorValued, kLinearAccelerationSize);
}

template <typename T>
IdmController<T>::~IdmController() {}

template <typename T>
const systems::InputPortDescriptor<T>& IdmController<T>::get_ego_input() const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::InputPortDescriptor<T>& IdmController<T>::get_agent_input()
    const {
  return systems::System<T>::get_input_port(1);
}

template <typename T>
void IdmController<T>::DoCalcOutput(const systems::Context<T>& context,
                                    systems::SystemOutput<T>* output) const {
  // Obtain the input/output structures we need to read from and write into.
  const systems::VectorBase<T>* ego_input_vector =
      this->EvalVectorInput(context, this->get_ego_input().get_index());
  DRAKE_ASSERT(ego_input_vector != nullptr);
  const PoseVector<T>* const ego_pose =
      dynamic_cast<const PoseVector<T>*>(ego_input_vector);
  DRAKE_ASSERT(ego_pose != nullptr);

  // TODO(jadecastro): Make this PoseBundle and select a certain PoseVector.
  const systems::VectorBase<T>* agents_input_vector =
      this->EvalVectorInput(context, this->get_agent_input().get_index());
  DRAKE_ASSERT(agents_input_vector != nullptr);
  const PoseVector<T>* const agents_pose =
      dynamic_cast<const PoseVector<T>*>(agents_input_vector);
  DRAKE_ASSERT(agents_pose != nullptr);

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

  ImplDoCalcOutput(*ego_pose, *agents_pose, params, driving_command);
}

template <typename T>
void IdmController<T>::ImplDoCalcOutput(const PoseVector<T>& ego_pose,
                                        const PoseVector<T>& agents_pose,
                                        const IdmPlannerParameters<T>& params,
                                        DrivingCommand<T>* command) const {
  const auto ego_position = ego_pose.get_translation();
  const T& x_ego = ego_position.translation().x();
  const T& v_ego = 10.;  // TODO(jadecastro): Require velocity from SimpleCar.
  // const auto agent_position = agents_pose.get_translation();
  const T& x_agent = 100.; // agent_position.translation().x();
  const T& v_agent = 0.;  // TODO(jadecastro): Require velocity from
                          // TrajectoryCar.

  // Ensure that we are supplying the planner with sane parameters and
  // input values.
  const T net_distance = x_agent - x_ego - params.l_a();
  DRAKE_DEMAND(net_distance > 0.);
  const T closing_velocity = v_ego - v_agent;

  command->set_throttle(
      IdmModel(net_distance, closing_velocity, v_ego, params));
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
    const systems::LeafContext<T>& context,
    systems::Parameters<T>* params) const {
  // Default values from https://en.wikipedia.org/wiki/Intelligent_driver_model.
  auto idm_params = dynamic_cast<IdmPlannerParameters<T>*>(
      params->get_mutable_numeric_parameter(0));
  DRAKE_DEMAND(idm_params != nullptr);
  idm_params->set_v_ref(v_ref_);         // desired velocity in free traffic.
  idm_params->set_a(T(1.0));             // max acceleration.
  idm_params->set_b(T(3.0));             // comfortable braking deceleration.
  idm_params->set_s_0(T(1.0));           // minimum desired net distance.
  idm_params->set_time_headway(T(0.1));  // desired headway to lead vehicle.
  idm_params->set_delta(T(4.0));  // recommended choice of free-road exponent.
  idm_params->set_l_a(T(4.5));    // length of leading car.
}

// These instantiations must match the API documentation in
// idm_planner.h.
template class IdmController<double>;
template class IdmController<drake::TaylorVarXd>;
template class IdmController<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
