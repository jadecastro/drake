#include "drake/automotive/idm_controller.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_formula.h"
#include "drake/math/saturate.h"

namespace drake {
namespace automotive {

using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using maliput::api::Rotation;
using math::saturate;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

static constexpr int kIdmParamsIndex{0};

template <typename T>
IdmController<T>::IdmController(const RoadGeometry& road)
    : road_(road),
      ego_pose_index_{
          this->DeclareVectorInputPort(PoseVector<T>()).get_index()},
      ego_velocity_index_{
          this->DeclareVectorInputPort(FrameVelocity<T>()).get_index()},
      traffic_index_{this->DeclareAbstractInputPort().get_index()},
      acceleration_index_{
          this->DeclareVectorOutputPort(systems::BasicVector<T>(1))
              .get_index()} {
  this->DeclareNumericParameter(IdmPlannerParameters<T>());
}

template <typename T>
IdmController<T>::~IdmController() {}

template <typename T>
const systems::InputPortDescriptor<T>& IdmController<T>::ego_pose_input()
    const {
  return systems::System<T>::get_input_port(ego_pose_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& IdmController<T>::ego_velocity_input()
    const {
  return systems::System<T>::get_input_port(ego_velocity_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& IdmController<T>::traffic_input() const {
  return systems::System<T>::get_input_port(traffic_index_);
}

template <typename T>
const systems::OutputPortDescriptor<T>& IdmController<T>::acceleration_output()
    const {
  return systems::System<T>::get_output_port(acceleration_index_);
}

template <typename T>
void IdmController<T>::DoCalcOutput(const systems::Context<T>& context,
                                    systems::SystemOutput<T>* output) const {
  // Obtain the parameters.
  const IdmPlannerParameters<T>& idm_params =
      this->template GetNumericParameter<IdmPlannerParameters>(context,
                                                               kIdmParamsIndex);

  // Obtain the input/output data structures.
  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(context, ego_pose_index_);
  DRAKE_ASSERT(ego_pose != nullptr);

  const FrameVelocity<T>* const ego_velocity =
      this->template EvalVectorInput<FrameVelocity>(context,
                                                    ego_velocity_index_);
  DRAKE_ASSERT(ego_velocity != nullptr);

  const PoseBundle<T>* const traffic_poses =
      this->template EvalInputValue<PoseBundle<T>>(context, traffic_index_);
  DRAKE_ASSERT(traffic_poses != nullptr);

  systems::BasicVector<T>* const accel_output =
      output->GetMutableVectorData(acceleration_index_);
  DRAKE_ASSERT(accel_output != nullptr);

  ImplDoCalcOutput(*ego_pose, *ego_velocity, *traffic_poses, idm_params,
                   accel_output);
}

// Specialization for double scalar type.
template <typename T>
void IdmController<T>::ImplDoCalcOutput(
    const PoseVector<double>& ego_pose,
    const FrameVelocity<double>& ego_velocity,
    const PoseBundle<double>& traffic_poses,
    const IdmPlannerParameters<double>& idm_params,
    systems::BasicVector<double>* command) const {
  DRAKE_DEMAND(idm_params.IsValid());
  const double scan_distance{100.};  // TODO(jadecastro): Make this a parameter.
  double headway_distance{};

  const RoadPosition ego_position =
      road_.ToRoadPosition({ego_pose.get_isometry().translation().x(),
                             ego_pose.get_isometry().translation().y(),
                             ego_pose.get_isometry().translation().z()},
                            nullptr, nullptr, nullptr);

  // Find the single closest car ahead.
  const RoadOdometry<double>& lead_car_odom =
      PoseSelector<double>::FindSingleClosestPose(
          ego_position.lane, ego_pose, ego_velocity, traffic_poses,
          scan_distance, WhichSide::kAhead, &headway_distance);

  const double& s_dot_ego =
      PoseSelector<double>::GetIsoLaneVelocity(
          ego_position, ego_velocity).sigma_v;
  const double& s_dot_lead = PoseSelector<double>::GetIsoLaneVelocity(
      {lead_car_odom.lane, lead_car_odom.pos}, lead_car_odom.vel).sigma_v;

  // Saturate the net_distance at distance_lower_bound away from the ego car to
  // avoid near-singular solutions inherent to the IDM equation.
  const double net_distance = saturate(
      headway_distance - idm_params.bloat_diameter(),
      idm_params.distance_lower_limit(),
      std::numeric_limits<double>::infinity());
  const double closing_velocity = s_dot_ego - s_dot_lead;

  // Compute the acceleration command from the IDM equation.
  (*command)[0] = IdmPlanner<double>::Evaluate(idm_params, s_dot_ego,
                                               net_distance, closing_velocity);
}

// Specialization for AutoDiffXd scalar type.
template <typename T>
void IdmController<T>::ImplDoCalcOutput(
    const PoseVector<AutoDiffXd>& ego_pose,
    const FrameVelocity<AutoDiffXd>& ego_velocity,
    const PoseBundle<AutoDiffXd>& traffic_poses,
    const IdmPlannerParameters<AutoDiffXd>& idm_params,
    systems::BasicVector<AutoDiffXd>* command) const {
  DRAKE_DEMAND(idm_params.IsValid());
  //const double scan_distance{100.};
  //AutoDiffXd headway_distance{};

  const RoadPosition ego_position =
      road_.ToRoadPosition({
          ExtractDoubleOrThrow(ego_pose.get_isometry().translation().x()),
              ExtractDoubleOrThrow(ego_pose.get_isometry().translation().y()),
              ExtractDoubleOrThrow(ego_pose.get_isometry().translation().z())},
        nullptr, nullptr, nullptr);
  const Vector3<AutoDiffXd> ego_pos_vector(ego_position.pos.s(),
                                           ego_position.pos.r(),
                                           ego_position.pos.h());
  const Matrix3<AutoDiffXd> ego_position_prime{((Eigen::Matrix3d() <<
      ego_pos_vector(0).derivatives(),
      ego_pos_vector(0).derivatives(),
      ego_pos_vector(0).derivatives()).finished())};
  std::cerr << ego_position_prime(0) << std::endl;

  // Find the single closest car ahead.
  const RoadOdometry<AutoDiffXd>& lead_car_odom =
      PoseSelector<AutoDiffXd>::FindSingleClosestPose(
          ego_position.lane, ego_pose_double, ego_velocity_double,
          traffic_poses_double, scan_distance, WhichSide::kAhead,
          &headway_distance);

  const AutoDiffXd& s_dot_ego = PoseSelector<AutoDiffXd>::GetSigmVelocity(
          {ego_position.lane, ego_position.pos, ego_velocity});
  const AutoDiffXd& s_dot_lead = PoseSelector<AutoDiffXd>::GetSigmaVelocity(
      {lead_car_odom.lane, lead_car_odom.pos, lead_car_odom.vel});

  // Saturate the net_distance at distance_lower_bound away from the ego car to
  // avoid near-singular solutions inherent to the IDM equation.
  const AutoDiffXd net_distance = saturate(
      headway_distance - idm_params.bloat_diameter(),
      idm_params.distance_lower_limit(),
      std::numeric_limits<AutoDiffXd>::infinity());
  const AutoDiffXd closing_velocity = s_dot_ego - s_dot_lead;

  // Compute the acceleration command from the IDM equation.
  (*command)[0] = IdmPlanner<AutoDiffXd>::Evaluate(idm_params, s_dot_ego,
                                                   net_distance,
                                                   closing_velocity);
}

template <typename T>
IdmController<AutoDiffXd>* IdmController<T>::DoToAutoDiffXd() const {
  return new IdmController<AutoDiffXd>(road_);
}

// These instantiations must match the API documentation in idm_controller.h.
template class IdmController<double>;
template class IdmController<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
