#include "drake/automotive/idm_controller.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/api/lane_data.h"  // For explicit-constructed RoadGeometry
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/extract_double.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using maliput::api::Rotation;
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

template <typename T>
void IdmController<T>::ImplDoCalcOutput(
    const PoseVector<T>& ego_pose,
    const FrameVelocity<T>& ego_velocity,
    const PoseBundle<T>& traffic_poses,
    const IdmPlannerParameters<T>& idm_params,
    systems::BasicVector<T>* command) const {
  using std::max;

  DRAKE_DEMAND(idm_params.IsValid());
  const double scan_distance{100.};  // TODO(jadecastro): Make this a parameter.
  T headway_distance{0.};

  auto translation = ego_pose.get_isometry().translation();
  std::cout << " x " << translation.x() << std::endl;
  std::cout << " y " << translation.y() << std::endl;
  std::cout << " z " << translation.z() << std::endl;
  const maliput::api::GeoPosition geo_position(
      ExtractDoubleOrThrow(translation.x()),
      ExtractDoubleOrThrow(translation.y()),
      ExtractDoubleOrThrow(translation.z()));
  const RoadPosition ego_position =
      road_.ToRoadPosition(geo_position, nullptr, nullptr, nullptr);

  // Find the single closest car ahead.
  const RoadOdometry<T> lead_car_odom =
      PoseSelector<T>::FindSingleClosestPose(
          ego_position.lane, ego_pose, ego_velocity, traffic_poses,
          scan_distance, WhichSide::kAhead, &headway_distance);

  T s_dot_ego =
      PoseSelector<T>::GetSigmaVelocity({ego_position, ego_velocity});
  T s_dot_lead = PoseSelector<T>::GetSigmaVelocity(
      {{lead_car_odom.lane, lead_car_odom.pos}, lead_car_odom.vel});

  // Manual population of the partial derivatives for the vehicle
  // poses/velocities.
  ComputePartials(ego_pose, ego_velocity, lead_car_odom, &s_dot_ego,
                  &s_dot_lead);

  // Saturate the net_distance at distance_lower_bound away from the ego car to
  // avoid near-singular solutions inherent to the IDM equation.
  const T actual_headway = headway_distance - idm_params.bloat_diameter();
  const T net_distance = max(actual_headway, idm_params.distance_lower_limit());
  const T closing_velocity = s_dot_ego - s_dot_lead;

  // Compute the acceleration command from the IDM equation.
  (*command)[0] = IdmPlanner<T>::Evaluate(idm_params, s_dot_ego,
                                          net_distance, closing_velocity);
}

template <typename T>
template <typename T1>
void IdmController<T>::ComputePartials(
    const PoseVector<std::enable_if_t<std::is_same<T1, double>::value, T1>>&
    ego_pose,
    const FrameVelocity<std::enable_if_t<std::is_same<T1, double>::value, T1>>&
    ego_velocity,
    const RoadOdometry<std::enable_if_t<std::is_same<T1, double>::value, T1>>&
    lead_car_odom,
    double* s_dot_ego, double* s_dot_lead) const {}

template <typename T>
template <typename T1>
void IdmController<T>::ComputePartials(
    const PoseVector<std::enable_if_t<std::is_same<T1, AutoDiffXd>::value, T1>>&
    ego_pose,
    const FrameVelocity<std::enable_if_t<
    std::is_same<T1, AutoDiffXd>::value, T1>>& ego_velocity,
    const RoadOdometry<std::enable_if_t<
    std::is_same<T1, AutoDiffXd>::value, T1>>& lead_car_odom,
    AutoDiffXd* s_dot_ego, AutoDiffXd* s_dot_lead) const {
  auto translation = ego_pose.get_isometry().translation();
  std::cout << " x " << translation.x() << std::endl;
  std::cout << " y " << translation.y() << std::endl;
  std::cout << " z " << translation.z() << std::endl;
  std::cout << " x derivs " << translation.x().derivatives() << std::endl;
  std::cout << " y derivs " << translation.y().derivatives() << std::endl;
  std::cout << " z derivs " << translation.z().derivatives() << std::endl;

  auto velocity = ego_velocity.get_velocity().translational();
  std::cout << " x velocity " << velocity.x() << std::endl;
  std::cout << " y velocity " << velocity.y() << std::endl;
  std::cout << " z velocity " << velocity.z() << std::endl;
  std::cout << " x velocity derivs " << velocity.x().derivatives() << std::endl;
  std::cout << " y velocity derivs " << velocity.y().derivatives() << std::endl;
  std::cout << " z velocity derivs " << velocity.z().derivatives() << std::endl;

  auto vel = lead_car_odom.vel.get_velocity().translational();
  std::cout << " x vel " << vel.x() << std::endl;
  std::cout << " y vel " << vel.y() << std::endl;
  std::cout << " z vel " << vel.z() << std::endl;
  std::cout << " x vel derivs " << vel.x().derivatives() << std::endl;
  std::cout << " y vel derivs " << vel.y().derivatives() << std::endl;
  std::cout << " z vel derivs " << vel.z().derivatives() << std::endl;

  const RoadPosition ego_position =
      road_.ToRoadPosition({
          ExtractDoubleOrThrow(translation.x()),
              ExtractDoubleOrThrow(translation.y()),
              ExtractDoubleOrThrow(translation.z())},
        nullptr, nullptr, nullptr);
  const Vector3<AutoDiffXd> ego_pos_vector(ego_position.pos.s(),
                                           ego_position.pos.r(),
                                           ego_position.pos.h());
  /* const Matrix3<AutoDiffXd> ego_position_prime{((Eigen::Matrix3d() <<
      ego_pos_vector(0).derivatives(),
      ego_pos_vector(0).derivatives(),
      ego_pos_vector(0).derivatives()).finished())};
      std::cerr << ego_position_prime(0) << std::endl;*/
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
