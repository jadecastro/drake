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
    const PoseVector<T>& ego_pose, const FrameVelocity<T>& ego_velocity,
    const PoseBundle<T>& traffic_poses,
    const IdmPlannerParameters<T>& idm_params,
    systems::BasicVector<T>* command) const {
  using std::max;

  DRAKE_DEMAND(idm_params.IsValid());
  const double scan_distance{100.};  // TODO(jadecastro): Make this a parameter.
  // Initialize to a "model value" that has entries for the derivatives.
  T headway_distance_value{0.};

  auto translation = ego_pose.get_isometry().translation();
  const maliput::api::GeoPosition geo_position(
      ExtractDoubleOrThrow(translation.x()),
      ExtractDoubleOrThrow(translation.y()),
      ExtractDoubleOrThrow(translation.z()));
  const RoadPosition ego_position =
      road_.ToRoadPosition(geo_position, nullptr, nullptr, nullptr);

  // Find the single closest car ahead.
  const RoadOdometry<T> lead_car_odom = PoseSelector<T>::FindSingleClosestPose(
      ego_position.lane, ego_pose, ego_velocity, traffic_poses, scan_distance,
      WhichSide::kAhead, &headway_distance_value);

  T headway_distance =
      headway_distance_value * ego_pose.get_isometry().translation().x();
  std::cout << " s ego " << ego_position.pos.s() << std::endl;
  std::cout << " s lead " << lead_car_odom.pos.s() << std::endl;
  std::cout << " headway_distance " << headway_distance << std::endl;

  T s_dot_ego = PoseSelector<T>::GetSigmaVelocity({ego_position, ego_velocity});
  T s_dot_lead = PoseSelector<T>::GetSigmaVelocity(
      {{lead_car_odom.lane, lead_car_odom.pos}, lead_car_odom.vel});

  // Manual population of the partial derivatives for the vehicle
  // poses/velocities.
  ComputePartials(ego_pose, ego_velocity, lead_car_odom, traffic_poses,
                  idm_params,
                  &s_dot_ego, &s_dot_lead, &headway_distance);

  // Saturate the net_distance at distance_lower_bound away from the ego car to
  // avoid near-singular solutions inherent to the IDM equation.
  const T actual_headway = headway_distance - idm_params.bloat_diameter();
  const T net_distance = max(idm_params.distance_lower_limit(), actual_headway);
  const T closing_velocity = s_dot_ego - s_dot_lead;

  std::cout << " net_distance " << net_distance << std::endl;
  std::cout << " closing_velocity " << closing_velocity << std::endl;
  std::cout << " s_dot_ego " << s_dot_ego << std::endl;

  // Compute the acceleration command from the IDM equation.
  (*command)[0] = IdmPlanner<T>::Evaluate(idm_params, s_dot_ego, net_distance,
                                          closing_velocity);
  std::cerr << " idm acceleration (command) " << (*command)[0] << std::endl;

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
    const PoseBundle<std::enable_if_t<std::is_same<T1, double>::value, T1>>&
        traffic_poses,
    const IdmPlannerParameters<std::enable_if_t<
        std::is_same<T1, double>::value, T1>>& params,
    double* s_dot_ego, double* s_dot_lead, double* headway_distance) const {}

template <typename T>
template <typename T1>
void IdmController<T>::ComputePartials(
    const PoseVector<std::enable_if_t<std::is_same<T1, AutoDiffXd>::value, T1>>&
        ego_pose,
    const FrameVelocity<std::enable_if_t<std::is_same<T1, AutoDiffXd>::value,
                                         T1>>& ego_velocity,
    const RoadOdometry<std::enable_if_t<std::is_same<T1, AutoDiffXd>::value,
                                        T1>>& lead_car_odom,
    const PoseBundle<std::enable_if_t<std::is_same<T1, AutoDiffXd>::value, T1>>&
        traffic_poses,
    const IdmPlannerParameters<std::enable_if_t<
        std::is_same<T1, AutoDiffXd>::value, T1>>& params,
    AutoDiffXd* s_dot_ego, AutoDiffXd* s_dot_lead, AutoDiffXd* headway_distance)
const {
  // The ego car's position has intact partial derivatives.  NOTE: For some
  // reason, these derivatives only exist in the AutoDiffXd data structure when
  // compiling with SNOPT (they do not exist when compiled with IPOPT - why??).
  auto ego_trans = ego_pose.get_isometry().translation();
  /*
  std::cout << " x ego_trans " << ego_trans.x() << std::endl;
  std::cout << " y ego_trans " << ego_trans.y() << std::endl;
  std::cout << " z ego_trans " << ego_trans.z() << std::endl;
  std::cout << " x derivs ego_trans " << ego_trans.x().derivatives() << std::endl;
  std::cout << " y derivs ego_trans " << ego_trans.y().derivatives() << std::endl;
  std::cout << " z derivs ego_trans " << ego_trans.z().derivatives() << std::endl;

  // The ego car's velocity has intact partial derivatives (regardless of the
  // chosen optimizer).
  auto ego_vel = ego_velocity.get_velocity().translational();
  std::cout << " x ego_vel " << ego_vel.x() << std::endl;
  std::cout << " y ego_vel " << ego_vel.y() << std::endl;
  std::cout << " z ego_vel " << ego_vel.z() << std::endl;
  std::cout << " x ego_vel derivs " << ego_vel.x().derivatives() << std::endl;
  std::cout << " y ego_vel derivs " << ego_vel.y().derivatives() << std::endl;
  std::cout << " z ego_vel derivs " << ego_vel.z().derivatives() << std::endl;

  // Note that the partial derivatives of the lead car's velocity are empty.
  auto lead_vel = lead_car_odom.vel.get_velocity().translational();
  std::cout << " x lead_vel " << lead_vel.x() << std::endl;
  std::cout << " y lead_vel " << lead_vel.y() << std::endl;
  std::cout << " z lead_vel " << lead_vel.z() << std::endl;
  std::cout << " x lead_vel derivs " << lead_vel.x().derivatives() << std::endl;
  std::cout << " y lead_vel derivs " << lead_vel.y().derivatives() << std::endl;
  std::cout << " z lead_vel derivs " << lead_vel.z().derivatives() << std::endl;
  // std::cout << " x lead_vel dderivs " << lead_vel.x().derivatives()(0).value()
  //          << std::endl;

  // Check what the derivatives of the parameters look like.
  std::cout << " params.v_ref() " << params.v_ref() << std::endl;
  std::cout << " params.v_ref() derivs " << params.v_ref().derivatives()
            << std::endl;

  std::cout << " headway_distance " << *headway_distance << std::endl;
  std::cout << " headway_distance derivs " << headway_distance->derivatives()
            << std::endl;

  // Arbitrarily take the 0th traffic car, looking for autodiff junk. If it
  // contains these, then the problem likely exists in PoseSelector.
  std::cout << " num traffic poses: " << traffic_poses.get_num_poses()
            << std::endl;
  auto traffic_vel =
      traffic_poses.get_velocity(0).get_velocity().translational();
  std::cout << " x traffic_vel " << traffic_vel.x() << std::endl;
  std::cout << " y traffic_vel " << traffic_vel.y() << std::endl;
  std::cout << " z traffic_vel " << traffic_vel.z() << std::endl;
  std::cout << " x traffic_vel derivs " << traffic_vel.x().derivatives() << std::endl;
  std::cout << " y traffic_vel derivs " << traffic_vel.y().derivatives() << std::endl;
  std::cout << " z traffic_vel derivs " << traffic_vel.z().derivatives() << std::endl;

  // Attempt to assign one of the derivatives to some number...
  if (s_dot_ego->derivatives().size() > 0) {
    //(*s_dot_ego).derivatives()(0) = 5.7;
    std::cout << " s_dot_ego derivs " << s_dot_ego->derivatives() << std::endl;
    std::cout << " s_dot_lead derivs " << s_dot_lead->derivatives()
              << std::endl;
    const T something = (*s_dot_ego) - (*s_dot_lead);
    std::cout << "  s_dot_ego - s_dot_lead " << something << std::endl;
    std::cout << "  s_dot_ego - s_dot_lead derivs " << something.derivatives()
              << std::endl;
  }
  */

  const RoadPosition ego_position =
      road_.ToRoadPosition({ExtractDoubleOrThrow(ego_trans.x()),
                            ExtractDoubleOrThrow(ego_trans.y()),
                            ExtractDoubleOrThrow(ego_trans.z())},
                           nullptr, nullptr, nullptr);
  const Vector3<AutoDiffXd> ego_pos_vector(
      ego_position.pos.s(), ego_position.pos.r(), ego_position.pos.h());
  /* const Matrix3<AutoDiffXd> ego_position_prime{((Eigen::Matrix3d() <<
      ego_pos_vector(0).derivatives(),
      ego_pos_vector(0).derivatives(),
      ego_pos_vector(0).derivatives()).finished())};
      std::cerr << ego_position_prime(0) << std::endl;*/

  // Spike test: Assign derivatives manually to fill in the missing derivatives.
  // NOTE: These derivatives are specific to the AutomotiveSimulator diagram
  // when consumed by the Dircol trajectory optimization solver, where index 0
  // is time, and the remaining indices are the derivatives with respect to the
  // diagram's states at time index k, followed by those at time index k+1.  For
  // this particular AutomotiveSimulator diagram, we have four states, ordered
  // as follows:
  //  - lead car s-position
  //  - lead car s-velocity (sigma_v)
  //  - ego s-position
  //  - ego s-velocity (sigma_v)
  (*headway_distance).derivatives()(0) = 0.;
  (*headway_distance).derivatives()(1) = 1.;
  (*headway_distance).derivatives()(2) = 0.;
  (*headway_distance).derivatives()(3) = -1.;
  (*headway_distance).derivatives()(4) = 0.;
  (*headway_distance).derivatives()(5) = 1.;
  (*headway_distance).derivatives()(6) = 0.;
  (*headway_distance).derivatives()(7) = -1.;
  (*headway_distance).derivatives()(8) = 0.;
  std::cout << " NEW headway_distance derivs "
            << headway_distance->derivatives() << std::endl;
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
