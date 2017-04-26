#include "drake/automotive/idm_controller.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
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
  // std::cout << "IdmController: [" << this->get_name() << "]: Method called!" << std::endl;
  // Obtain the parameters.
  const IdmPlannerParameters<T>& idm_params =
      this->template GetNumericParameter<IdmPlannerParameters>(context,
                                                               kIdmParamsIndex);

  // Obtain the input/output data structures.
  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(context, ego_pose_index_);
  DRAKE_ASSERT(ego_pose != nullptr);

  // std::cout << "ego_pose =\n" << ego_pose->get_isometry().matrix() << std::endl;
  using std::isnan;
  DRAKE_THROW_UNLESS(!isnan(ego_pose->get_translation().x()));

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
  // std::cout << "IdmController [" << this->get_name() << "]: Done!" << std::endl;
}

template <typename T>
void IdmController<T>::ImplDoCalcOutput(
    const PoseVector<T>& ego_pose, const FrameVelocity<T>& ego_velocity,
    const PoseBundle<T>& traffic_poses,
    const IdmPlannerParameters<T>& idm_params,
    systems::BasicVector<T>* command) const {
  DRAKE_DEMAND(idm_params.IsValid());
  const double scan_distance{100.};
  double headway_distance{};

  const RoadPosition ego_position =
      road_.ToRoadPosition({ego_pose.get_isometry().translation().x(),
                             ego_pose.get_isometry().translation().y(),
                             ego_pose.get_isometry().translation().z()},
                            nullptr, nullptr, nullptr);
  // std::cout << "ego position: " << ego_position.lane->id().id << ", s = " << ego_position.pos.s << ", r = " << ego_position.pos.r << ", h = " << ego_position.pos.h << std::endl;
  /*
  std::cout << "IdmController::ImplDoCalcOutput [" << this->get_name() << "]: "
      << "Obtaining lead car odometry:\n"
      << "  - ego_pose = " << ego_pose << "\n"
      << "  - ego_velocity = " << ego_velocity << "\n"
      << "  - traffic_poses =\n" << traffic_poses.ToString("    ") << "\n"
      << "  - scan_distance = " << scan_distance << "\n"
      << "  - headway_distance = " << headway_distance << std::endl;
  */
  // Find the single closest car ahead.
  const RoadOdometry<T>& lead_car_odom =
      PoseSelector::FindSingleClosestAheadAndInBranches(
          road_, ego_pose, ego_velocity, traffic_poses, scan_distance,
          &headway_distance);
  // const RoadOdometry<T>& lead_car_odom = PoseSelector::FindSingleClosestPose(
  //     ego_position.lane, ego_pose, ego_velocity, traffic_poses, scan_distance,
  //     WhichSide::kAhead, &headway_distance);
  /*
  std::cout << "IdmController::ImplDoCalcOutput [" << this->get_name() << "]: "
      << "Lead car odometry:\n"
      << "  - lane id: " << lead_car_odom.lane->id() << "\n"
      << "  - pos: " << lead_car_odom.pos << "\n"
      << "  - frame velocity: " << lead_car_odom.vel
      << std::endl;
  */
  const T& s_dot_ego = PoseSelector::GetIsoLaneVelocity(
      ego_position, ego_velocity).sigma_v;
  const T& s_dot_lead = PoseSelector::GetIsoLaneVelocity(
      {lead_car_odom.lane, lead_car_odom.pos}, lead_car_odom.vel).sigma_v;
  const bool ego_with_s = s_dot_ego > 0.;

  // Saturate the net_distance at distance_lower_bound away from the ego car to
  // avoid near-singular solutions inherent to the IDM equation.
  const T net_distance = saturate(
      headway_distance - idm_params.bloat_diameter(),
      idm_params.distance_lower_limit(), std::numeric_limits<T>::infinity());
  const T closing_velocity = cond(ego_with_s, s_dot_ego - s_dot_lead,
                                  s_dot_lead - s_dot_ego);

  // Compute the acceleration command from the IDM equation.
  using std::abs;
  (*command)[0] = IdmPlanner<T>::Evaluate(idm_params, abs(s_dot_ego),
                                          net_distance, closing_velocity);
  /*
  std::cout << "IdmController::ImplDoCalcOutput [" << this->get_name() << "]: "
      << "I/O of call to IdmPlanner<T>::Evaluate():\n"
      << "  - s_dot_ego = " << s_dot_ego << "\n"
      << "  - s_dot_lead = " << s_dot_lead << "\n"
      << "  - headway_distance = " << headway_distance << "\n"
      << "  - bloat_diameter = " << idm_params.bloat_diameter() << "\n"
      << "  - net_distance = " << net_distance << "\n"
      << "  - closing_velocity = " << closing_velocity << "\n"
      << "  - Acceleration command = " << (*command)[0]
      << std::endl;
  */
  std::cerr << " Net distance " << net_distance << std::endl;
  //std::cerr << " Closing velocity " << closing_velocity << std::endl;
  //std::cerr << " Ego speed " << s_dot_ego << std::endl;
  //std::cerr << " Acceleration command " << (*command)[0] << std::endl;
  using std::isnan;
  DRAKE_THROW_UNLESS(!isnan((*command)[0]));
}

// These instantiations must match the API documentation in idm_controller.h.
template class IdmController<double>;

}  // namespace automotive
}  // namespace drake
