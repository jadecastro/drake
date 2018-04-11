#pragma once

#include <Eigen/Geometry>

#include "drake/automotive/agent_trajectory.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/extract_double.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;

/// Container with static data for the agent model.
struct AgentData {
  enum class AgentType { kCar, kBicycle, kPedestrian };
  // TODO(jadecastro) Add more types as necessary.

  explicit AgentData(const AgentType& t) : type(t) {}

  AgentType type{AgentType::kCar};
};

// For convenience, we provide a ready-made specialization for the kCar type.
// ** TODO: consider whether to do this or:
//   1. Provide a ready-made specialization for TrajectoryCar
//   2. Provide a constructor for TrajectoryAgent so that we have
//      TrajectoryAgent(AgentType::kCar, trajectory)
struct CarAgent : AgentData {
  CarAgent() : AgentData(AgentType::kCar) {}
};

/// TrajectoryAgent models an agent that follows a pre-established trajectory.
///
/// output port 0:
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
///   heading is defined around the +z axis, so positive-turn-left
/// * velocity
///   (OutputPort getter: raw_pose_output())
///
/// output port 1: A PoseVector containing X_WC, where C is the car frame.
///   (OutputPort getter: pose_output())
///
/// output port 2: A FrameVelocity containing Xdot_WC, where C is the car frame.
///   (OutputPort getter: velocity_output())
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_plants
template <typename T>
class TrajectoryAgent final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryAgent)

  /// Constructs a TrajectoryAgent system that traces a given two-dimensional @p
  /// curve.  Throws an error if the curve is empty (has a zero @p path_length).
  explicit TrajectoryAgent(const AgentData& agent_data,
                           const AgentTrajectory& trajectory)
      : systems::LeafSystem<T>(
            systems::SystemTypeTag<automotive::TrajectoryAgent>{}),
        agent_data_(agent_data),
        trajectory_(trajectory) {
    if (trajectory_.empty()) {
      throw std::invalid_argument{"empty trajectory"};
    }
    this->DeclareVectorOutputPort(&TrajectoryAgent::CalcStateOutput);
    this->DeclareVectorOutputPort(&TrajectoryAgent::CalcPoseOutput);
    this->DeclareVectorOutputPort(&TrajectoryAgent::CalcVelocityOutput);
  }

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit TrajectoryAgent(const TrajectoryAgent<U>& other)
      : TrajectoryAgent<T>(other.agent_data_, other.trajectory_) {}

  /// See class description for details about the following ports.
  /// @{
  const systems::OutputPort<T>& raw_pose_output() const {
    return this->get_output_port(0);
  }
  const systems::OutputPort<T>& pose_output() const {
    return this->get_output_port(1);
  }
  const systems::OutputPort<T>& velocity_output() const {
    return this->get_output_port(2);
  }
  /// @}

 private:
  void CalcStateOutput(const systems::Context<T>& context,
                       SimpleCarState<T>* output_vector) const {
    ImplCalcOutput(GetValues(context), output_vector);
  }

  void CalcPoseOutput(const systems::Context<T>& context,
                      systems::rendering::PoseVector<T>* pose) const {
    ImplCalcPose(GetValues(context), pose);
  }

  void CalcVelocityOutput(
      const systems::Context<T>& context,
      systems::rendering::FrameVelocity<T>* velocity) const {
    ImplCalcVelocity(GetValues(context), velocity);
  }

  // Allow different specializations to access each other's private data.
  template <typename>
  friend class TrajectoryAgent;

  void ImplCalcOutput(const TrajectoryAgentValues& values,
                      SimpleCarState<T>* output) const {
    // Convert TrajectoryAgentValues into a SimpleCarState.
    // *** Need to convert x, y, below to Body frame ***
    output->set_x(T{values.pose().get_translation().x()});
    output->set_y(T{values.pose().get_translation().y()});
    // TODO(jadecastro) Enable Autodiff support for
    // math::QuaternionToSpaceXYZ().
    const Eigen::Vector4d vector{T{values.pose().get_rotation().w()},
                                 T{values.pose().get_rotation().x()},
                                 T{values.pose().get_rotation().y()},
                                 T{values.pose().get_rotation().z()}};
    output->set_heading(math::QuaternionToSpaceXYZ(vector).z());
    output->set_velocity(
        T{values.velocity().get_velocity().translational().norm()});
  }

  void ImplCalcPose(const TrajectoryAgentValues& values,
                    systems::rendering::PoseVector<T>* pose) const {
    // Convert the TrajectoryAgentValues into a pose vector.
    pose->set_translation(T{values.pose().get_translation()});
    pose->set_rotation(T{values.pose().get_rotation()});
  }

  void ImplCalcVelocity(const TrajectoryAgentValues& values,
                        systems::rendering::FrameVelocity<T>* velocity) const {
    // Convert the TrajectoryAgentValues into a spatial velocity.
    velocity->set_velocity(
        SpatialVelocity<T>{values.velocity().get_velocity()});
  }

  // Extract the appropriately-typed state from the context.
  TrajectoryAgentValues GetValues(const systems::Context<T>& context) const {
    return trajectory_.value(ExtractDoubleOrThrow(context.get_time()));
  }

  const AgentData agent_data_;
  const AgentTrajectory trajectory_;
};

}  // namespace automotive

namespace systems {
namespace scalar_conversion {
// Disable symbolic support, because we use ExtractDoubleOrThrow.
template <>
struct Traits<automotive::TrajectoryAgent> : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems

}  // namespace drake
