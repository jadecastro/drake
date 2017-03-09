#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/simple_car_config.h"
#inlcude "drake/automotive/lane_direction.h"
#include "drake/automotive/pose_selector.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// PurePursuitController --
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// Inputs:
///
/// Outputs:
///
/// @ingroup automotive_systems
template <typename T>
class PurePursuitController : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PurePursuitController)

  /// Constructor.
  PurePursuitController();
  ~PurePursuitController() override;

  /// Returns the port to the individual input/output ports.
  const systems::InputPortDescriptor<T>& driving_command_input() const;
  const systems::InputPortDescriptor<T>& lane_input() const;
  const systems::InputPortDescriptor<T>& ego_pose_input() const;
  const systems::OutputPortDescriptor<T>& driving_command_output() const;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  // 
  void ImplDoCalcOutput(const SimpleCarParams<T>& car_params,
                        const DrivingCommand<T>& input_command,
                        const LaneDirection& lane_direction,
                        const systems::rendering::PoseVector<T>& ego_pose,
                        DrivingCommand<T>* output_command) const;

  // 
  const GeoPosition ComputeGoalPoint(
      const PurePursuitController<T>& pp_params, const Lane* lane,
      const RoadPosition& position) const;

  // Indices for the input / output ports.
  int command_input_index_{};
  int lane_index_{};
  int ego_pose_index_{};
  int command_output_index_{};
};

}  // namespace automotive
}  // namespace drake
