#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/simple_car_config.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// PurePursuitController 
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
  PurePursuitController(std::unique_ptr<RoadGeometry> road);
  ~PurePursuitController() override;

  /// Returns the port to the individual input/output ports.
  const systems::InputPortDescriptor<T>& driving_command_input() const;
  const systems::InputPortDescriptor<T>& lane_input() const;
  const systems::InputPortDescriptor<T>& ego_pose_input() const;
  const systems::OutputPortDescriptor<T>& driving_command_output() const;

  // System<T> overrides.
  // The output of this system is an algebraic relation of its inputs.
  bool has_any_direct_feedthrough() const override { return true; }

  /// Sets the parameters with the default parameters for SimpleCar.
  void SetDefaultParameters(const systems::LeafContext<T>& context,
                            systems::Parameters<T>* params) const override;

 protected:
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::OutputPortDescriptor<T>& descriptor) const override;
  std::unique_ptr<systems::Parameters<T>> AllocateParameters() const override;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void ImplDoCalcOutput(const PurePursuitControllerParameters<T>& pp_params,
                        const SimpleCarConfig<T>& car_params,
                        const DrivingCommand<T>& input_command,
                        const maliput::api::LaneId& lane_id,
                        const systems::rendering::PoseVector<T>& ego_pose,
                        DrivingCommand<T>* output_command) const;

  const maliput::api::GeoPosition ComputeGoalPoint(
      const PurePursuitControllerParameters<T>& params,
      const maliput::api::Lane* lane,
      const maliput::api::RoadPosition& position) const;

  const std::unique_ptr<maliput::api::RoadGeometry> road_{};
};

}  // namespace automotive
}  // namespace drake
