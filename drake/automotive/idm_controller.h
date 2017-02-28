#pragma once

#include <memory>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// IdmController -- an IDM (Intelligent Driver Model) planner.
///
/// IDM: Intelligent Driver Model:
///    https://en.wikipedia.org/wiki/Intelligent_driver_model
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::TaylorVarXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
///
/// Inputs:
///   Port 0:
///      @p pose_ego PoseBundle for the ego car.
///   Port 1:
///      @p pose_agent PoseBundle for the traffic cars.
/// Outputs:
///   Port 0:
///      @p vdot_ego linear acceleration of the ego car (scalar) [m/s^2].
template <typename T>
class IdmController : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IdmController)

  /// @p v_ref desired velocity of the ego car in units of m/s.
  explicit IdmController(const T& v_ref);
  ~IdmController() override;

  /// Returns the port to the ego car input subvector.
  const systems::InputPortDescriptor<T>& get_ego_input() const;
  const systems::InputPortDescriptor<T>& get_agent_input() const;

  // System<T> overrides.
  // The output of this system is an algebraic relation of its inputs.
  bool has_any_direct_feedthrough() const override { return true; }

  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::OutputPortDescriptor<T>& descriptor) const override;

  std::unique_ptr<systems::Parameters<T>> AllocateParameters() const override;

  void SetDefaultParameters(const systems::LeafContext<T>& context,
                            systems::Parameters<T>* params) const override;

  static T IdmModel(const T& net_distance,
                    const T& closing_velocity,
                    const T& ego_velocity,
                    const IdmPlannerParameters<T>& params) {
    const T& v_ref = params.v_ref();
    const T& a = params.a();
    const T& b = params.b();
    const T& s_0 = params.s_0();
    const T& time_headway = params.time_headway();
    const T& delta = params.delta();

    DRAKE_DEMAND(a > 0.0);
    DRAKE_DEMAND(b > 0.0);

    const T position_deficit =
        ego_velocity * closing_velocity / (2 * sqrt(a * b));

    return a * (1. - pow(ego_velocity / v_ref, delta) -
                pow((s_0 + ego_velocity * time_headway + position_deficit) /
                    net_distance, 2.));
  };

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void ImplDoCalcOutput(const systems::rendering::PoseVector<T>& ego_pose,
                        const systems::rendering::PoseVector<T>& agents_pose,
                        const IdmPlannerParameters<T>& params,
                        DrivingCommand<T>* output) const;

  const T v_ref_;  // Desired vehicle velocity.
};

}  // namespace automotive
}  // namespace drake
