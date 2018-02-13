#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/calc_ongoing_road_position.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/pure_pursuit_controller.h"
#include "drake/automotive/simple_car.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(automotive, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::automotive;

  m.doc() = "Bindings for the SimpleCar plant.";

  py::module::import("pydrake.systems.framework");

  // TODO(jadecastro) Bind AutoDiffXd.
  // TODO(jadecastro) Bind symbolic::Expression.
  using T = double;

  using pysystems::DefClone;

  py::enum_<RoadPositionStrategy>(m, "RoadPositionStrategy")
      .value("kCache", RoadPositionStrategy::kCache)
      .value("kExhaustiveSearch", RoadPositionStrategy::kExhaustiveSearch);

  py::class_<LaneDirection>(m, "LaneDirection")
      .def(py::init<const maliput::dragway::Lane*, bool>());
  pysystems::AddValueInstantiation<LaneDirection>(m);

  // TODO(jadecastro) How to write the below instantiation against
  // "maliput::api"?
  // Use BasicVector's approach to expose base class members here.
  py::class_<IdmController<T>, LeafSystem<T>>(m, "IdmController")
      .def(py::init<const maliput::dragway::RoadGeometry&,
           RoadPositionStrategy, double>())
      .def("ego_pose_input", &IdmController<T>::ego_pose_input)
      .def("ego_velocity_input", &IdmController<T>::ego_velocity_input)
      .def("traffic_input", &IdmController<T>::traffic_input);

  py::class_<PurePursuitController<T>, LeafSystem<T>>(
      m, "PurePursuitController")
      .def(py::init<>())
      .def("ego_pose_input", &PurePursuitController<T>::ego_pose_input)
      .def("lane_input", &PurePursuitController<T>::lane_input)
      .def("steering_command_output",
           &PurePursuitController<T>::steering_command_output);

  py::class_<SimpleCar<T>, LeafSystem<T>>(m, "SimpleCar")
      .def(py::init<>())
      .def("state_output", &SimpleCar<T>::state_output)
      .def("pose_output", &SimpleCar<T>::pose_output)
      .def("velocity_output", &SimpleCar<T>::velocity_output);

  m.def("create_lane_direction",
        [](const maliput::dragway::Lane* lane, bool with_s) {
          return new LaneDirection(lane, with_s);
        });

  m.def("create_driving_command", []() -> BasicVector<T>* {
      return new DrivingCommand<T>();
    });
}

}  // namespace pydrake
}  // namespace drake
