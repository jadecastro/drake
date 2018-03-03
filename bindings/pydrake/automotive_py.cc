#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/calc_ongoing_road_position.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/maliput/api/road_geometry.h"
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

  m.doc() = "Bindings for Automotive systems.";

  py::module::import("pydrake.systems.framework");

  // TODO(jadecastro) Bind AutoDiffXd.
  // TODO(jadecastro) Bind symbolic::Expression.
  using T = double;

  using pysystems::DefClone;

  py::enum_<RoadPositionStrategy>(m, "RoadPositionStrategy")
      .value("kCache", RoadPositionStrategy::kCache)
      .value("kExhaustiveSearch", RoadPositionStrategy::kExhaustiveSearch);

  py::enum_<ScanStrategy>(m, "ScanStrategy")
      .value("kPath", ScanStrategy::kPath)
      .value("kBranches", ScanStrategy::kBranches);

  py::class_<LaneDirection>(m, "LaneDirection")
      .def(py::init<const maliput::api::Lane*, bool>());
  pysystems::AddValueInstantiation<LaneDirection>(m);

  // TODO(jadecastro) How to write the below instantiation against
  // "maliput::api"?
  // Use BasicVector's approach to expose base class members here.
  py::class_<IdmController<T>, LeafSystem<T>>(m, "IdmController")
      .def(py::init<const maliput::api::RoadGeometry&,
           ScanStrategy, RoadPositionStrategy, double>())
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

  // TODO: Just add to the ctor?
  m.def("create_lane_direction",
        [](const maliput::api::Lane* lane, bool with_s) {
          return new LaneDirection(lane, with_s);
        });

  py::class_<DrivingCommand<T>, BasicVector<T>>(m, "DrivingCommand")
      .def(py::init<>())
      .def("steering_angle", &DrivingCommand<T>::steering_angle)
      .def("acceleration", &DrivingCommand<T>::acceleration)
      .def("set_steering_angle", &DrivingCommand<T>::set_steering_angle,
           py::arg("steering_angle"))
      .def("set_acceleration", &DrivingCommand<T>::set_acceleration,
           py::arg("acceleration"));
}

}  // namespace pydrake
}  // namespace drake
