#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/automotive/gen/driving_command.h"
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

  py::class_<SimpleCar<T>, LeafSystem<T>>(m, "SimpleCar")
      .def(py::init<>())
      .def("state_output", &SimpleCar<T>::state_output)
      .def("pose_output", &SimpleCar<T>::pose_output)
      .def("velocity_output", &SimpleCar<T>::velocity_output);

  m.def("create_driving_command", []() -> BasicVector<T>* {
      return new DrivingCommand<T>();
    });
}

}  // namespace pydrake
}  // namespace drake
