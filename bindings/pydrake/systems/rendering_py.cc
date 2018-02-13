#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/systems/rendering/pose_aggregator.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(rendering, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::rendering;

  m.doc() = "Bindings for the rendering portion of the Systems framework.";

  py::module::import("pydrake.systems.framework");

  using T = double;

  py::class_<PoseAggregator<T>, LeafSystem<T>> pose_aggregator(
      m, "PoseAggregator");
  pose_aggregator
    .def(py::init())
    .def("AddSingleInput", &PoseAggregator<T>::AddSingleInput)
    .def("AddSinglePoseAndVelocityInput",
         &PoseAggregator<T>::AddSinglePoseAndVelocityInput)
    .def("AddBundleInput", &PoseAggregator<T>::AddBundleInput)
    .def("set_name", &PoseAggregator<T>::set_name);

  py::class_<PoseVector<T>, BasicVector<T>> pose_vector(m, "PoseVector");
  pose_vector
    .def(py::init())
    .def("get_isometry", &PoseVector<T>::get_isometry)
    .def("get_translation", &PoseVector<T>::get_translation)
    .def("set_translation", &PoseVector<T>::set_translation)
    .def("get_rotation", &PoseVector<T>::get_rotation)
    .def("set_rotation", &PoseVector<T>::set_rotation);

  pose_vector.attr("kSize") = int{PoseVector<T>::kSize};

  // TODO(eric.cousineau): Add more systems as needed.
}

}  // namespace pydrake
}  // namespace drake
