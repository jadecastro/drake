#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/systems/rendering/frame_velocity.h"
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

  py::class_<FrameVelocity<T>, BasicVector<T>> frame_velocity(
      m, "FrameVelocity");
  frame_velocity
    .def(py::init());

  frame_velocity.attr("kSize") = int{FrameVelocity<T>::kSize};

  m.def("create_frame_velocity",
        [](const Vector3<T>& w, const Vector3<T>& v) {
          FrameVelocity<T> frame_velocity;
          frame_velocity.set_velocity(multibody::SpatialVelocity<T>(w, v));
          return frame_velocity;
        }, py::arg("w"), py::arg("v"));

  // TODO(eric.cousineau): Add more systems as needed.
}

}  // namespace pydrake
}  // namespace drake
