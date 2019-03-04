#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/loader.h"
#include "drake/automotive/maliput/multilane/road_geometry.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

using std::make_unique;
using std::unique_ptr;

PYBIND11_MODULE(multilane, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::maliput;
  constexpr auto& doc = pydrake_doc.drake.maliput;

  m.doc() = "Bindings for the Multilane backend.";

  py::class_<multilane::RoadGeometry, api::RoadGeometry>(
      m, "RoadGeometry", doc.multilane.RoadGeometry.doc);

  py::class_<multilane::BuilderFactoryBase>(
      m, "BuilderFactoryBase", doc.multilane.BuilderFactoryBase.doc);

  py::class_<multilane::BuilderFactory, multilane::BuilderFactoryBase>(
      m, "BuilderFactory", doc.multilane.BuilderFactory.doc)
      .def(py::init<>());

  m.def("LoadFile", &multilane::LoadFile, doc.multilane.LoadFile.doc);
}

}  // namespace pydrake
}  // namespace drake
