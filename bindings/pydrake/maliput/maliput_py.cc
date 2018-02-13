#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

using std::make_unique;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(api, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::maliput::api;

  m.doc() = "Bindings for Maliput.";

  py::class_<RoadGeometryId>(m, "RoadGeometryId")
      .def(py::init<std::string>());
}

}  // namespace pydrake
}  // namespace drake
