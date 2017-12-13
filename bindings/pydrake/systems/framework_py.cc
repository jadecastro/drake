#include <pybind11/eigen.h>
#include <pybind11/eval.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/common/nice_type_name.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/supervector.h"
#include "drake/systems/framework/system.h"

namespace py = pybind11;

using std::make_unique;
using std::unique_ptr;
using std::vector;

PYBIND11_MODULE(framework, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  // Aliases for commonly used return value policies.
  // `py_ref` is used when `keep_alive` is explicitly used (e.g. for extraction
  // methods, like `GetMutableSubsystemState`).
  auto py_ref = py::return_value_policy::reference;
  // `py_iref` is used when pointers / lvalue references are returned (no need
  // for `keep_alive`, as it is implicit.
  auto py_iref = py::return_value_policy::reference_internal;

  m.doc() = "Bindings for the core Systems framework.";

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  // TODO(eric.cousineau): Resolve `str_py` workaround.
  auto str_py = py::eval("str");

  // TODO(eric.cousineau): Show constructor, but somehow make sure `pybind11`
  // knows this is abstract?
  py::class_<System<T>>(m, "System")
    .def("set_name", &System<T>::set_name)
    .def("get_input_port", &System<T>::get_input_port, py_iref)
    .def("get_output_port", &System<T>::get_output_port, py_iref)
    .def("CreateDefaultContext", &System<T>::CreateDefaultContext)
    .def("AllocateOutput", &System<T>::AllocateOutput)
    .def(
        "GetGraphvizString",
        [str_py](const System<T>* self) {
          // @note This is a workaround; for some reason,
          // casting this using `py::str` does not work, but directly
          // calling the Python function (`str_py`) does.
          return str_py(self->GetGraphvizString());
        });

  py::class_<LeafSystem<T>, System<T>>(m, "LeafSystem");

  py::class_<Context<T>>(m, "Context")
    .def("FixInputPort",
         py::overload_cast<int, unique_ptr<BasicVector<T>>>(
             &Context<T>::FixInputPort))
    .def("get_time", &Context<T>::get_time)
    .def("Clone", &Context<T>::Clone)
    .def("__copy__", &Context<T>::Clone)
    .def("get_state", &Context<T>::get_state, py_iref)
    .def("get_mutable_state", &Context<T>::get_mutable_state, py_iref);

  py::class_<LeafContext<T>, Context<T>>(m, "LeafContext");

  py::class_<Diagram<T>, System<T>>(m, "Diagram")
    .def("GetMutableSubsystemState",
        [](Diagram<T>* self, const System<T>& arg1, Context<T>* arg2)
        -> auto&& {
          // @note Use `auto&&` to get perfect forwarding.
          // @note Compiler does not like `py::overload_cast` with this setup?
          return self->GetMutableSubsystemState(arg1, arg2);
        }, py_ref, py::keep_alive<0, 3>());

  // Glue mechanisms.
  py::class_<DiagramBuilder<T>>(m, "DiagramBuilder")
    .def(py::init<>())
    .def(
        "AddSystem",
        [](DiagramBuilder<T>* self, unique_ptr<System<T>> arg1) {
          return self->AddSystem(std::move(arg1));
        })
    .def("Connect",
         py::overload_cast<const OutputPort<T>&, const InputPortDescriptor<T>&>(
             &DiagramBuilder<T>::Connect))
    .def("ExportInput", &DiagramBuilder<T>::ExportInput)
    .def("ExportOutput", &DiagramBuilder<T>::ExportOutput)
    .def("Build", &DiagramBuilder<T>::Build)
    .def("BuildInto", &DiagramBuilder<T>::BuildInto);

  py::class_<OutputPort<T>>(m, "OutputPort");
  py::class_<SystemOutput<T>>(m, "SystemOutput");

  py::class_<InputPortDescriptor<T>>(m, "InputPortDescriptor");

  // Value types.
  py::class_<VectorBase<T>>(m, "VectorBase")
    .def("CopyToVector", &VectorBase<T>::CopyToVector)
    .def("SetFromVector", &VectorBase<T>::SetFromVector);

  py::class_<BasicVector<T>, VectorBase<T>>(m, "BasicVector")
    .def(py::init<VectorX<T>>())
    .def("get_value", &BasicVector<T>::get_value);

  py::class_<Supervector<T>, VectorBase<T>>(m, "Supervector");

  py::class_<Subvector<T>, VectorBase<T>>(m, "Subvector");

  // TODO(eric.cousineau): Interfacing with the C++ abstract value types may be
  // a tad challenging. This should be more straightforward once
  // scalar-type conversion is supported, as the template-exposure mechanisms
  // should be relatively similar.
  py::class_<AbstractValue>(m, "AbstractValue");

  // Parameters.
  // TODO(eric.cousineau): Fill this out.
  py::class_<Parameters<T>>(m, "Parameters");

  // State.
  py::class_<State<T>>(m, "State")
    .def(py::init<>())
    .def("get_continuous_state",
         &State<T>::get_continuous_state, py_iref)
    .def("get_mutable_continuous_state",
         &State<T>::get_mutable_continuous_state, py_iref);

  // - Constituents.
  py::class_<ContinuousState<T>>(m, "ContinuousState")
    .def(py::init<>())
    .def("get_vector", &ContinuousState<T>::get_vector, py_iref)
    .def("get_mutable_vector",
         &ContinuousState<T>::get_mutable_vector, py_iref);

  py::class_<DiscreteValues<T>>(m, "DiscreteValues");

  py::class_<AbstractValues>(m, "AbstractValues");
}
