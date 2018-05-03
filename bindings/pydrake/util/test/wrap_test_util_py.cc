#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"

namespace drake {
namespace pydrake {
namespace {

// Another simple struct with a value.
struct MyValue {
  double value{0.};
};

// A simple struct with a mutable bare pointer member.
struct MyContainer {
  double get_value() const { return member->value; }  // ** For testing only... we'll get rid of this.
  const MyValue* member{nullptr};
};

}  // namespace

PYBIND11_MODULE(wrap_test_util, m) {
  py::class_<MyValue>(m, "MyValue")
      .def(py::init<>())
      .def_readwrite("value", &MyValue::value, py_reference_internal);

  py::class_<MyContainer> my_container(m, "MyContainer");
  my_container
      .def(py::init<>())
      .def("get_value", &MyContainer::get_value);
      // .def_readwrite("member", &MyContainer::member);  // Shouln't work!
  DefReadWriteKeepAlive(&my_container, "member", &MyContainer::member);
}

}  // namespace pydrake
}  // namespace drake
