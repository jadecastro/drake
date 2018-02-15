#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_base.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(trajectories, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for classes in the common/trajectories subfolder.";

  py::module::import("pydrake.systems.framework");

  py::class_<Polynomial<double>>(m, "Polynomial")
      .def(py::init<>())
      .def(py::init<const Polynomial<double>::CoefficientType&>());
  // TODO(jadecastro) This should probably live inside common_py.cc?
  // TODO(jadecastro) Expose all relevant constructors and member functions.

  /*
  py::class_<PiecewisePolynomialBase, PiecewiseFunction>(
      m, "PiecewisePolynomialBase")
      .def(py::init<std::vector<double> const&>())
      .def("getNumberOfCoefficients",
           &PiecewisePolynomialBase::getNumberOfCoefficients)
      .def("getTotalNumberOfCoefficients",
           &PiecewisePolynomialBase::getTotalNumberOfCoefficients);
  */
  py::class_<PiecewisePolynomial<double>>(m, "PiecewisePolynomial");

  py::class_<PiecewisePolynomialTrajectory>ppt(
      m, "PiecewisePolynomialTrajectory");
  pysystems::DefClone(&ppt);
  ppt
      .def(py::init<const PiecewisePolynomial<double>&>());

  /*
  m.def("create_piecewise_polynomial", [](
      std::vector<PiecewisePolynomial<double>::PolynomialMatrix> const&
      polynomials,
      std::vector<double> const& breaks) -> PiecewisePolynomialBase* {
          // TODO(jadecastro) Should this be PiecewisePolynomial?
          return new PiecewisePolynomial<double>(polynomials, breaks);
        });
  */
  // TODO(jadecastro) The compiler balks on the above due to PolynomialMatrix.

  m.def("create_zoh_piecewise_polynomial", [](
      const std::vector<double>& breaks,
      const std::vector<PiecewisePolynomial<double>::CoefficientMatrix>&
      knots) -> PiecewisePolynomial<double> {
          return PiecewisePolynomial<double>::ZeroOrderHold(breaks, knots);
        });

  m.def("create_foh_piecewise_polynomial", [](
      const std::vector<double>& breaks,
      const std::vector<PiecewisePolynomial<double>::CoefficientMatrix>&
      knots) -> PiecewisePolynomial<double> {
          return PiecewisePolynomial<double>::FirstOrderHold(breaks, knots);
        });
}

}  // namespace pydrake
}  // namespace drake
