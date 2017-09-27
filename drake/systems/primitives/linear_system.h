#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/systems/primitives/affine_system.h"

namespace drake {
namespace systems {

// TODO(jadecastro) Update this documentation.......

/// A discrete OR continuous linear system.
///
/// If time_period>0.0, then the linear system will have the following discrete-
/// time state update:
///   @f[ x[n+1] = A x[n] + B u[n], @f]
///
/// or if time_period==0.0, then the linear system will have the following
/// continuous-time state update:
///   @f[\dot{x} = A x + B u. @f]
///
/// In both cases, the system will have the output:
///   @f[y = C x + D u, @f]
/// where `u` denotes the input vector, `x` denotes the state vector, and
/// `y` denotes the output vector.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
///
/// @ingroup primitive_systems
///
/// @see AffineSystem
/// @see MatrixGain
template <typename T>
class LinearSystem : public AffineSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSystem)

  /// Constructs a %LinearSystem with a fixed set of coefficient matrices `A`,
  /// `B`,`C`, and `D`.
  /// The coefficient matrices must obey the following dimensions:
  /// | Matrix  | Num Rows    | Num Columns |
  /// |:-------:|:-----------:|:-----------:|
  /// | A       | num states  | num states  |
  /// | B       | num states  | num inputs  |
  /// | C       | num outputs | num states  |
  /// | D       | num outputs | num inputs  |
  ///
  /// Subclasses must use the protected constructor, not this one.
  LinearSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
               const Eigen::Ref<const Eigen::MatrixXd>& B,
               const Eigen::Ref<const Eigen::MatrixXd>& C,
               const Eigen::Ref<const Eigen::MatrixXd>& D,
               double time_period = 0.0);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit LinearSystem(const LinearSystem<U>&);

  /// Creates a unique pointer to LinearSystem<T> by decomposing @p dynamics and
  /// @p outputs using @p state_vars and @p input_vars.
  ///
  /// @throws runtime_error if either @p dynamics or @p outputs is not linear in
  /// @p state_vars and @p input_vars.
  static std::unique_ptr<LinearSystem<T>> MakeLinearSystem(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& dynamics,
      const Eigen::Ref<const VectorX<symbolic::Expression>>& output,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& state_vars,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& input_vars,
      double time_period = 0.0);

 protected:
  /// Constructor that specifies scalar-type conversion support.
  /// @param converter scalar-type conversion support helper (i.e., AutoDiff,
  /// etc.); pass a default-constructed object if such support is not desired.
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  LinearSystem(SystemScalarConverter converter,
               const Eigen::Ref<const Eigen::MatrixXd>& A,
               const Eigen::Ref<const Eigen::MatrixXd>& B,
               const Eigen::Ref<const Eigen::MatrixXd>& C,
               const Eigen::Ref<const Eigen::MatrixXd>& D,
               double time_period);
};

enum WhichAction { Linearize, Throw };

struct LinearizationData {
  // Default constructor.
  LinearizationData() = default;

  // Fully-parameterized constructor.
  LinearizationData(std::unique_ptr<LinearSystem<double>> linear_system_in,
                    Eigen::VectorXd f0_in)
      : linear_system(std::move(linear_system_in)), f0(f0_in) {}

  std::unique_ptr<LinearSystem<double>> linear_system;
  Eigen::VectorXd f0;
};

/// Takes the first-order Taylor expansion of a System around a nominal
/// operating point (defined by the Context).
///
/// @param system The system or subsystem to linearize.
/// @param context Defines the nominal operating point about which the system
/// should be linearized.  See note below.
/// @param equilibrium_check_tolerance Specifies the tolerance on ensuring that
/// the derivative vector isZero at the nominal operating point.  @default 1e-6.
/// @param which_action tells the function how to handle non-equilibrium context
/// The function may either throw a runtime error or else attempt to linearize
/// about that non-equilibrium condition.
/// @returns A LinearSystem that approximates the original system in the
/// vicinity of the operating point.  See note below.
/// @throws std::runtime_error if the system the operating point is not an
/// equilibrium point of the system (within the specified tolerance)
///
/// Note: The inputs in the Context must be connected, either to the
/// output of some upstream System within a Diagram (e.g., if system is a
/// reference to a subsystem in a Diagram), or to a constant value using, e.g.
///   context->FixInputPort(0,default_input);
///
/// Note: The inputs, states, and outputs of the returned system are NOT the
/// same as the original system.  Denote x0,u0 as the nominal state and input
/// defined by the Context, and y0 as the value of the output at (x0,u0),
/// then the created systems inputs are (u-u0), states are (x-x0), and
/// outputs are (y-y0).
///
/// @ingroup primitive_systems
///
std::unique_ptr<LinearSystem<double>> Linearize(
    const System<double>& system, const Context<double>& context,
    double equilibrium_check_tolerance = 1e-6);

///
///
LinearizationData LinearizeAboutNonequilibrium(
    const System<double>& system, const Context<double>& context,
    double equilibrium_check_tolerance = 1e-6);

/// Returns the controllability matrix:  R = [B, AB, ..., A^{n-1}B].
/// @ingroup control_systems
Eigen::MatrixXd ControllabilityMatrix(const LinearSystem<double>& sys);

/// Returns true iff the controllability matrix is full row rank.
/// @ingroup control_systems
bool IsControllable(const LinearSystem<double>& sys,
                    double threshold = Eigen::Default);

/// Returns the observability matrix: O = [ C; CA; ...; CA^{n-1} ].
/// @ingroup estimator_systems
Eigen::MatrixXd ObservabilityMatrix(const LinearSystem<double>& sys);

/// Returns true iff the observability matrix is full column rank.
/// @ingroup estimator_systems
bool IsObservable(const LinearSystem<double>& sys,
                  double threshold = Eigen::Default);

}  // namespace systems
}  // namespace drake
