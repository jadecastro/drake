#pragma once

#include "drake/automotive/gen/driving_command.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port.h"

namespace drake {
namespace automotive {

/// An adder for arbitrarily many inputs of type DrivingCommand.
/// @tparam T The type of mathematical object being added.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class DrivingCommandAdder final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrivingCommandAdder)

  /// Construct an %DrivingCommandAdder System.
  /// @param num_inputs is the number of DrivingCommand input ports to be added.
  DrivingCommandAdder(int num_inputs);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit DrivingCommandAdder(const DrivingCommandAdder<U>&);

  /// Returns the output port on which the sum is presented.
  const systems::OutputPort<T>& get_output_port() const {
    return systems::LeafSystem<T>::get_output_port(0);
  }

 private:
  // Sums the input ports into a DrivingCommand-typed output port. If the input
  // ports are not the appropriate count, a std::runtime_error will be thrown.
  void CalcSum(const systems::Context<T>& context,
               DrivingCommand<T>* sum) const;
};

}  // namespace automotive
}  // namespace drake
