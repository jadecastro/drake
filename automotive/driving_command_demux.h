#pragma once

#include <memory>

#include "drake/automotive/gen/driving_command.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// This system splits a vector valued signal on its input into multiple
/// DrivingCommand outputs.
///
/// The input to this system directly feeds through to its output.
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
/// @ingroup primitive_systems
template <typename T>
class DrivingCommandDemux final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrivingCommandDemux)

  /// Constructs %DrivingCommandDemux with @p num_output_ports output ports and
  /// one vector valued input port of size 2 * num_output_ports.
  ///
  /// @p output_ports_sizes must exactly divide the expected size of the input,
  /// otherwise a std::runtime_error is thrown.
  ///
  /// @param size is the number of DrivingCommand output ports.
  explicit DrivingCommandDemux(int num_output_ports);

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit DrivingCommandDemux(const DrivingCommandDemux<U>&);

 private:
  // Sets the port_index-th output port value.
  void CopyToOutput(const systems::Context<T>& context,
                    systems::OutputPortIndex port_index,
                    systems::BasicVector<T>* output) const;
};

}  // namespace automotive
}  // namespace drake
