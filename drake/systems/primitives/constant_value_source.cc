// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/systems/primitives/constant_value_source-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

// Explicitly instantiates on the most common scalar types.
template class ConstantValueSource<double>;
template class ConstantValueSource<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
