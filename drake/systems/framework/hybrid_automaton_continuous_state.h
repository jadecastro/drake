#pragma once

// TODO: triage this list.
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/systems/framework/vector_base.h"
#include "drake/systems/framework/continuous_state.h"

namespace drake {
namespace systems {
namespace internal {

using std::make_unique;
using std::unique_ptr;

/// MutableVector specializes VectorBase, enabling construction of a mutable
/// VectorBase object created elsewhere.  This is an implementation detail
/// required for HybridAutomatonContinuousState. User code should not, in
/// general, interact with this class.
template <typename T>
class MutableVector : public VectorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MutableVector)

  explicit MutableVector(VectorBase<T>* vector) : vector_(vector) {}

  ~MutableVector() override {}

  int size() const override { return vector_->size(); }

  const T& GetAtIndex(const int index) const override {
    return vector_->GetAtIndex(index);
  }

  T& GetAtIndex(const int index) override {
    return vector_->GetAtIndex(index);
  }

 private:
  VectorBase<T>* vector_;
};

// HybridAutomatonContinuousState specializes ContinuousState by providing a
// constructor Its purpose is to make construction of ContinuousState
// possible.
template <typename T>
class HybridAutomatonContinuousState : public ContinuousState<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HybridAutomatonContinuousState)

  /// Constructs a ContinuousState that is composed of other ContinuousStates,
  /// which are not owned by this object.
  explicit HybridAutomatonContinuousState(VectorBase<T>* state,
                                          int num_q, int num_v, int num_z)
  : ContinuousState<T>(set_and_own(state), num_q, num_v, num_z) {}

  ~HybridAutomatonContinuousState() override {}

 private:
  static unique_ptr<VectorBase<T>> set_and_own(VectorBase<T>* vector) {
    return make_unique<MutableVector<T>>(vector);
  }
};

}  // namespace internal
}  // namespace systems
}  // namespace drake
