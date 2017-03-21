#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {

/// Describes the row indices of a PurePursuitControllerParameters.
struct PurePursuitControllerParametersIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 1;

  // The index of each individual coordinate.
  static const int kSLookahead = 0;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class PurePursuitControllerParameters : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef PurePursuitControllerParametersIndices K;

  /// Default constructor.  Sets all rows to zero.
  PurePursuitControllerParameters()
      : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  PurePursuitControllerParameters<T>* DoClone() const override {
    return new PurePursuitControllerParameters;
  }

  /// @name Getters and Setters
  //@{
  /// distance along the s-direction to place the reference point
  const T& s_lookahead() const { return this->GetAtIndex(K::kSLookahead); }
  void set_s_lookahead(const T& s_lookahead) {
    this->SetAtIndex(K::kSLookahead, s_lookahead);
  }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(s_lookahead());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
