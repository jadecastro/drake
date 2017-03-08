#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {

/// Describes the row indices of a MobilPlannerParameters.
struct MobilPlannerParametersIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 10;

  // The index of each individual coordinate.
  static const int kVRef = 0;
  static const int kA = 1;
  static const int kB = 2;
  static const int kS0 = 3;
  static const int kTimeHeadway = 4;
  static const int kDelta = 5;
  static const int kP = 6;
  static const int kThreshold = 7;
  static const int kMaxDeceleration = 8;
  static const int kSLookahead = 9;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class MobilPlannerParameters : public systems::BasicVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef MobilPlannerParametersIndices K;

  /// Default constructor.  Sets all rows to zero.
  MobilPlannerParameters() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  MobilPlannerParameters<T>* DoClone() const override {
    auto result = new MobilPlannerParameters;
    result->set_value(this->get_value());
    return result;
  }

  /// @name Getters and Setters
  //@{
  /// desired velocity in free traffic
  const T& v_ref() const { return this->GetAtIndex(K::kVRef); }
  void set_v_ref(const T& v_ref) { this->SetAtIndex(K::kVRef, v_ref); }
  /// max acceleration
  const T& a() const { return this->GetAtIndex(K::kA); }
  void set_a(const T& a) { this->SetAtIndex(K::kA, a); }
  /// comfortable braking deceleration
  const T& b() const { return this->GetAtIndex(K::kB); }
  void set_b(const T& b) { this->SetAtIndex(K::kB, b); }
  /// minimum desired net distance
  const T& s_0() const { return this->GetAtIndex(K::kS0); }
  void set_s_0(const T& s_0) { this->SetAtIndex(K::kS0, s_0); }
  /// desired time headway to vehicle in front
  const T& time_headway() const { return this->GetAtIndex(K::kTimeHeadway); }
  void set_time_headway(const T& time_headway) {
    this->SetAtIndex(K::kTimeHeadway, time_headway);
  }
  /// free-road exponent
  const T& delta() const { return this->GetAtIndex(K::kDelta); }
  void set_delta(const T& delta) { this->SetAtIndex(K::kDelta, delta); }
  /// politeness factor [0, 1]
  const T& p() const { return this->GetAtIndex(K::kP); }
  void set_p(const T& p) { this->SetAtIndex(K::kP, p); }
  /// acceleration threshold
  const T& threshold() const { return this->GetAtIndex(K::kThreshold); }
  void set_threshold(const T& threshold) {
    this->SetAtIndex(K::kThreshold, threshold);
  }
  /// maximum safe deceleration
  const T& max_deceleration() const {
    return this->GetAtIndex(K::kMaxDeceleration);
  }
  void set_max_deceleration(const T& max_deceleration) {
    this->SetAtIndex(K::kMaxDeceleration, max_deceleration);
  }
  /// distance along the s-direction to place the reference point
  const T& s_lookahead() const { return this->GetAtIndex(K::kSLookahead); }
  void set_s_lookahead(const T& s_lookahead) {
    this->SetAtIndex(K::kSLookahead, s_lookahead);
  }
  //@}
};

}  // namespace automotive
}  // namespace drake
