#include "drake/systems/framework/hybrid_automaton_continuous_state.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {
namespace internal {
namespace {

const int kSize{6};

class HybridAutomatonContinuousStateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    basic_vector_ = BasicVector<int>::Make({1, 1, 2, 3, 5, 8});
    mutable_vector_ = std::make_unique<MutableVector<int>>(basic_vector_.get());
  }

  std::unique_ptr<VectorBase<int>> basic_vector_;
  std::unique_ptr<MutableVector<int>> mutable_vector_;
};

// Tests the overridden getters work properly.
TEST_F(HybridAutomatonContinuousStateTest, GetAtIndex) {
  ASSERT_EQ(kSize, mutable_vector_->size());
  EXPECT_EQ(1, mutable_vector_->GetAtIndex(0));
}

// Tests that the a change in the mutable vector reflects in the source vector
// and vice versa.
TEST_F(HybridAutomatonContinuousStateTest, Mutability) {
  (*mutable_vector_)[4] = 42;
  EXPECT_EQ(42, (*basic_vector_)[4]);
  EXPECT_EQ(42, (*mutable_vector_)[4]);

  (*basic_vector_)[5] = 555;
  EXPECT_EQ(555, (*basic_vector_)[5]);
  EXPECT_EQ(555, (*mutable_vector_)[5]);
}

// Tests the ContinuousState accessors and mutators using the a vector created
// using the HybridAutomatonContinuousState constructor.
TEST_F(HybridAutomatonContinuousStateTest, ContinuousState) {
  std::unique_ptr<VectorBase<double>> vector =
      BasicVector<double>::Make({1., 1., 2., 3., 5., 8.});
  std::unique_ptr<HybridAutomatonContinuousState<double>> hybrid_cs =
      make_unique<HybridAutomatonContinuousState<double>>(vector.get(), kSize,
                                                          0, 0);

  EXPECT_EQ(kSize, hybrid_cs->size());
  EXPECT_EQ(kSize, hybrid_cs->get_generalized_position().size());
  EXPECT_EQ(8, hybrid_cs->get_generalized_position().GetAtIndex(5));

  hybrid_cs->get_mutable_generalized_position()->SetAtIndex(0, 21);
  EXPECT_EQ(21, hybrid_cs->get_vector()[0]);
  EXPECT_EQ(21, (*vector)[0]);
}

}  // namespace
}  // namespace internal
}  // namespace systems
}  // namespace drake
