#pragma once

#include "test_builder.h"

template <typename T>
class BouncingBall : public TestBuilder<T> {
 public:
  typedef typename TestBuilder<T>::ModalSubsystem ModalSubsystem;

  /// Constructor for the BouncingBall system.
  BouncingBall() {
  //explicit BouncingBall(ModalSubsystem<T>& mss) : ball_subsystem_(mss) {
    TestBuilder<T> builder;
    ball_subsystem_ = builder.AddModalSubsystem();
  };

  ~BouncingBall() {}

 private:
  //typedef typename systems::HybridAutomaton<T>::ModalSubsystem ModalSubsystem;

  const ModalSubsystem* ball_subsystem_;
};
