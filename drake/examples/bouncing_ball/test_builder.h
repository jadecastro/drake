#pragma once

template <typename T>
class TestBuilder {
 public:
  TestBuilder() {}
  virtual ~TestBuilder() {}

  typedef int ModalSubsystem;

  const ModalSubsystem* AddModalSubsystem() {
    ModalSubsystem modal_subsystem = 7;
    return &modal_subsystem;
  }
};
