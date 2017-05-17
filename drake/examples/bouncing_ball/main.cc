#include "test.h"
#include <iostream>
#include <memory>

int main(int argc, char **argv){
  typedef typename TestBuilder<double>::ModalSubsystem ModalSubsystem;
  std::unique_ptr<BouncingBall<double>> dut_;
  dut_ = std::make_unique<BouncingBall<double>>();
  //dut_.reset(new BouncingBall<double>);
  std::cout << "Hello, world!\n";
  return 0;
}
