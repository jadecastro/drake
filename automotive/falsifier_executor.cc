#include <gflags/gflags.h>

#include "drake/automotive/falsifier.h"

namespace drake {
namespace automotive {
namespace {

/// A simple executable to run the falsifier.
int DoMain(void) {
  auto falsifier = std::make_unique<Falsifier>();
  falsifier->Run();

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::automotive::DoMain();
}
