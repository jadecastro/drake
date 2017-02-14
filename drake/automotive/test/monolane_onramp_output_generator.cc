#include "drake/automotive/monolane_onramp_merge.h"

#include <cmath>
#include <fstream>
#include <memory>

#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/utility/generate_urdf.h"

#include "gtest/gtest.h"
#include "spruce.hh"

namespace drake {
namespace automotive {
namespace {

class MonolaneOnrampMergeTest : public ::testing::Test {
 public:
  MonolaneOnrampMergeTest() {}

  void SetUp() {
    directory_.setAsTemp();
    directory_.append("GenerateUrdfTest");

    dut_.reset(new MonolaneOnrampMerge);
    rg_ = dut_->BuildOnramp();
  }

 protected:
  std::unique_ptr<MonolaneOnrampMerge> dut_;
  std::unique_ptr<const maliput::api::RoadGeometry> rg_;

  const std::string kJunkBasename{"onramp"};
  spruce::path directory_;
};

TEST_F(MonolaneOnrampMergeTest, OutputFile) {
  EXPECT_NE(nullptr, rg_.get());
  GenerateUrdfFile(rg_.get(), directory_.getStr(), kJunkBasename,
                   maliput::utility::ObjFeatures());

  std::cerr << " directory_: " << directory_.getStr() << std::endl;
  spruce::path expected_urdf(directory_);
  expected_urdf.append(kJunkBasename + ".urdf");
  EXPECT_TRUE(expected_urdf.isFile());

  spruce::path expected_obj(directory_);
  expected_obj.append(kJunkBasename + ".obj");
  EXPECT_TRUE(expected_obj.isFile());

  spruce::path expected_mtl(directory_);
  expected_mtl.append(kJunkBasename + ".mtl");
  EXPECT_TRUE(expected_mtl.isFile());

  // Quick regression test on the URDF, which is mostly static content.
  std::string actual_urdf_contents;
  {
    std::ifstream is(expected_urdf.getStr());
    std::stringstream ss;
    ASSERT_TRUE(is.is_open());
    while (true) {
      char c = is.get();
      if (is.eof()) { break; }
      ss << c;
    }
    actual_urdf_contents = ss.str();
  }

  //EXPECT_TRUE(spruce::file::remove(expected_urdf));
  //EXPECT_TRUE(spruce::file::remove(expected_obj));
  //EXPECT_TRUE(spruce::file::remove(expected_mtl));
}

}  // namespace
}  // namespace automotive
}  // namespace drake
