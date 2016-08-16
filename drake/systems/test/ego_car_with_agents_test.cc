#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/common/drake_path.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

using drake::parsers::ModelInstanceIdTable;

class EgoCarWithAgentsTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tree.reset(new RigidBodyTree());

    // Defines four rigid bodies.
    r1b1 = new RigidBody();
    r1b1->set_model_name("ego");
    r1b1->set_name("body1");

    r2b1 = new RigidBody();
    r2b1->set_model_name("agent1");
    r2b1->set_name("body1");

    r3b1 = new RigidBody();
    r3b1->set_model_name("agent2");
    r3b1->set_name("body1");

    r4b1 = new RigidBody();
    r4b1->set_model_name("agent3");
    r4b1->set_name("body1");
  }

 public:
  // TODO(amcastro-tri): A stack object here (preferable to a pointer)
  // generates build issues on Windows platforms. See git-hub issue #1854.
  std::unique_ptr<RigidBodyTree> tree;
  // TODO(amcastro-tri): these pointers will be replaced by Sherm's
  // unique_ptr_reference's.
  RigidBody* r1b1{};
  RigidBody* r2b1{};
  RigidBody* r3b1{};
  RigidBody* r4b1{};
};

TEST_F(EgoCarWithAgentsTest, TestAddCars) {
  // Adds rigid bodies r1b1 and r2b1 to the rigid body tree and verify they can
  // be found.

  // RigidBodyTree takes ownership of these bodies.
  // User still has access to these bodies through the raw pointers.
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r1b1));
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r2b1));

  EXPECT_TRUE(tree->FindBody("body1", "ego") != nullptr);
  EXPECT_TRUE(tree->FindBody("body1", "agent1") != nullptr);

  // Adds floating joints that connect r1b1 and r2b1 to the rigid body tree's
  // world at zero offset.
  tree->AddFloatingJoint(DrakeJoint::QUATERNION,
                         {r1b1->get_body_index(), r2b1->get_body_index()});

  // Verfies that the two rigid bodies are located in the correct place.
  const DrakeJoint& jointR1B1 = tree->FindBody("body1", "ego")->getJoint();
  EXPECT_TRUE(jointR1B1.isFloating());
  EXPECT_TRUE(jointR1B1.getTransformToParentBody().matrix() ==
              Eigen::Isometry3d::Identity().matrix());

  const DrakeJoint& jointR2B1 = tree->FindBody("body1", "agent1")->getJoint();
  EXPECT_TRUE(jointR2B1.isFloating());
  EXPECT_TRUE(jointR2B1.getTransformToParentBody().matrix() ==
              Eigen::Isometry3d::Identity().matrix());
}

// Ensures RigidBodyTree::doKinemantics(q, v, bool) is explicitly instantiated
// with vector block input parameters. For more information, see:
// https://github.com/RobotLocomotion/drake/issues/2634.
TEST_F(EgoCarWithAgentsTest, TestDoKinematicsWithVectorBlocks) {
  std::string file_name =
      GetDrakePath() +
      "/systems/plants/test/rigid_body_tree/two_dof_robot.urdf";
  drake::parsers::urdf::AddModelInstanceFromURDF(file_name, tree.get());

  VectorX<double> q;
  VectorX<double> v;
  q.resize(tree->number_of_positions());
  v.resize(tree->number_of_velocities());
  q.setZero();
  v.setZero();

  Eigen::VectorBlock<VectorX<double>> q_block = q.head(q.size());
  Eigen::VectorBlock<VectorX<double>> v_block = v.head(v.size());

  KinematicsCache<double> cache = tree->doKinematics(q_block, v_block);
  EXPECT_TRUE(cache.hasV());
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
