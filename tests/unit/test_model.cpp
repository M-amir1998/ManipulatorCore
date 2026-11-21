#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/math/linalg.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"
#include "manipulatorcore/model/dh.hpp"
#include "manipulatorcore/model/urdf.hpp"
#include "robot_fixtures.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace mc {
namespace {

TEST(Joint, MotionAndSubspace) {
  Joint revolute;
  revolute.axis = Vec3::UnitZ();
  EXPECT_MATRIX_NEAR(revolute.motion(0.5).rotation(), rotZ(0.5), 1e-15);
  Vec6 S_r;
  S_r << 0, 0, 1, 0, 0, 0;
  EXPECT_MATRIX_NEAR(revolute.motionSubspace(), S_r, 0.0);

  Joint prismatic;
  prismatic.type = JointType::Prismatic;
  prismatic.axis = Vec3::UnitX();
  EXPECT_MATRIX_NEAR(prismatic.motion(0.25).translation(), Vec3(0.25, 0.0, 0.0), 0.0);
  Vec6 S_p;
  S_p << 0, 0, 0, 1, 0, 0;
  EXPECT_MATRIX_NEAR(prismatic.motionSubspace(), S_p, 0.0);
}

TEST(Joint, ValidationRejectsBadInput) {
  Joint j;
  j.axis = Vec3(1.0, 1.0, 0.0);  // not unit
  EXPECT_THROW(j.validate(), InvalidArgument);
  j.axis = Vec3::UnitZ();
  j.limits.lower = 1.0;
  j.limits.upper = -1.0;
  EXPECT_THROW(j.validate(), InvalidArgument);
  j.limits = {};
  j.limits.velocity = 0.0;
  EXPECT_THROW(j.validate(), InvalidArgument);
  j.limits = {};
  j.damping = -1.0;
  EXPECT_THROW(j.validate(), InvalidArgument);
}

TEST(Inertia, SpatialMatrixIsSymmetricPositiveDefinite) {
  const Inertia I = Inertia::solidCylinder(2.0, 0.05, Vec3(0.1, 0.0, 0.0), Vec3(0.5, 0.2, 0.1));
  const Mat6 G = I.spatialMatrix();
  EXPECT_TRUE(isSymmetric(G, 1e-14));
  EXPECT_TRUE(isPositiveDefinite(G));
  // Kinetic energy of a pure translation is 1/2 m |v|^2.
  Vec6 V = Vec6::Zero();
  V.tail<3>() = Vec3(1.0, -2.0, 0.5);
  EXPECT_NEAR(0.5 * V.dot(G * V), 0.5 * 2.0 * V.tail<3>().squaredNorm(), 1e-13);
}

TEST(Inertia, KineticEnergyMatchesCentroidalFormula) {
  // KE = 1/2 m |v_c|^2 + 1/2 w^T I_c w with v_c = v + w x c.
  Random rng(3);
  const Inertia I = Inertia::solidBox(3.0, Vec3(0.2, 0.3, 0.4), Vec3(0.1, -0.2, 0.3));
  for (int k = 0; k < 20; ++k) {
    const Vec6 V = test::randomTwist(rng);
    const Vec3 w = V.head<3>();
    const Vec3 vc = V.tail<3>() + w.cross(I.com);
    const double expected = 0.5 * I.mass * vc.squaredNorm() + 0.5 * w.dot(I.inertia_com * w);
    EXPECT_NEAR(0.5 * V.dot(I.spatialMatrix() * V), expected, 1e-12);
  }
}

TEST(Inertia, CylinderMomentsAreCorrect) {
  const double m = 2.0;
  const double r = 0.1;
  const double L = 1.0;
  const Inertia I = Inertia::solidCylinder(m, r, Vec3::Zero(), Vec3(0.0, 0.0, L));
  EXPECT_NEAR(I.inertia_com(2, 2), 0.5 * m * r * r, 1e-15);
  EXPECT_NEAR(I.inertia_com(0, 0), m * (3 * r * r + L * L) / 12.0, 1e-15);
  EXPECT_MATRIX_NEAR(I.com, Vec3(0.0, 0.0, 0.5), 0.0);
}

TEST(Inertia, CombineMatchesParallelAxisTheorem) {
  // Two equal point masses at +-d along x: I_yy = I_zz = 2 m d^2 about the common COM.
  const Inertia a = Inertia::pointMass(1.0, Vec3(0.5, 0.0, 0.0));
  const Inertia b = Inertia::pointMass(1.0, Vec3(-0.5, 0.0, 0.0));
  const Inertia c = Inertia::combine(a, b);
  EXPECT_NEAR(c.mass, 2.0, 0.0);
  EXPECT_MATRIX_NEAR(c.com, Vec3::Zero(), 1e-15);
  EXPECT_NEAR(c.inertia_com(1, 1), 0.5, 1e-15);
  EXPECT_NEAR(c.inertia_com(2, 2), 0.5, 1e-15);
  EXPECT_NEAR(c.inertia_com(0, 0), 0.0, 1e-15);
}

TEST(Inertia, TransformedPreservesSpatialInertia) {
  // G_a = Ad_{T_body_a}^T G_body Ad_{T_body_a}
  Random rng(5);
  const Inertia I = Inertia::solidBox(1.5, Vec3(0.1, 0.4, 0.2), Vec3(0.2, 0.1, -0.1));
  const SE3 T_a_body = test::randomPose(rng);
  const Mat6 Ad = T_a_body.inverse().adjoint();
  EXPECT_MATRIX_NEAR(I.transformed(T_a_body).spatialMatrix(), (Ad.transpose() * I.spatialMatrix() * Ad).eval(), 1e-12);
}

TEST(Inertia, ValidationRejectsUnphysicalParameters) {
  Inertia I = Inertia::solidSphere(1.0, 0.1, Vec3::Zero());
  I.mass = -1.0;
  EXPECT_THROW(I.validate(), InvalidArgument);
  Inertia triangle;
  triangle.mass = 1.0;
  triangle.inertia_com = Vec3(1.0, 1.0, 3.0).asDiagonal();  // violates Ixx + Iyy >= Izz
  EXPECT_THROW(triangle.validate(), InvalidArgument);
}

TEST(RobotModel, LimitsAndClamping) {
  const RobotModel model = makeRedundant7R();
  EXPECT_EQ(model.dof(), 7);
  VecX q = model.upperLimits();
  EXPECT_TRUE(model.withinLimits(q));
  q(2) += 0.1;
  EXPECT_FALSE(model.withinLimits(q));
  EXPECT_TRUE(model.withinLimits(q, 0.2));
  EXPECT_MATRIX_NEAR(model.clampToLimits(q), model.upperLimits(), 0.0);
  EXPECT_FALSE(model.withinLimits(VecX::Zero(3)));
  EXPECT_THROW(model.checkConfiguration(VecX::Zero(6)), InvalidArgument);
  VecX bad = VecX::Zero(7);
  bad(0) = std::numeric_limits<double>::infinity();
  EXPECT_THROW(model.checkConfiguration(bad), InvalidArgument);
}

TEST(RobotModel, BuiltinRobotsAreValid) {
  for (const RobotModel& model : test::allBuiltinRobots()) {
    EXPECT_GT(model.totalMass(), 0.0) << model.name();
    for (const Body& b : model.bodies()) {
      EXPECT_GT(b.inertia.mass, 0.0) << model.name() << "/" << b.name;
      EXPECT_FALSE(b.collision.empty()) << model.name() << "/" << b.name;
    }
  }
  EXPECT_TRUE(makeIndustrial6R().withinLimits(industrial6RHome()));
  EXPECT_TRUE(makeRedundant7R().withinLimits(redundant7RHome()));
  EXPECT_EQ(makeRedundant7R().bodyIndex("elbow_link"), 3);
  EXPECT_EQ(makeRedundant7R().bodyIndex("missing"), -1);
}

TEST(RobotModel, RejectsEmptyModels) { EXPECT_THROW(RobotModel("empty", {}), InvalidArgument); }

TEST(DH, StandardTransformMatchesDefinition) {
  DHRow row;
  row.a = 0.3;
  row.alpha = 0.5;
  row.d = 0.2;
  row.theta = 0.1;
  const double q = 0.7;
  const SE3 T = dhTransform(DHConvention::Standard, row, q);
  const SE3 expected = SE3::fromRotation(rotZ(0.8)) * SE3::fromTranslation(Vec3(0, 0, 0.2)) *
                       SE3::fromTranslation(Vec3(0.3, 0, 0)) * SE3::fromRotation(rotX(0.5));
  EXPECT_POSE_NEAR(T, expected, 1e-15);
}

TEST(DH, ModifiedTransformMatchesDefinition) {
  DHRow row;
  row.a = 0.3;
  row.alpha = 0.5;
  row.d = 0.2;
  row.theta = 0.1;
  row.type = JointType::Prismatic;
  const double q = 0.4;  // prismatic: d = 0.6
  const SE3 T = dhTransform(DHConvention::Modified, row, q);
  const SE3 expected = SE3::fromRotation(rotX(0.5)) * SE3::fromTranslation(Vec3(0.3, 0, 0)) *
                       SE3::fromRotation(rotZ(0.1)) * SE3::fromTranslation(Vec3(0, 0, 0.6));
  EXPECT_POSE_NEAR(T, expected, 1e-15);
}

DHTable mixedTable(DHConvention convention) {
  DHTable t;
  t.convention = convention;
  t.rows = {
      {0.1, -kHalfPi, 0.4, 0.2, JointType::Revolute, {}, "a"},
      {0.5, 0.3, 0.1, -0.4, JointType::Revolute, {}, "b"},
      {0.0, kHalfPi, 0.2, 0.3, JointType::Prismatic, {-0.5, 0.5, 1.0, 5.0, 100.0}, "c"},
      {0.2, -0.7, 0.3, 0.0, JointType::Revolute, {}, "d"},
  };
  t.tool = SE3(rotY(0.3), Vec3(0.0, 0.05, 0.1));
  return t;
}

class DHEquivalence : public ::testing::TestWithParam<DHConvention> {};

TEST_P(DHEquivalence, ChainModelReproducesDirectDHProduct) {
  const DHTable table = mixedTable(GetParam());
  const RobotModel model = robotFromDH(table, "dh");
  Random rng(9);
  for (int k = 0; k < 50; ++k) {
    const VecX q = test::randomConfiguration(model, rng);
    const std::vector<SE3> frames = dhFrames(table, q);
    const KinematicState state = forwardKinematics(model, q);
    for (int i = 0; i < model.dof(); ++i) {
      EXPECT_POSE_NEAR(state.T_base_body[static_cast<std::size_t>(i)] * dhFrameInBody(table, i),
                       frames[static_cast<std::size_t>(i)], 1e-12);
    }
    EXPECT_POSE_NEAR(state.T_base_ee, dhForwardKinematics(table, q), 1e-12);
  }
}

INSTANTIATE_TEST_SUITE_P(Conventions, DHEquivalence, ::testing::Values(DHConvention::Standard, DHConvention::Modified));

TEST(DH, Industrial6RMatchesItsDHTable) {
  const RobotModel model = makeIndustrial6R();
  const DHTable table = industrial6RDHTable();
  Random rng(11);
  for (int k = 0; k < 50; ++k) {
    const VecX q = test::randomConfiguration(model, rng);
    EXPECT_POSE_NEAR(endEffectorPose(model, q), dhForwardKinematics(table, q), 1e-12);
  }
}

TEST(URDF, RoundTripPreservesKinematicsAndDynamics) {
  for (const RobotModel& original : test::allBuiltinRobots()) {
    const std::string xml = toURDF(original);
    const RobotModel parsed = fromURDF(xml, {.gravity = original.gravity()});
    ASSERT_EQ(parsed.dof(), original.dof()) << original.name();
    EXPECT_EQ(parsed.name(), original.name());
    EXPECT_POSE_NEAR(parsed.toolTransform(), original.toolTransform(), 1e-12);
    for (int i = 0; i < original.dof(); ++i) {
      const Body& a = original.body(i);
      const Body& b = parsed.body(i);
      EXPECT_EQ(a.name, b.name);
      EXPECT_EQ(a.joint.name, b.joint.name);
      EXPECT_POSE_NEAR(a.joint.origin, b.joint.origin, 1e-12);
      EXPECT_MATRIX_NEAR(a.joint.axis, b.joint.axis, 1e-15);
      EXPECT_DOUBLE_EQ(a.joint.limits.lower, b.joint.limits.lower);
      EXPECT_DOUBLE_EQ(a.joint.limits.upper, b.joint.limits.upper);
      EXPECT_DOUBLE_EQ(a.joint.limits.velocity, b.joint.limits.velocity);
      EXPECT_DOUBLE_EQ(a.joint.limits.acceleration, b.joint.limits.acceleration);
      EXPECT_DOUBLE_EQ(a.joint.limits.effort, b.joint.limits.effort);
      EXPECT_DOUBLE_EQ(a.joint.damping, b.joint.damping);
      EXPECT_DOUBLE_EQ(a.joint.armature, b.joint.armature);
      EXPECT_NEAR(a.inertia.mass, b.inertia.mass, 1e-14);
      EXPECT_MATRIX_NEAR(a.inertia.com, b.inertia.com, 1e-14);
      EXPECT_MATRIX_NEAR(a.inertia.inertia_com, b.inertia.inertia_com, 1e-14);
      EXPECT_EQ(a.collision.size(), b.collision.size());
    }
    EXPECT_EQ(parsed.baseCollision().size(), original.baseCollision().size());
    Random rng(13);
    const VecX q = test::randomConfiguration(original, rng);
    EXPECT_POSE_NEAR(endEffectorPose(parsed, q), endEffectorPose(original, q), 1e-12);
  }
}

TEST(URDF, MergesFixedJointsAndSelectsChain) {
  const std::string xml = R"(<?xml version="1.0"?>
<!-- a small tree: world -> base (fixed) -> l1 (revolute) -> l2 (fixed) -> l3 (prismatic) -> tip (fixed) -->
<robot name="tree">
  <link name="world"/>
  <link name="base"><collision><geometry><box size="0.2 0.2 0.1"/></geometry></collision></link>
  <link name="l1"><inertial><origin xyz="0 0 0.1"/><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial></link>
  <link name="l2"><inertial><origin xyz="0.1 0 0"/><mass value="2"/><inertia ixx="0.02" iyy="0.02" izz="0.02"/></inertial></link>
  <link name="l3"/>
  <link name="tip"/>
  <link name="camera"/>
  <joint name="fix0" type="fixed"><parent link="world"/><child link="base"/><origin xyz="0 0 0.5"/></joint>
  <joint name="j1" type="continuous"><parent link="base"/><child link="l1"/><axis xyz="0 0 1"/></joint>
  <joint name="fix1" type="fixed"><parent link="l1"/><child link="l2"/><origin xyz="0 0 0.3" rpy="0 0 1.5707963267948966"/></joint>
  <joint name="j2" type="prismatic"><parent link="l2"/><child link="l3"/><axis xyz="1 0 0"/>
    <limit lower="-0.1" upper="0.2" effort="50" velocity="0.5"/></joint>
  <joint name="fix2" type="fixed"><parent link="l3"/><child link="tip"/><origin xyz="0.05 0 0"/></joint>
  <joint name="cam" type="fixed"><parent link="l1"/><child link="camera"/></joint>
</robot>)";
  const RobotModel model = fromURDF(xml, {.tip_link = "tip"});
  ASSERT_EQ(model.dof(), 2);
  EXPECT_EQ(model.body(0).name, "l1");
  EXPECT_NEAR(model.body(0).inertia.mass, 3.0, 1e-15);  // l1 + l2 merged
  EXPECT_NEAR(model.joint(0).limits.lower, -kPi, 1e-15);
  EXPECT_MATRIX_NEAR(model.joint(0).origin.translation(), Vec3(0, 0, 0.5), 1e-15);
  EXPECT_MATRIX_NEAR(model.joint(1).origin.translation(), Vec3(0, 0, 0.3), 1e-15);
  EXPECT_MATRIX_NEAR(model.toolTransform().translation(), Vec3(0.05, 0, 0), 1e-15);
  EXPECT_EQ(model.baseCollision().size(), 1U);
  // Without a tip link the tree branches at l1.
  EXPECT_THROW((void)fromURDF(xml), InvalidArgument);
}

TEST(URDF, RejectsMalformedDocuments) {
  EXPECT_THROW((void)fromURDF("<robot name='x'><link name='a'></robot>"), InvalidArgument);
  EXPECT_THROW((void)fromURDF("<notrobot/>"), InvalidArgument);
  EXPECT_THROW((void)fromURDF("<robot name='x'><link name='a'/></robot>"), InvalidArgument);  // no joints
  EXPECT_THROW((void)fromURDF("<robot><link name='a'/><link name='b'/><joint name='j' type='floating'>"
                              "<parent link='a'/><child link='b'/></joint></robot>"),
               InvalidArgument);
  EXPECT_THROW((void)fromURDF("<robot><link name='a'/><link name='b'/><joint name='j' type='revolute'>"
                              "<parent link='a'/><child link='b'/><origin xyz='1 2'/></joint></robot>"),
               InvalidArgument);
}

}  // namespace
}  // namespace mc
