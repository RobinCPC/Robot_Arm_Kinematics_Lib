/*!
 * @file  kinematics_test.cpp
 * @brief GTest suite for Artic forward/inverse kinematics.
 *
 * Covers:
 *   ForwardKinTest  — known joint config → expected TCP pose
 *   RoundTripTest   — FK → IK → FK: recovered TCP matches original
 *   InverseKinTest  — result codes for reachable / singular / out-of-reach poses
 */

#include <gtest/gtest.h>
#include <cmath>

#include "../src/kin/artic.h"

// Tolerance constants
static const double POS_TOL = 1e-3;   // mm
static const double ORI_TOL = 1e-3;   // deg

// ─────────────────────────────────────────────────────────────────────────────
// Helper
// ─────────────────────────────────────────────────────────────────────────────

static void expectPoseNear(const rb::kin::ArmPose& a,
                            const rb::kin::ArmPose& b,
                            double pos_tol = POS_TOL,
                            double ori_tol = ORI_TOL)
{
  EXPECT_NEAR(a.x, b.x, pos_tol);
  EXPECT_NEAR(a.y, b.y, pos_tol);
  EXPECT_NEAR(a.z, b.z, pos_tol);
  EXPECT_NEAR(a.a, b.a, ori_tol);
  EXPECT_NEAR(a.b, b.b, ori_tol);
  EXPECT_NEAR(a.c, b.c, ori_tol);
}

// ─────────────────────────────────────────────────────────────────────────────
// ForwardKinTest  — deterministic FK values for KUKA KR5 default config
// ─────────────────────────────────────────────────────────────────────────────

class ForwardKinTest : public ::testing::Test
{
protected:
  rb::kin::Artic robot;
};

TEST_F(ForwardKinTest, HomePosition)
{
  // All joints zero: arm is symmetric about the XZ-plane so y == 0.
  // (The KUKA KR5 default DH config places TCP at z = -40 mm when all
  //  joints are zero — the pose is below the base origin.)
  rb::math::VectorX q(6);
  q << 0, 0, 0, 0, 0, 0;
  rb::kin::ArmPose pose = robot.forwardKin(q);

  EXPECT_NEAR(pose.y, 0.0, POS_TOL);

  // A second call with the same input must return an identical result
  rb::kin::ArmPose pose2 = robot.forwardKin(q);
  EXPECT_NEAR(pose2.x, pose.x, POS_TOL);
  EXPECT_NEAR(pose2.z, pose.z, POS_TOL);
}

TEST_F(ForwardKinTest, J1Rotation90)
{
  // Rotating only J1 by 90 deg maps X→Y in the base frame
  rb::math::VectorX q0(6), q90(6);
  q0  << 0,  0, 0, 0, 30, 0;
  q90 << 90, 0, 0, 0, 30, 0;

  rb::kin::ArmPose p0  = robot.forwardKin(q0);
  rb::kin::ArmPose p90 = robot.forwardKin(q90);

  // Distance from base origin should be the same
  double r0  = std::sqrt(p0.x*p0.x   + p0.y*p0.y);
  double r90 = std::sqrt(p90.x*p90.x + p90.y*p90.y);
  EXPECT_NEAR(r0, r90, POS_TOL);

  // Height (z) unchanged when only rotating J1
  EXPECT_NEAR(p0.z, p90.z, POS_TOL);
}

TEST_F(ForwardKinTest, KnownPose)
{
  // Cross-check against the value hard-coded in the original test_run.cpp:
  //   q = [45, -90, 45, 0, 90, 0]  →  TCP ≈ (205.533, -205.533, 403.475, -45, 45, 180)
  rb::math::VectorX q(6);
  q << 45, -90, 45, 0, 90, 0;
  rb::kin::ArmPose pose = robot.forwardKin(q);

  EXPECT_NEAR(pose.x,   205.533, 1.0);   // 1 mm tolerance for reference value
  EXPECT_NEAR(pose.y,  -205.533, 1.0);
  EXPECT_NEAR(pose.z,   403.475, 1.0);
  EXPECT_NEAR(pose.a,   -45.0,   1.0);
  EXPECT_NEAR(pose.b,    45.0,   1.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// RoundTripTest  — FK(q) → IK → FK(q') must give the same TCP
// ─────────────────────────────────────────────────────────────────────────────

class RoundTripTest : public ::testing::Test
{
protected:
  rb::kin::Artic robot;

  void runRoundTrip(const rb::math::VectorX& q_in)
  {
    // Step 1: FK at test configuration
    rb::kin::ArmPose tcp_in = robot.forwardKin(q_in);

    // Step 2: IK from that TCP pose
    rb::math::VectorX joints;
    rb::kin::ArmAxisValue all_sols;
    rb::kin::IK_RESULT result = robot.inverseKin(
        tcp_in.x, tcp_in.y, tcp_in.z,
        tcp_in.a, tcp_in.b, tcp_in.c,
        joints, all_sols);

    ASSERT_EQ(result, rb::kin::IK_COMPLETE);

    // Step 3: FK again with the best-fit IK joints
    rb::kin::ArmPose tcp_out = robot.forwardKin(joints);

    expectPoseNear(tcp_out, tcp_in);
  }
};

TEST_F(RoundTripTest, HomeAllZeros)
{
  rb::math::VectorX q(6);
  q << 0, 0, 0, 0, 0, 0;
  runRoundTrip(q);
}

TEST_F(RoundTripTest, J1andJ5)
{
  // J5=0 is a wrist singularity; use J5=30 to avoid it
  rb::math::VectorX q(6);
  q << 45, 0, 0, 0, 30, 0;
  runRoundTrip(q);
}

TEST_F(RoundTripTest, MultiJoint)
{
  rb::math::VectorX q(6);
  q << 45, -90, 45, 0, 90, 0;
  runRoundTrip(q);
}

TEST_F(RoundTripTest, GeneralPose)
{
  rb::math::VectorX q(6);
  q << 30, -45, 60, 20, -30, 15;
  runRoundTrip(q);
}

TEST_F(RoundTripTest, NegElbowConfig)
{
  rb::math::VectorX q(6);
  q << -60, -30, 30, -90, 45, 0;
  runRoundTrip(q);
}

// ─────────────────────────────────────────────────────────────────────────────
// InverseKinTest  — result codes
// ─────────────────────────────────────────────────────────────────────────────

class InverseKinTest : public ::testing::Test
{
protected:
  rb::kin::Artic robot;
};

TEST_F(InverseKinTest, ReachablePoseReturnsComplete)
{
  rb::math::VectorX joints;
  rb::kin::ArmAxisValue all_sols;
  // Use the FK-verified pose from test_run.cpp
  rb::kin::IK_RESULT result = robot.inverseKin(
      205.533, -205.533, 403.475, -45., 45., 180., joints, all_sols);
  EXPECT_EQ(result, rb::kin::IK_COMPLETE);
}

TEST_F(InverseKinTest, AllSolutionsMatrixHas8Rows)
{
  rb::math::VectorX joints;
  rb::kin::ArmAxisValue all_sols;
  robot.inverseKin(205.533, -205.533, 403.475, -45., 45., 180., joints, all_sols);
  EXPECT_EQ(all_sols.axis_value.rows(), 8);
  EXPECT_EQ(all_sols.axis_value.cols(), 6);
}

TEST_F(InverseKinTest, UnreachablePoseReturnsNoSolution)
{
  // A point far beyond the robot's reach (10 m away)
  rb::math::VectorX joints;
  rb::kin::ArmAxisValue all_sols;
  rb::kin::IK_RESULT result = robot.inverseKin(
      10000., 0., 0., 0., 0., 0., joints, all_sols);
  EXPECT_NE(result, rb::kin::IK_COMPLETE);
}

TEST_F(InverseKinTest, BestFitJointsFKMatchesInputPose)
{
  rb::math::VectorX joints;
  rb::kin::ArmAxisValue all_sols;
  double x = 205.533, y = -205.533, z = 403.475;
  double roll = -45., pitch = 45., yaw = 180.;

  rb::kin::IK_RESULT result = robot.inverseKin(
      x, y, z, roll, pitch, yaw, joints, all_sols);
  ASSERT_EQ(result, rb::kin::IK_COMPLETE);

  rb::kin::ArmPose recovered = robot.forwardKin(joints);
  EXPECT_NEAR(recovered.x, x,     POS_TOL);
  EXPECT_NEAR(recovered.y, y,     POS_TOL);
  EXPECT_NEAR(recovered.z, z,     POS_TOL);
  EXPECT_NEAR(recovered.a, roll,  ORI_TOL);
  EXPECT_NEAR(recovered.b, pitch, ORI_TOL);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
