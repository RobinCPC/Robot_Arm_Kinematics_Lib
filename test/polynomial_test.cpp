/*!
 * @file  polynomial_test.cpp
 * @brief GTest suite for rb::math::Polynomial (quintic trajectory).
 *
 * Verifies that coeffQuintic() satisfies the 6 boundary conditions:
 *   pos/vel/accel at t=0  and  pos/vel/accel at t=T
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "../src/math/polynomial.h"
#include "../src/math/unit.h"

static const double POLY_TOL = 1e-9;

class PolynomialTest : public ::testing::Test
{
protected:
  // boundary conditions: {pos, vel, accel}
  std::vector<double> start = {0.0,  0.0, 0.0};
  std::vector<double> end   = {1.0,  0.0, 0.0};
  double T = 2.0;

  rb::math::Polynomial traj{5};

  void SetUp() override
  {
    traj.coeffQuintic(start, end, T);
  }
};

TEST_F(PolynomialTest, PositionAtT0)
{
  EXPECT_NEAR(traj.getPosition(0.0), start[0], POLY_TOL);
}

TEST_F(PolynomialTest, PositionAtTf)
{
  EXPECT_NEAR(traj.getPosition(T), end[0], POLY_TOL);
}

TEST_F(PolynomialTest, VelocityAtT0)
{
  EXPECT_NEAR(traj.getVelocity(0.0), start[1], POLY_TOL);
}

TEST_F(PolynomialTest, VelocityAtTf)
{
  EXPECT_NEAR(traj.getVelocity(T), end[1], POLY_TOL);
}

TEST_F(PolynomialTest, AccelAtT0)
{
  EXPECT_NEAR(traj.getAcceleration(0.0), start[2], POLY_TOL);
}

TEST_F(PolynomialTest, AccelAtTf)
{
  EXPECT_NEAR(traj.getAcceleration(T), end[2], POLY_TOL);
}

TEST_F(PolynomialTest, PositionMonotonicallyIncreasing)
{
  // With zero-velocity endpoints and start < end, position should
  // increase strictly between t=0 and t=T
  double prev = traj.getPosition(0.0);
  for (int i = 1; i <= 10; ++i)
  {
    double pos = traj.getPosition(T * i / 10.0);
    EXPECT_GE(pos, prev);
    prev = pos;
  }
}

TEST_F(PolynomialTest, NonZeroEndVelocity)
{
  // With non-zero end velocity the boundary condition should still hold
  rb::math::Polynomial traj2(5);
  std::vector<double> s2 = {100.0, 0.0, 0.0};
  std::vector<double> e2 = {200.0, 50.0, 0.0};
  double T2 = 3.0;
  traj2.coeffQuintic(s2, e2, T2);

  EXPECT_NEAR(traj2.getPosition(0.0),   s2[0], POLY_TOL);
  EXPECT_NEAR(traj2.getPosition(T2),    e2[0], POLY_TOL);
  EXPECT_NEAR(traj2.getVelocity(0.0),   s2[1], POLY_TOL);
  EXPECT_NEAR(traj2.getVelocity(T2),    e2[1], POLY_TOL);
  EXPECT_NEAR(traj2.getAcceleration(0.0), s2[2], POLY_TOL);
  EXPECT_NEAR(traj2.getAcceleration(T2),  e2[2], POLY_TOL);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
