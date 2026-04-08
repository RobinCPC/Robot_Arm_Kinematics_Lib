/*!
 * @file  basic_fk_ik.cpp
 * @brief Minimal usage example for RAKL.
 *
 * Shows how to:
 *   1. Create a default KUKA KR5 robot arm
 *   2. Run forward kinematics (joints → TCP pose)
 *   3. Run inverse kinematics (TCP pose → joint solutions)
 *   4. Verify the round-trip by feeding IK joints back into FK
 *
 * Build (from repo root):
 *   mkdir build && cd build
 *   cmake .. -DCMAKE_BUILD_TYPE=Release
 *   make basic_fk_ik
 *   ./basic_fk_ik
 */

#include <iostream>
#include <memory>

#include "kin/artic.h"

int main()
{
  // ── 1. Create robot (default KUKA KR5 DH parameters) ──────────────────────
  std::unique_ptr<rb::kin::Artic> robot(new rb::kin::Artic());

  // ── 2. Forward kinematics ──────────────────────────────────────────────────
  rb::math::VectorX joints_in(6);
  joints_in << 45., -90., 45., 0., 90., 0.;   // degrees

  rb::kin::ArmPose tcp = robot->forwardKin(joints_in);

  std::cout << "=== Forward Kinematics ===\n"
            << "Input joints (deg):  "
            << joints_in.transpose() << "\n"
            << "TCP position  (mm):  "
            << "x=" << tcp.x << "  y=" << tcp.y << "  z=" << tcp.z << "\n"
            << "TCP orientation (deg): "
            << "roll=" << tcp.a << "  pitch=" << tcp.b << "  yaw=" << tcp.c
            << "\n\n";

  // ── 3. Inverse kinematics ──────────────────────────────────────────────────
  rb::math::VectorX    joints_out;
  rb::kin::ArmAxisValue all_solutions;

  rb::kin::IK_RESULT result = robot->inverseKin(
      tcp.x, tcp.y, tcp.z,
      tcp.a, tcp.b, tcp.c,
      joints_out, all_solutions);

  if (result != rb::kin::IK_COMPLETE)
  {
    std::cerr << "IK failed with code " << result << "\n";
    return 1;
  }

  std::cout << "=== Inverse Kinematics ===\n"
            << "Best-fit solution (deg):  "
            << joints_out.transpose() << "\n"
            << "Solutions found: ";
  for (int i = 0; i < 8; ++i)
    if (all_solutions.solution_check[i]) std::cout << i << " ";
  std::cout << "\n\n";

  // ── 4. Round-trip verification ─────────────────────────────────────────────
  rb::kin::ArmPose tcp_check = robot->forwardKin(joints_out);

  double pos_err = std::max({std::abs(tcp_check.x - tcp.x),
                              std::abs(tcp_check.y - tcp.y),
                              std::abs(tcp_check.z - tcp.z)});
  double ori_err = std::max({std::abs(tcp_check.a - tcp.a),
                              std::abs(tcp_check.b - tcp.b),
                              std::abs(tcp_check.c - tcp.c)});

  std::cout << "=== Round-trip check ===\n"
            << "Max position error (mm):    " << pos_err << "\n"
            << "Max orientation error (deg): " << ori_err << "\n"
            << (pos_err < 1e-3 && ori_err < 1e-3 ? "PASS" : "FAIL") << "\n";

  return 0;
}
