#include <iostream>
#include <memory>
#include <chrono>
#include <algorithm>
#include <cmath>

#include "../src/kin/link.h"
#include "../src/kin/artic.h"
#include "../src/math/polynomial.h"

using namespace rb::kin;

struct Timer
{
  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  std::chrono::duration<double> time_step;
  Timer()
  {
    start = std::chrono::high_resolution_clock::now();
  }

  ~Timer()
  {
    end = std::chrono::high_resolution_clock::now();
    time_step = end - start;
    double ns = time_step.count() * 1.0e+6f;
    std::cout << "Timer took " << ns << " ns\n";
  }
};

// ===== Round-trip FK/IK test helpers =====

struct RoundTripResult
{
  bool   ik_complete;
  bool   pose_match;
  double max_pos_err;   // mm
  double max_ori_err;   // deg
};

static RoundTripResult checkRoundTrip(rb::kin::Artic& robot,
                                       const rb::math::VectorX& q_in,
                                       double tol_pos, double tol_ori)
{
  RoundTripResult r = {false, false, 0., 0.};

  // Step 1: FK at the test configuration
  rb::kin::ArmPose tcp = robot.forwardKin(q_in);

  // Step 2: IK from that TCP pose
  rb::math::VectorX joints;
  rb::kin::ArmAxisValue all_sols;
  rb::kin::IK_RESULT res = robot.inverseKin(
      tcp.x, tcp.y, tcp.z, tcp.a, tcp.b, tcp.c, joints, all_sols);

  r.ik_complete = (res == rb::kin::IK_COMPLETE);
  if (!r.ik_complete)
    return r;

  // Step 3: FK again with the best-fit IK joints
  rb::kin::ArmPose tcp2 = robot.forwardKin(joints);

  // Compare the two TCP poses
  r.max_pos_err = std::max({std::fabs(tcp2.x - tcp.x),
                             std::fabs(tcp2.y - tcp.y),
                             std::fabs(tcp2.z - tcp.z)});
  r.max_ori_err = std::max({std::fabs(tcp2.a - tcp.a),
                             std::fabs(tcp2.b - tcp.b),
                             std::fabs(tcp2.c - tcp.c)});
  r.pose_match  = (r.max_pos_err <= tol_pos && r.max_ori_err <= tol_ori);
  return r;
}

static int runRoundTripTests(rb::kin::Artic& robot)
{
  const double TOL_POS = 0.001;  // mm
  const double TOL_ORI = 0.001;  // deg

  struct TestCase { const char* name; rb::math::VectorX q; };
  std::vector<TestCase> cases;

  rb::math::VectorX q(6);

  q << 0, 0, 0, 0, 0, 0;
  cases.push_back({"Home (all zeros)", q});

  q << 45, 0, 0, 0, 30, 0;  // J5=0 is a wrist singularity; use J5=30 to avoid it
  cases.push_back({"J1+J5 (avoid singularity)", q});

  q << 45, -90, 45, 0, 90, 0;
  cases.push_back({"Multi-joint (existing)", q});

  q << 30, -45, 60, 20, -30, 15;
  cases.push_back({"General pose", q});

  q << -60, -30, 30, -90, 45, 0;
  cases.push_back({"Neg elbow config", q});

  int pass = 0, fail = 0;
  std::cout << "\n===== Round-trip FK/IK Tests (tol: "
            << TOL_POS << " mm, " << TOL_ORI << " deg) =====\n";
  std::cout << std::left
            << std::setw(28) << "Test"
            << std::setw(12) << "IK result"
            << std::setw(16) << "Pos err (mm)"
            << std::setw(16) << "Ori err (deg)"
            << "Status\n";
  std::cout << std::string(78, '-') << '\n';

  for (size_t i = 0; i < cases.size(); ++i)
  {
    RoundTripResult r = checkRoundTrip(robot, cases[i].q, TOL_POS, TOL_ORI);
    bool ok = r.ik_complete && r.pose_match;
    if (ok) ++pass; else ++fail;

    std::cout << std::left  << std::setw(28) << cases[i].name
              << std::setw(12) << (r.ik_complete ? "COMPLETE" : "FAILED")
              << std::fixed   << std::setprecision(6)
              << std::setw(16) << (r.ik_complete ? r.max_pos_err : -1.)
              << std::setw(16) << (r.ik_complete ? r.max_ori_err : -1.)
              << (ok ? "[PASS]" : "[FAIL]") << '\n';
  }
  std::cout << std::string(78, '-') << '\n';
  std::cout << "Round-trip result: " << pass << "/" << (pass + fail) << " passed\n";
  return fail;
}

int main(void)
{
#if __cplusplus >= 201402L
  std::cout << "Support C++ 14\n";
  //std::unique_ptr<Artic> robot = std::make_unique<Artic>();     // need C++14
  auto robot = std::make_unique<Artic>();                         // need C++14
#elif __cplusplus == 201103L
  std::cout << "Support C++ 11\n";
  std::unique_ptr<Artic> robot {new Artic};                       // need C++11
#else
  rb::kin::Artic* robot = new rb::kin::Artic();                   // if not support > C++11
#endif
  auto mPos = robot->getArmPose();
  std::cout << "initial position of TCP:\n";
  std::cout << mPos << '\n';
  std::cout << robot->getTCP() << '\n';

  // Test Forward Kinematics
  rb::math::Array6 qIn;
  qIn << 45. , -90 , 45. , 0  , 90. , 0;    // turn 1st joint 45 deg and 5th 90 deg.
  std::cout << "\nComputing time for Forward Kinematics of 6-axis robot arm: \n";
  {
    Timer timer;
    mPos = robot->forwardKin(qIn);
  }
  std::cout << "\n\nUpdate position of TCP:\n";
  std::cout << mPos << "\n\n";
  std::cout << robot->getTCP() << '\n';

  // Test Inverse Kinematics
  rb::math::VectorX joints;
  ArmAxisValue all_sols;
  IK_RESULT idx;
  std::cout << "\nComputing time for solving Inverse Kinematics of 6-axis robot arm: \n";
  {
    Timer timer;
    idx = robot->inverseKin(205.533,-205.533,403.475,
        -45.,     45.,   180.,
        joints, all_sols);
  }
  std::cout << '\n' << idx << '\n';
  std::cout << "\n The most fit solution: " << all_sols.fit << '\n';
  std::cout << "\n axis_value:\n" << all_sols.axis_value.transpose() << '\n';
  std::cout << "\n The joint values: \n" << joints << '\n';

#if __cplusplus < 201103L
  delete robot;
#endif

  // ===== Round-trip FK/IK Tests =====
  runRoundTripTests(*robot);

  // ===== Testing Link class =====
  std::vector<std::unique_ptr<rb::kin::Link>> links;
  links.push_back(std::unique_ptr<rb::kin::Link>(new rb::kin::Link(  0.0,  0., 339.,  0.,170,-170)));
  links.push_back(std::unique_ptr<rb::kin::Link>(new rb::kin::Link(  0.0, 90.,   0., 90., 45,-190)));
  links.push_back(std::unique_ptr<rb::kin::Link>(new rb::kin::Link(250.0,  0.,   0.,  0., 79,-209)));
  links.push_back(std::unique_ptr<rb::kin::Link>(new rb::kin::Link( 70.0, 90., 250.,  0.,190,-190)));
  links.push_back(std::unique_ptr<rb::kin::Link>(new rb::kin::Link(  0.0,-90.,   0.,  0.,120,-120)));
  links.push_back(std::unique_ptr<rb::kin::Link>(new rb::kin::Link(  0.0, 90.,  95.,  0.,350,-350)));
  rb::math::Matrix4 T06 = rb::math::Matrix4::Identity();
  for(auto& i : links)
  {
    T06 *= *i;
  }

  std::cout << "\n Links from 0 to 6:\n";
  std::cout << T06 << "\n\n";

  // group as kinematic chain
  std::unique_ptr<rb::kin::Artic> minibot(new Artic(links));
  std::cout << "\n Group Links as Articulated Robot Arm, then its pose:\n";
  std::cout << minibot->getArmPose() << '\n';


  // ===== Testing Polynomial class =====
  rb::math::Polynomial traj(5);  // a quintic (order of 5 polynomial function)
  std::vector<double> start = {403.475, 0., 0.};
  std::vector<double> end = {303.475, 0., 0.};
  double T = 2.0;
  traj.coeffQuintic(start, end, T);
  std::cout << "coefficient: \n" << traj.getCoeff();

  int step = 120;
  std::vector<double> z_t(step);
  rb::math::MatrixX sols(step, 6);
  std::cout << "\nFor solving Inverse Kinematics of 6-axis robot arm "<< step << " times: \n";
  {
    Timer timer;
    for(int i=0; i < step; ++i)
    {
      double dt = (T/(step-1))*i;
       z_t[i] = traj.getPosition(dt);
      idx = robot->inverseKin(205.533,-205.533, z_t[i],
          -45.,     45.,   180.,
          joints, all_sols);
      sols.row(i) << joints.transpose();
    }
  }
  std::cout << "\n The Solution values: \n" << sols << '\n';

  std::cout << "\n=====Distance Porfile=====\n";
  for(auto& z : z_t)
    std::cout << z << "\n";

  std::cout << "\n=====Velocity Porfile=====\n";
  for(int i=0; i < step; ++i)
  {
    double dt = (T/(step-1))*i;
    std::cout << traj.getVelocity(dt) << "\n";
  }

  return 0;
}
