#include <iostream>
#include <memory>
#include <chrono>

#include "../src/kin/link.h"
#include "../src/kin/artic.h"

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
  std::cout << mPos.T[5] << '\n';
  std::cout << mPos << '\n';

  // Test Forward Kinematics
  rb::math::Array6 qIn;
  qIn << 45. , -90 , 45. , 0  , 90. , 0;    // turn 1st joint 45 deg and 5th 90 deg.
  std::cout << "\nComputing time for Forward Kinematics of 6-axis robot arm: \n";
  {
    Timer timer;
    mPos = robot->forwardKin(qIn);
  }
  std::cout << "\n\nUpdate position of TCP:\n";
  std::cout << mPos.T[5] << "\n\n";
  std::cout << mPos << "\n\n";

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

  // ===== Testing Link class =====
  std::vector<rb::kin::Link> links;
  links.push_back(rb::kin::Link(  0.0,180.,-335.,  0.,170,-170));
  links.push_back(rb::kin::Link( 75.0, 90.,   0.,-90., 45,-190));
  links.push_back(rb::kin::Link(270.0,  0.,   0.,  0., 79,-209));
  links.push_back(rb::kin::Link( 90.0, 90.,-295.,  0.,190,-190));
  links.push_back(rb::kin::Link(  0.0,-90.,   0.,  0.,120,-120));
  links.push_back(rb::kin::Link(  0.0,-90.,  80.,  0.,350,-350));
  rb::math::Matrix4 T06 = rb::math::Matrix4::Identity();
  for(auto i : links)
  {
    T06 *= i;
  }

  std::cout << "\n Links from 0 to 6:\n";
  std::cout << T06 << "\n\n";


  return 0;
}
