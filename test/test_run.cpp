#include <iostream>
#include <memory>
#include <chrono>

#include "../src/rakl.h"

using namespace RA;

// TODO: Use smart_ptr to declare robot and use chrono to
// check computing time of FK and IK.
int main(void)
{
    //std::unique_ptr<rakl> robot {new rakl};                   // need C++11
    //std::unique_ptr<rakl> robot = std::make_unique<rakl>();   // need C++14
    auto robot = std::make_unique<rakl>();                      // need C++14
    auto mPos = robot->getArmPos();
    std::cout << "initial position of TCP:\n";
    std::cout << mPos.T[5] << '\n';
    std::cout << mPos << '\n';

    // Test Forward Kinematics
    Array6d qIn;
    qIn << 45. , -90 , 0 , 0  , 90. , 0;    // turn 1st joint 45 deg and 5th 90 deg.
    mPos = robot->forwardKin(qIn);
    std::cout << "\n\nUpdate position of TCP:\n";
    std::cout << mPos.T[5] << "\n\n";
    std::cout << mPos;

    // Test Inverse Kinematics
    Array6d joints;
    ARM_AXIS_VALUE all_sols;
    IK_RESULT idx = robot->inverseKin(261.63, -261.63,
                                      615., -45., 0., 180.,
                                      joints, all_sols);
    std::cout << '\n' << idx << '\n';

    return 0;
}
