#include <iostream>
#include <memory>
#include <chrono>

#include "../src/kin/artic.h"

using namespace rb::kin;

// TODO: Use smart_ptr to declare robot and use chrono to
// check computing time of FK and IK.
int main(void)
{
#if __cplusplus == 201103L
    std::cout << "Support C++ 11\n";
    std::unique_ptr<Artic> robot {new Artic};                       // need C++11
#elif __cplusplus >= 201402L
    std::cout << "Support C++ 14\n";
    //std::unique_ptr<Artic> robot = std::make_unique<Artic>();     // need C++14
    auto robot = std::make_unique<Artic>();                         // need C++14
#endif
    auto mPos = robot->getArmPose();
    std::cout << "initial position of TCP:\n";
    std::cout << mPos.T[5] << '\n';
    std::cout << mPos << '\n';

    // Test Forward Kinematics
    rb::math::Array6 qIn;
    qIn << 45. , -90 , 45. , 0  , 90. , 0;    // turn 1st joint 45 deg and 5th 90 deg.
    mPos = robot->forwardKin(qIn);
    std::cout << "\n\nUpdate position of TCP:\n";
    std::cout << mPos.T[5] << "\n\n";
    std::cout << mPos << "\n\n";

    // Test Inverse Kinematics
    rb::math::Array6 joints;
    ArmAxisValue all_sols;
    IK_RESULT idx = robot->inverseKin(205.533,-205.533,403.475,
                                         -45.,     45.,   180.,
                                      joints, all_sols);
    std::cout << '\n' << idx << '\n';
    std::cout << "\n The most fit solution: " << all_sols.fit << '\n';
    std::cout << "\n axis_value:\n" << all_sols.axis_value.transpose() << '\n';
    std::cout << "\n The joint values: \n" << joints << '\n';

    return 0;
}
