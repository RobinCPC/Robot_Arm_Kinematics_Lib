#include <iostream>
#include "../src/rakl.h"

int main(void)
{
    rakl robot = rakl();
    RA::ARM_POS mPos = robot.getArmPos();
    std::cout << "initial position of TCP:\n";
    std::cout << mPos.T[5] << '\n';
    std::cout << "\tx, \ty, \tz\n";
    std::cout << '\t' << mPos.x << '\t' << mPos.y << '\t' << mPos.z << '\n';
    std::cout << "\ta, \tb, \tc\n";
    std::cout << '\t' << mPos.a << '\t' << mPos.b << '\t' << mPos.c << '\n';

    Array6d qIn;
    qIn << 90. , -90 , 0 , 0  , 0 , 0;
    mPos = robot.forwardKin(qIn);
    std::cout << "\n\nUpdate position of TCP:\n";
    std::cout << mPos.T[5] << "\n\n";
    std::cout << "\tx, \ty, \tz\n";
    std::cout << '\t' << mPos.x << '\t' << mPos.y << '\t' << mPos.z << '\n';
    std::cout << "\ta, \tb, \tc\n";
    std::cout << '\t' << mPos.a << '\t' << mPos.b << '\t' << mPos.c << '\n';
    return 0;
}
