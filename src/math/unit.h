/*!
 * @file        unit.h
 * @brief       A head file for converting units.
 * @author      Chien-Pin Chen
 */

#ifndef RB_UNIT_H_
#define RB_UNIT_H_

namespace rb    //! Robot Arm Library namespace
{
    namespace math  //! math module namespace
    {
        //! constant to present value of pi (more precise)
        static const double      PI = 3.1415926535897932384626433832795;

        //!< constant to present converting from radian to degree
        static const double RAD2DEG = 57.295779513082320876798154814105;

        //!< constant to present  converting from degree to radian
        static const double DEG2RAD = 0.01745329251994329576923690768489;

        //!< constant as critera if some variable close enough to zero
        static const double EPSILON = 2.22044604925031e-5;

    }
}

#endif // RB_UNIT_H_
