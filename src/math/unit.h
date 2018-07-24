/*!
 * @file        unit.h
 * @brief       A head file for converting units.
 * @author      Chien-Pin Chen
 */

#ifndef RB_UNIT_H_
#define RB_UNIT_H_
#include <limits>

namespace rb    //! Robot Arm Library namespace
{
namespace math  //! math module namespace
{
  /*! Constant to present value of pi (more precise). */
  static const double      PI = 3.1415926535897932384626433832795;

  /*! Constant to present converting from radian to degree. */
  static const double RAD2DEG = 57.295779513082320876798154814105;

  /*! Constant to present  converting from degree to radian. */
  static const double DEG2RAD = 0.01745329251994329576923690768489;

  /*! Constant as critera if some variable close enough to zero. */
  static const double EPSILON = std::numeric_limits<float>::epsilon();

  /*! Constant to present value of gravity. */
  static const double GRAVITY = 0.980665;
}
}

#endif // RB_UNIT_H_
