/*!
 * @file        matrix.h
 * @brief       A head file define tpyes, and function for matrix manipulating
 * @author      Chien-Pin Chen
 */

#ifndef RB_MATRIX_H_
#define RB_MATRIX_H_
#include "unit.h"

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace rb    //! Robot Arm Library namespace
{
namespace math  //! math module namespace
{
typedef Eigen::MatrixXd MatrixX;        //!< make MatrixX as alias of Eigen::MatrixXd
typedef Eigen::Matrix3d Matrix3;        //!< make Matrix3 as alias of Eigen::Matrix3d
typedef Eigen::Matrix4d Matrix4;        //!< make Matrix4 as alias of Eigen::Matrix4d

typedef Eigen::VectorXd VectorX;        //!< make VectorX as alias of Eigen::VectorXd
typedef Eigen::Vector3d Vector3;        //!< make Vector3 as alias of Eigen::Vector3d
typedef Eigen::Vector4d Vector4;        //!< make Vector4 as alias of Eigen::Vector4d

typedef Eigen::AngleAxisd AngleAxis;    //!< make AngleAxis as alias of Eigen::AngleAxisd

//! make Array as alias of Eigen::Array (class template)
template <typename T, size_t R, size_t C>
  using Array = Eigen::Array<T, R, C>;

//! make Array6 as alias of Eigen::Array<double, 6, 1>
typedef Eigen::Array<double, 6, 1> Array6;

/*!
 * @brief Compute homogeneous transformation matrix for given link properties,
 *   and return the matrix.
 * @param A         Given link length.
 * @param alpha     Given link twist.
 * @param D         Given link offset.
 * @param theta     Given joint angle.
 * @return  Homogeneous transformation matrix of given link properties.
 */
inline Matrix4 homoTrans(const double& A, const double& alpha, const double& D, const double theta)
{
  double ct = cos(DEG2RAD * theta);
  double st = sin(DEG2RAD * theta);
  double ca = cos(DEG2RAD * alpha);
  double sa = sin(DEG2RAD * alpha);

  Matrix4 T;
  T << ct,   -st,   0,     A,
    st*ca, ct*ca, -sa, -sa*D,
    st*sa, ct*sa,  ca,  ca*D,
        0,     0,   0,     1;
  return T;
}

/*!
 * @brief Compute rotation matrix about X axis for given angle in radians.
 */
inline Matrix4 rotateX(const double& rad)
{
  Matrix3 m33;
  m33 = AngleAxis(rad, Vector3::UnitX());
  Matrix4 m = Matrix4::Identity();
  m.topLeftCorner(3, 3) << m33;
  return m;
}

/*!
 * @brief Compute rotation matrix about Y axis for given angle in radians.
 */
inline Matrix4 rotateY(const double& rad)
{
  Matrix3 m33;
  m33 = AngleAxis(rad, Vector3::UnitY());
  Matrix4 m = Matrix4::Identity();
  m.topLeftCorner(3, 3) << m33;
  return m;
}

/*!
 * @brief Compute rotation matrix about Z axis for given angle in radians.
 */
inline Matrix4 rotateZ(const double& rad)
{
  Matrix3 m33;
  m33 = AngleAxis(rad, Vector3::UnitZ());
  Matrix4 m = Matrix4::Identity();
  m.topLeftCorner(3, 3) << m33;
  return m;
}

/*!
 * @brief Extract Roll-Pitch-Yaw (Z-Y-X Euler) angles in radians from a
 *        homogeneous transformation matrix.
 * @param m         Input 4x4 HT matrix.
 * @param roll_z    Output roll  (rotation about Z) in radians.
 * @param pitch_y   Output pitch (rotation about Y) in radians.
 * @param yaw_x     Output yaw   (rotation about X) in radians.
 */
inline void tr2rpy(const Matrix4& m, double& roll_z, double& pitch_y, double& yaw_x)
{
  if (fabs(m(0,0)) < EPSILON && fabs(m(1,0)) < EPSILON)
  {
    roll_z  = 0;
    pitch_y = atan2(-m(2,0), m(0,0));
    yaw_x   = atan2(-m(1,2), m(1,1));
  }
  else
  {
    roll_z  = atan2(m(1,0), m(0,0));
    double sr = sin(roll_z);
    double cr = cos(roll_z);
    pitch_y = atan2(-m(2,0), cr * m(0,0) + sr * m(1,0));
    yaw_x   = atan2(sr * m(0,2) - cr * m(1,2), cr * m(1,1) - sr * m(0,1));
  }
}

/*!
 * @brief Build a homogeneous transformation matrix from Roll-Pitch-Yaw
 *        angles (Z-Y-X Euler) given in radians.
 * @param roll_z    Roll  (rotation about Z) in radians.
 * @param pitch_y   Pitch (rotation about Y) in radians.
 * @param yaw_x     Yaw   (rotation about X) in radians.
 * @param out       Output 4x4 HT matrix (rotation part only; translation unchanged).
 */
inline void rpy2tr(const double& roll_z, const double& pitch_y, const double& yaw_x, Matrix4& out)
{
  out = rotateZ(roll_z) * rotateY(pitch_y) * rotateX(yaw_x);
}

}   // namespace math
}   // namespace rb

#endif // RB_MATRIX_H_

