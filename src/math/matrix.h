/*!
 * @file        matrix.h
 * @brief       A head file define tpyes, and function for matrix manipulating
 * @author      Chien-Pin Chen
 */

#ifndef RB_MATRIX_H_
#define RB_MATRIX_H_
#include "unit.h"

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

}   // namespace math
}   // namespace rb

#endif // RB_MATRIX_H_

