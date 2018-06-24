/*!
 * @file        matrix.h
 * @brief       A head file define tpyes, and function for matrix manipulating
 * @author      Chien-Pin Chen
 */

#ifndef RB_MATRIX_H_
#define RB_MATRIX_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace rb    //! Robot Arm Library namespace
{
    namespace math  //! math module namespace
    {
        typedef Eigen::MatrixXd MatrixX;        //!< make MatrixX as alias of Eigen::MatrixXd
        typedef Eigen::Matrix3d Matrix3;        //!< make Matrix3 as alias of Eigen::Matrix3d
        typedef Eigen::Matrix4d Matrix4;        //!< make Matrix4 as alias of Eigen::Matrix4d

        typedef Eigen::Vector3d Vector3;        //!< make Vector3 as alias of Eigen::Vector3d
        typedef Eigen::Vector4d Vector4;        //!< make Vector4 as alias of Eigen::Vector4d

        typedef Eigen::AngleAxisd AngleAxis;    //!< make AngleAxis as alias of Eigen::AngleAxisd

        //! make Array as alias of Eigen::Array (class template)
        template <typename T, size_t R, size_t C>
        using Array = Eigen::Array<T, R, C>;

        //! make Array6 as alias of Eigen::Array<double, 6, 1>
        typedef Eigen::Array<double, 6, 1> Array6;
    }
}

#endif // RB_MATRIX_H_

