/*!
 * @file        rakl.h
 * @brief       A kinematics library for articulated robot arm
 * @author      Chien-Pin Chen
 */
#pragma once
#include <Eigen/Dense>

using Eigen::Matrix;
using Eigen::Vector7d;
using Eigen::MatrixXd;


/*!
 * @enum IK_RESULT
 * A set of enumeration to present the result of inverse kinematics (IK)
 */
enum IK_RESULT
{
    IK_COMPLELE,        /*!< value 0 */
    IK_NO_SOLUTION,     /*!< value 1 */
    IK_ANGLE_LIMIT,     /*!< value 2 */
    IK_SINGULAR,        /*!< value 3 */
    IK_INPUT_INVALID    /*!< value 4 */
};

namespace RA //!< Robot Arm Property
{
    /*! @struct AMR_POS     rakl.h
     *  @brief  A struct variable for Tool Center Point (TCP)
     *  ARM_POS storages the ouput of forward kinematic
     */
    typedef Matrix<Vector7d, 4,4> Matrix744d;
    typedef struct ARM_POS
    {
        /*! Position of TCP (mm) */
        double x;
        double y;
        double z;
        /*! Orientation of TCP (degree) */
        double a;   //!< roll
        double b;   //!< pitch
        double c;   //!< yaw
        /*! 2D Array of Homogeneous Transformation Matrix for 6 axes
         *  and work_base coordination system
         */
        Matrix744d T;
    };

    /*! @struct ARM_AXIS_VALUE  rakl.h
     *  @brief  A struct variable for joints of robot arm
     *  ARM_AXIS_VALUE storage the output of Inverse kinematic
     */
    typedef struct ARM_AXIS_VALUE
    {
        /*! Joints Angle of 8 solutions*/
        MatrixXd axis_value(8,6);
        /*! use following method to find the fittest folution */
        int fit;                    //!< the number of the fittest solutiions.
        bool solution_check[8];     //!< check if having solution.
        bool singular_check[8];     //!< check if in singular point.
        bool limit_check[8];        //!< check if over angle limit.

        ARM_AXIS_VALUE()
        {
            axis_value = MatrixXd::Constant(8, 6, 0.0);
            fit = 0;
            for (int i = 0; i < 8; i++)
            {
                solution_check[i] = singular_check[i] = limit_check[i] = 0;
            }
        }
    };

}

