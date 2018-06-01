/*!
 * @file        rakl.h
 * @brief       A kinematics library for articulated robot arm
 * @author      Chien-Pin Chen
 */
#pragma once
#include <Eigen/Dense>

using Eigen::Matrix;
using Eigen::Array;
using Eigen::MatrixXd;
using Eigen::Matrix4d;

typedef Array<double, 6, 1> Array6d;

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
        Array< Matrix4d, 7, 1> T;
    }ARM_POS;

    /*! @struct ARM_AXIS_VALUE  rakl.h
     *  @brief  A struct variable for joints of robot arm
     *  ARM_AXIS_VALUE storage the output of Inverse kinematic
     */
    typedef struct ARM_AXIS_VALUE
    {
        /*! Joints Angle of 8 solutions*/
        MatrixXd axis_value;
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
    }ARM_AXIS_VALUE;

}

// Class of kinematics of 6-axis robot arm
/*! @class rakl     rakl.h
 *  @brief A kinematics class
 *  rakl implement the kinematics of 6-axis (articulated) robot arm
 */
class rakl
{
public:
    rakl();         //!< Default Constructor
    rakl(
        Array6d a0,                     //!< Link length of all links (mm)
        Array6d alpha0,                 //!< Twist angle of all links (degree)
        Array6d d0,                     //!< Link offset of all links (mm)
        Array6d ini_theta,              //!< Initial value of all joint angles
        Array6d uplimit0,               //!< Upper limit fo all joints
        Array6d lowlimit0               //!< Lower limit fo all joints
        );
    ~rakl();        //!< Destructor
    RA::ARM_POS forwardKin(Array6d q);
    RA::ARM_POS getArmPos(void);


protected:
    /*! Input data of modified D-H parameter for robot arm */
    Array6d a;                          //!< Link length (mm)
    Array6d alpha;                      //!< Link twist angle (degree)
    Array6d d;                          //!< Link offset (mm)
    Array6d theta;                      //!< Joint angle (degree)
    Array6d uplimit;                    //!< Upper  limit of all joints
    Array6d lowlimit;                   //!< Lower limit of all joints
    Matrix4d work_base_T;               //!< Homogeneous transformation matrix of work base
    Matrix4d work_base;                 //!< Position & Orientation of TCP in work_base coordination

private:
    Array6d m_ini_theta;            //!< Storage initialize 6 joint angle
    Array6d m_pre_theta;            //!< Storage previous set of 6 joint angle, this will use to precheck result of IK
    Matrix4d m_T_act;               //!< HT matrix of TCP wrt work base
    Matrix4d m_tool_T;              //!< HT matrix of TCP wrt  joint 6th (last joint coordination)
    RA::ARM_POS m_pos_act;          //!< position & orient ation (x,y,z,a,b,c) of TCP, and HT matrix of each joint & work base
    int pre_fit_solution;           //!< The index of configuration of previous IK

    // private functions
    Matrix4d Homo_trans(double& A, double& alpha, double& D, double& theta);
    void tr2rpy(const Matrix4d& m, double& roll_z, double& pitch_y, double& yaw_x);
};
