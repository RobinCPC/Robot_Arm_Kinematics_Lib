/*!
 * @file        rakl.h
 * @brief       A kinematics library for articulated robot arm
 * @author      Chien-Pin Chen
 */
#ifndef RA_RAKL_H_
#define RA_RAKL_H_
#pragma once
#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using Eigen::Array;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::AngleAxisd;

typedef Array<double, 6, 1> Array6d;


namespace RA //! Robot Arm Library namespace
{
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

    /*! @struct ARM_POS     rakl.h
     *  @brief  A struct variable for Tool Center Point (TCP)
     *  ARM_POS storages the output of forward kinematic
     */
    struct ARM_POS
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
    };

    /*!
     *  @brief  Overload << operator to print position & orieantation in  ARM_POS.
     */
    std::ostream& operator<< (std::ostream& ost, const ARM_POS& pose);

    /*! @struct ARM_AXIS_VALUE  rakl.h
     *  @brief  A struct variable for joints of robot arm
     *  ARM_AXIS_VALUE storage the output of Inverse kinematic
     */
    struct ARM_AXIS_VALUE
    {
        /*! Joints Angle of 8 solutions*/
        MatrixXd axis_value;
        /*! Use following method to find the fittest solution */
        int fit;                    //!< the index of the fittest solution.
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


// Class of kinematics of 6-axis robot arm
/*! @class rakl     rakl.h
 *  @brief A kinematics class
 *  rakl implement the kinematics of 6-axis (articulated) robot arm
 */
class rakl
{
public:
    rakl();         //!< Default Constructor

    /*! Constructor with certain robot arm data. */
    rakl(
        const Array6d& a0,              //!< Link length of all links (mm)
        const Array6d& alpha0,          //!< Twist angle of all links (degree)
        const Array6d& d0,              //!< Link offset of all links (mm)
        const Array6d& ini_theta,       //!< Initial value of all joint angles
        const Array6d& uplimit0,        //!< Upper limit of all joints
        const Array6d& lowlimit0        //!< Lower limit of all joints
        );
    ~rakl();        //!< Destructor

    /*!
     * @brief Compute forward Kinematics for given angle, update each joint
     *  value, and return current current position and orientation of TCP.
     * @param q         An array of joint degree.
     * @return ARM_POS  a structure include position and orientation of TCP.
     */
    ARM_POS forwardKin(const Array6d& q);



    /*!
     * @brief Compute homogeneous transformation matrix for given link properties,
     *   and return the matrix.
     * @param A         Given link length.
     * @param alpha     Given link twist.
     * @param D         Given link offset.
     * @param theta     Given joint angle.
     * @return  Homogeneous transformation matrix of given link properties.
     */
    Matrix4d Homo_trans(double& A, double& alpha, double& D, const double& theta);

    /*!
     * @brief Get the position and orientation of the robot arm.
     * @return  ARM_POS
     */
    ARM_POS getArmPos(void);

    /*!
     * @brief Set the HT matrix of the offset b/w the arm flange and
     *        equiped tool or end-effector.
     * @param tool_offset
     * @return bool     Check if setting success
     */
    bool setToolOffset(ARM_POS tool_offset);

    /*!
     * @brief Set the HT matrix of the working base of the robot arm.
     * @param   Matrix4d
     * @return
     */
    void setBase(Matrix4d& base);

    /*!
     * @brief Get the HT matrix of the working base of the robot arm.
     * @return Matrix4d
     */
    Matrix4d getBase(void);

    /*!
     * @brief Get the link length `a` value between each joint of the robot arm.
     * @return Array6d
     */
    Array6d getA(void);

    /*!
     * @brief Get the link twist \f$\alpha\f$  value between each joint of the robot arm.
     * @return Array6d
     */
    Array6d getAlpha(void);

    /*!
     * @brief Get the link offset `d` value between each joint of the robot arm.
     * @return Array6d
     */
    Array6d getD(void);

    /*!
     * @brief Get the joint angle \f$\theta\f$ value of each joint of the robot arm.
     * @return Array6d
     */
    Array6d getTheta(void);

    /*!
     * @brief Set the upper limits of joint angles for the robot arm.
     * @param Array6d
     */
    void setUpLimit(Array6d& up_lim);

    /*!
     * @brief Get the upper limits of joint angles of the robot arm.
     * @return Array6d
     */
    Array6d getUpLimit(void);

    /*!
     * @brief Set the lower limits of joint angles for the robot arm.
     * @param Array6d
     */
    void setLowLimit(Array6d& low_lim);

    /*!
     * @brief Get the lower limits of joint angles of the robot arm.
     * @return Array6d
     */
    Array6d getLowLimit(void);


protected:
    /*! Link length data member of modified D-H parameter for robot arm */
    Array6d a;                          //!< Link length (mm)
    /*! Link twist data member of modified D-H parameter for robot arm */
    Array6d alpha;                      //!< Link twist angle (degree)
    /*! Link offset data member of modified D-H parameter for robot arm */
    Array6d d;                          //!< Link offset (mm)
    /*! Joint angle data member of modified D-H parameter for robot arm */
    Array6d theta;                      //!< Joint angle (degree)
    Array6d uplimit;                    //!< Upper  limit of all joints
    Array6d lowlimit;                   //!< Lower limit of all joints
    Matrix4d work_base_T;               //!< Homogeneous transformation matrix of work base
    Matrix4d work_base;                 //!< Position & Orientation of TCP in work_base coordination

private:
    Array6d m_ini_theta;            //!< Storage initialize 6 joint angle
    Array6d m_pre_theta;            //!< Storage previous set of 6 joint angle, this will use to precheck result of IK
    Matrix4d m_T_act;               //!< HT matrix of TCP with respected to work base
    Matrix4d m_tool_T;              //!< HT matrix of TCP with respected to 6th joint (last joint coordination)
    ARM_POS m_pos_act;          //!< position & orientation (x,y,z,a,b,c) of TCP, and HT matrix of each joint & work base
    int pre_fit_solution;           //!< The index of configuration of previous IK

    // private functions
    // TODO: could move functions for matrix manipulating, such as rpy2tr & tr2rpy to math.h
    void tr2rpy(const Matrix4d& m, double& roll_z, double& pitch_y, double& yaw_x);
    void rpy2tr(double& roll_z, double& pitch_y, double& yaw_x, Matrix4d& tool_mat);
    Matrix4d rotateX(const double& deg);
    Matrix4d rotateY(const double& deg);
    Matrix4d rotateZ(const double& deg);
};

}       // namespace RA
#endif  // RA_RAKL_H_
