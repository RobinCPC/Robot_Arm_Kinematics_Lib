/*!
 * @file        artic.h
 * @brief       A kinematics library for articulated robot arm
 * @author      Chien-Pin Chen
 */

#ifndef RB_ARTIC_H_
#define RB_ARTIC_H_

#include "kinematic_chain.h"


namespace rb //! Robot Arm Library namespace
{
namespace kin //! Kinematics module namespace
{
// Class of kinematics of articulated (6-axis) robot arm
/*! @class Artic     artic.h
 *  @brief A kinematics class for articulated (6-axis) robot arm
 *  Artic implement the kinematics of articulated (6-axis) robot arm
 */
class Artic : public KinematicChain
{
public:
  /*! Default Constuctor */
  Artic();

  /*! Constructor with certain robot arm data. */
  Artic(
      const rb::math::VectorX& a0,              //!< Link length of all links (mm)
      const rb::math::VectorX& alpha0,          //!< Twist angle of all links (degree)
      const rb::math::VectorX& d0,              //!< Link offset of all links (mm)
      const rb::math::VectorX& ini_theta,       //!< Initial value of all joint angles
      const rb::math::VectorX& uplimit0,        //!< Upper limit of all joints
      const rb::math::VectorX& lowlimit0        //!< Lower limit of all joints
      );

  /*! Destructor */
  ~Artic();

  /*!
   * @brief Compute forward Kinematics for given angle, update each joint
   *  value, and return current current position and orientation of TCP.
   * @param q         An array of joint degree.
   * @return ArmPose  a structure include position and orientation of TCP.
   */
  ArmPose forwardKin(const rb::math::VectorX& q);

  /*!
   * @brief Compute inverse kinematics for given position and orientation in
   *        Cartesian coordination system.
   * @return IK_RESULT  a enumerator indicates the result of inverse kinematics.
   */
  IK_RESULT inverseKin(
      const double& x,            //!< x value of the position.
      const double& y,            //!< y value of the position.
      const double& z,            //!< z value of the position.
      const double& roll,         //!< roll value of the orientation.
      const double& pitch,        //!< pitch value of the orientation.
      const double& yaw,          //!< yaw value of the orientation.
      rb::math::VectorX& joints,   //!< the best fittest IK solution.
      ArmAxisValue& all_sols      //!< a data structure to store all possible solutions.
      );

  /*!
   * @brief Compute inverse kinematics for two links (pitch-pitch) structure
   *        by given first joint.
   * @param th1_rad  angle (radian) of first joint.
   * @param p0       the position of wrist point wrt joint 0 (base)
   * @param config   a vector of 3 bits to indicator 8 configuration (solutions) of robot arm.
   * @param all_sols
   * @return
   */
  void solvePitchPitchIK(const double& th1_rad, const rb::math::Vector4& p0,
      const std::vector<bool>& config,
      ArmAxisValue& all_sols);

  /*!
   * @brief Compute inverse kinematics for wrist mechanism (row-pitch-row) structure
   *        by given first joint.
   * @param th1_rad  angle (radian) of first joint.
   * @param config   a vector of 3 bits to indicator 8 configuration (solutions) of robot arm.
   * @param flange_tr a H.T. Matrix of robot flange with respect to it base.
   * @param all_sols a structure to storage all possible solutions of inverse kinematics.
   */
  void solveRowPitchRowIK(const double& th1_rad, const std::vector<bool>& config,
      const rb::math::Matrix4& flange_tr,
      ArmAxisValue& all_sols);

  /*!
   * @brief Find the solutions in all_sols that is most fit to previous joints value
   * @param sols a structure to storage all possible solutions of inverse kinematics.
   */
  IK_RESULT solutionCheck(ArmAxisValue& sols);

  /*!
   * @brief Check the value of individual joint depend join limits. Will map
   *        map the value to corrected range if possible.
   * @param njoint    index of joint indicate which joint is examined.
   * @param rad       the value of joint will be check and update as output.
   */
  void preCheck(const int& njoint, double& rad);

  /*!
   * @brief Get the position and orientation of the flange of robot arm.
   * @return  ArmPose
   */
  ArmPose getArmPose(void) const;

  /*!
   * @brief Get the link length `a` value between each joint of the robot arm.
   * @return rb::math::VectorX
   */
  rb::math::VectorX getA(void) const;

  /*!
   * @brief Get the link twist \f$\alpha\f$  value between each joint of the robot arm.
   * @return rb::math::VectorX
   */
  rb::math::VectorX getAlpha(void) const;

  /*!
   * @brief Get the link offset `d` value between each joint of the robot arm.
   * @return rb::math::VectorX
   */
  rb::math::VectorX getD(void) const;

  /*!
   * @brief Get the joint angle \f$\theta\f$ value of each joint of the robot arm.
   * @return rb::math::VectorX
   */
  rb::math::VectorX getTheta(void) const;

  /*!
   * @brief Set the upper limits of joint angles for the robot arm.
   * @param up_lim    An rb::math::VectorX contain the upper limit of all joints.
   */
  void setUpLimit(rb::math::VectorX& up_lim);

  /*!
   * @brief Get the upper limits of joint angles of the robot arm.
   * @return rb::math::VectorX
   */
  rb::math::VectorX getUpLimit(void) const;

  /*!
   * @brief Set the lower limits of joint angles for the robot arm.
   * @param low_lim   An rb::math::VectorX contain the lower limit of all joints.
   */
  void setLowLimit(rb::math::VectorX& low_lim);

  /*!
   * @brief Get the lower limits of joint angles of the robot arm.
   * @return rb::math::VectorX
   */
  rb::math::VectorX getLowLimit(void) const;

protected:
  /*! Link length data member of modified D-H parameter for robot arm */
  rb::math::VectorX a;                          //!< Link length (mm)
  /*! Link twist data member of modified D-H parameter for robot arm */
  rb::math::VectorX alpha;                      //!< Link twist angle (degree)
  /*! Link offset data member of modified D-H parameter for robot arm */
  rb::math::VectorX d;                          //!< Link offset (mm)
  /*! Joint angle data member of modified D-H parameter for robot arm */
  rb::math::VectorX theta;                      //!< Joint angle (degree)
  rb::math::VectorX up_lim_;                    //!< Upper limit of all joints
  rb::math::VectorX low_lim_;                   //!< Lower limit of all joints

private:
  rb::math::VectorX ini_theta_;                 //!< Storage initialize 6 joint angle
  rb::math::VectorX pre_theta_;                 //!< Storage previous set of 6 joint angle, this will use to precheck result of IK
  rb::math::Matrix4 base_tcp_tf_;              //!< HT matrix of TCP with respected to robot base
  ArmPose tcp_pose_;                           //!< Position & orientation (x,y,z,a,b,c) of TCP in world coordination
  int pre_fit_solution_;                        //!< The index of configuration of previous IK

  // private functions
  // TODO: could move functions for matrix manipulating, such as rpy2tr & tr2rpy to math.h
  void tr2rpy(const rb::math::Matrix4& m, double& roll_z, double& pitch_y, double& yaw_x);
  void rpy2tr(double& roll_z, double& pitch_y, double& yaw_x, rb::math::Matrix4& tool_mat);
  rb::math::Matrix4 rotateX(const double& deg);
  rb::math::Matrix4 rotateY(const double& deg);
  rb::math::Matrix4 rotateZ(const double& deg);
};

}       // namespace kin
}       // namespace rb

#endif  // RB_ARTIC_H_
