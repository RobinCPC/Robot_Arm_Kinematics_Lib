/*!
 * @file        kinematic_chain.h
 * @brief       A header file for the class of general kinematic chain
 * @author      Chien-Pin Chen
 */

#ifndef RB_KINEMATIC_CHAIN_H_
#define RB_KINEMATIC_CHAIN_H_
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <array>

#include "link.h"

#include <Eigen/StdVector>

namespace rb //! Robot Arm Library namespace
{
namespace kin //! Kinematics module namespace
{
/*!
 * @enum IK_RESULT
 * A set of enumeration to present the result of inverse kinematics (IK)
 */
enum IK_RESULT
{
  IK_COMPLETE,        /*!< value 0 */
  IK_NO_SOLUTION,     /*!< value 1 */
  IK_ANGLE_LIMIT,     /*!< value 2 */
  IK_SINGULAR,        /*!< value 3 */
  IK_INPUT_INVALID    /*!< value 4 */
};

/*! @struct ArmPose     kinematic_chain.h
 *  @brief  A struct variable for Tool Center Point (TCP)
 *  ArmPose storages the output of forward kinematic
 */
struct ArmPose
{
  /*! Position of TCP (mm) */
  double x;   //!< frond and back
  double y;   //!< left and right
  double z;   //!< up and down
  /*! Orientation of TCP (degree) */
  double a;   //!< roll
  double b;   //!< pitch
  double c;   //!< yaw
};

/*!
 *  @brief  Overload << operator to print position & orientation in  ArmPose.
 *  @param  ost     ostream object to storage string for printing out.
 *  @param  pose    ArmPose object that will be printed out.
 */
std::ostream& operator<< (std::ostream& ost, const ArmPose& pose);

#if __cplusplus == 201103L
/*!
 *  @brief  a template to ostream a vector of different type (char, double)
 *  @tparam T   the type that storage in vector for printing out.
 */
template<typename T>
  void ostrVec(std::ostream& ost, std::vector<T> vec_inp, int width=12)
  {
    ost << '\n';
    for(auto& inp : vec_inp)
      ost << std::right << std::setw(width) << inp;
    return;
  }
#endif

/*! @struct ArmAxisValue  kinematic_chain.h
 *  @brief  A struct variable for joints of robot arm
 *  ArmAxisValue storage the output of Inverse kinematic
 */
struct ArmAxisValue
{
  /*! Joints Angle of 8 solutions*/
  rb::math::MatrixX axis_value;
  /*! Use following method to find the fittest solution */
  int fit;                                        //!< the index of the fittest solution.
  std::array<bool, 8> solution_check = {{0}};     //!< check if having solution.
  std::array<bool, 8> singular_check = {{0}};     //!< check if in singular point.
  std::array<bool, 8> limit_check = {{0}};        //!< check if over angle limit.

  ArmAxisValue()
  {
    axis_value = rb::math::MatrixX::Constant(8, 6, 0.0);
    fit = 0;
    for (int i = 0; i < 8; i++)
    {
      solution_check[i] = singular_check[i] = limit_check[i] = 0;
    }
  }
};


/*! @class KinematicChain     kinematic_chain.h
 *  @brief A kinematics class for general serial links.
 *  KinematicChain implement the kinematics of general serial links
 */
class KinematicChain
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /*! Default Constuctor */
  KinematicChain();

  /*! Constructor with a Links vector and other parameter*/
  KinematicChain(
      std::vector<rb::kin::Link*> links,
      rb::math::Matrix4  base=rb::math::Matrix4::Identity(),
      rb::math::Matrix4  tool=rb::math::Matrix4::Identity(),
      rb::math::Vector3  gravity={0., 0., rb::math::GRAVITY},
      std::string manufactor="None",
      std::string model="None"
      );

  /*! Destructor */
  virtual ~KinematicChain();

  /*!
   * @brief Compute forward Kinematics for given angle, update each joint
   *  value, and return current current position and orientation of TCP.
   * @param q         An array of joint value (degree or mm).
   * @param update    A boolean to check if update the value of joints and frames.
   * @return  a structure include position and orientation of TCP.
   */
  virtual ArmPose forwardKin(const rb::math::VectorX& q, const bool update=true);

  /*!
   * @brief Compute inverse kinematics for given tranformation matrix of TCP in
   *        Cartesian coordination system.
   * @return IK_RESULT  a enumerator indicates the result of inverse kinematics.
   */
  virtual IK_RESULT inverseKin(
          const rb::math::Matrix4& world_tcp_tf,  //!< Transformation of tcp in world coordination.
          rb::math::VectorX& joints,              //!< The best fittest solution.
          ArmAxisValue& all_sols                  //!< A data structure to store all possible solutions.
          ) = 0;

  /*!
   * @brief Compute homogeneous transformation matrix for given link properties,
   *   and return the matrix.
   * @param A         Given link length.
   * @param alpha     Given link twist.
   * @param D         Given link offset.
   * @param theta     Given joint angle.
   * @return  Homogeneous transformation matrix of given link properties.
   */
  rb::math::Matrix4 homoTrans(double& A, double& alpha, double& D, const double& theta);

  /*!
   * @brief Set the HT matrix of the offset b/w the arm flange and
   *        equipped tool or end-effector.
   * @param tool_pose
   * @return     Check if setting success
   */
  bool setTool(const ArmPose& tool_pose);

  /*!
   * @brief Get the HT matrix of the offset b/w the arm flange and
   *        equipped tool or end-effector.
   * @param tool_pose
   * @return bool     Check if setting success
   */
  bool getTool(ArmPose& tool_pose) const;

  /*!
   * @brief Set the Homogeneous Transformation matrix of the working base of the robot arm.
   * @param base  a Homogeneous Transformation matrix from World -> the robot base.
   * @return
   */
  void setBase(const rb::math::Matrix4& base);

  /*!
   * @brief Get the HT matrix of the working base of the robot arm.
   * @return rb::math::Matrix4
   */
  rb::math::Matrix4 getBase(void) const;

  /*!
   * @brief Get the HT matrix of TCP w.r.t world coordination.
   * @return
   */
  rb::math::Matrix4 getTCP(void) const;

  /*!
   * @brief Set the degree of freedom according to the property of links.
   * @return
   */
  void setDOF(void);

  /*!
   * @brief Get the degree of freedom of this kinematic chain
   * @return
   */
  int getDOF(void) const;

protected:
  /*! A vector of Link class. */
  std::vector<rb::kin::Link*> links_;
  //std::vector<rb::kin::Link*, Eigen::aligned_allocator<rb::kin::Link*> > links_;

  /*!< A vector of HT matrix to the pose of each joint and TCP. */
  std::vector<rb::math::Matrix4*> frames_;
  //std::vector<rb::math::Matrix4*, Eigen::aligned_allocator<rb::math::Matrix4*> > frames_;

  int dof_;                                   //!< A integer indicates the degree of freedom
  rb::math::Matrix4 base_tf_;                 //!< HT matrix of robot base with respected to world coordination
  rb::math::Matrix4 tool_tf_;                 //!< HT matrix of TCP with respected to robot flange (last joint)
  rb::math::Matrix4 world_tcp_tf_;            //!< HT matrix of TCP with respected to world coordination.
  rb::math::Vector3 gravity_;                 //!< Gravity set as a vector.

  std::string manufactor_;                    //!< the name of manufactor of the robot
  std::string model_;                         //!< the model of the robot named by its manufactor

private:
  // private functions
  // TODO: could move functions for matrix manipulating, such as rpy2tr & tr2rpy to math.h
  void tr2rpy(const rb::math::Matrix4& m, double& roll_z, double& pitch_y, double& yaw_x) const;
  void rpy2tr(double& roll_z, double& pitch_y, double& yaw_x, rb::math::Matrix4& tool_mat);
  rb::math::Matrix4 rotateX(const double& deg);
  rb::math::Matrix4 rotateY(const double& deg);
  rb::math::Matrix4 rotateZ(const double& deg);
};

}       // namespace kin
}       // namespace rb

#endif  // RB_KINEMATIC_CHAIN_H_
