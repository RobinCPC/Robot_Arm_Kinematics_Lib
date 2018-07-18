/*!
 * @file        link.h
 * @brief       A head file describe robot single link class
 * @author      Chien-Pin Chen
 */

#ifndef RB_LINK_H_
#define RB_LINK_H_

#include "../math/matrix.h"

namespace rb //! Robot Arm Library namespace
{
namespace kin //! Kinematics module namespace
{
/*!
 * @enum Joint
 * A set of enumeration to indicate the joint type of the robot link.
 */
enum Joint
{
  REVOLUTE,       //!< Rotated joint type
  PRISMATIC       //!< Translated joint type
};

/*! @class Link     link.h
 *  @brief A class handle single robot link
 *  Link implement the model of single of robot link
 */
class Link
{
public:
  Link();                                 //!< Default Constructor

  /*! Constructor with certain link data. */
  Link(
      const double a0,                    //!< Length of the link (mm)
      const double alpha0,                //!< Twist angle of the link (degree)
      const double d0,                    //!< Joint offset of the link (mm)
      const double theta0,                //!< Joint angle of the link (degree)
      const double up_lim0=360.,          //!< Upper limit of the link
      const double low_lim0=-360.,        //!< Lower limit of the link
      const double offset0=0.,            //!< Joint coordinate offset
      const Joint type0=Joint::REVOLUTE,  //!< Joint type of the link
      /*! The mass of the link */
      const double mass0=0.,
      /*! The Inertia Matrix of the link */
      const rb::math::Matrix3 I0=rb::math::Matrix3::Identity(),
      /*! The vector of the link Center of Gravity w.r.t link coordinate frame */
      const rb::math::Vector3 r0=rb::math::Vector3::Zero()
      );


  ~Link();                                //!< Destructor

  /*!
   *  @brief  Overload multiply operator to concatenate transformation of two links.
   *  @param  lhs     A link class at left hand side.
   *  @param  rhs     A link class at right hand side.
   *  @return A 4X4 matrix that storage chaining transformation of two links.
   */
  friend rb::math::Matrix4 operator* (Link& lhs, Link& rhs);

  /*!
   *  @brief  Overload multiply operator to concatenate transformation of two links.
   *  @param  lhs     A 4x4 matrix at left hand side.
   *  @param  rhs     A link class at right hand side.
   *  @return A 4X4 matrix that storage chaining transformation of two links.
   */
  friend rb::math::Matrix4 operator* (rb::math::Matrix4& lhs, Link& rhs);

  /*!
   *  @brief  Overload multiply operator to concatenate transformation of two links.
   *  @param  lhs     A 4x4 matrix at left hand side.
   *  @param  rhs     A link class at right hand side.
   *  @return A 4X4 matrix that storage chaining transformation of a 4x4 matrix and a link.
   */
  friend rb::math::Matrix4& operator*= (rb::math::Matrix4& lhs, Link& rhs);

  /*!
   * @brief Compute homogeneous transformation matrix of the link by giving joint value,
   *   and return the matrix.
   * @param q         Given joint value.
   * @param update    A boolean to indicate if need to update transformation (tf) of the link.
   * @return  Homogeneous transformation matrix of the link by giving joint value.
   */
  rb::math::Matrix4 computeTransform(const double& q, const bool& update=false);

  /*! Link length data member of modified D-H parameter for link */
  double a;                   //!< Link length (mm)
  /*! Link twist data member of modified D-H parameter for link */
  double alpha;               //!< Link twist angle (degree)
  /*! Link offset data member of modified D-H parameter for link */
  double d;                   //!< Link offset (mm)
  /*! Joint angle data member of modified D-H parameter for link */
  double theta;               //!< Joint angel (degree)
  /*! Upper limit data member of modified D-H parameter for link */
  double up_lim;               //!< Upper limit of joint or link offset.
  /*! Lower limit data member of modified D-H parameter for link */
  double low_lim;              //!< Lower limit of joint or link offset.
  /*! Joint offset data member of modified D-H parameter for link */
  double offset;              //!< Joint (revolute or prismatic) coordinate offset.

  /*! Link transformation data member of modified D-H parameter for link */
  rb::math::Matrix4 tf;       //!< Homogeneous transformation matrix describe internal link transformation.

protected:
  /*! Joint type data member of of the link */
  Joint jt_type;              //!< Index to indicate the joint type of this link.

  double mass;                //!< Link mass.
  rb::math::Matrix3 I;        //!< Inertia matrix about link Center of Gravity (CoG).
  rb::math::Vector3 r;        //!< Link CoG w.r.t link coordinate frame.

private:

};

}       // namespace kin
}       // namespace rb

#endif  // RB_LINK_H_
