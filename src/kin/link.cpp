/*!
 * @file        link.cpp
 * @brief       A source file describe robot single link class
 * @author      Chien-Pin Chen
 */

#include "link.h"

namespace rb
{
    namespace kin
    {
        /*! Default constructor */
        Link::Link() :
            a(0), alpha(0), d(0), theta(0),
            up_lim(0), low_lim(0), offset(0),
            jt_type(Joint::REVOLUTE), mass(0),
            I(rb::math::Matrix3::Identity()),
            r(rb::math::Vector3::Zero())
        {
            // compute link transformation matrix
            this->tf = rb::math::homoTrans(a, alpha, d, theta);
        }

        Link::Link(
                const double a0,
                const double alpha0,
                const double d0,
                const double theta0,
                const double up_lim0,
                const double low_lim0,
                const double offset0,
                const Joint type0,
                const double mass0,
                const rb::math::Matrix3 I0,
                const rb::math::Vector3 r0)
        {
            // Initial the properties of link
            this->a = a0;
            this->alpha = alpha0;
            this->jt_type = type0;
            this->offset = offset0;
            this->up_lim = up_lim0;
            this->low_lim = low_lim0;

            switch(type0)
            {
                case Joint::REVOLUTE:
                    this->d = d0;
                    this->theta = theta0 + offset0;
                    break;
                case Joint::PRISMATIC:
                    this->d = d0 + offset0;
                    this->theta = theta0;
                    break;
            }

            // compute link transformation matrix
            this->tf = rb::math::homoTrans(a0, alpha0, d0, theta0);

            // update dynamic properties
            this->mass = mass0;
            this->I = I0;
            this->r = r0;
        }

        Link::~Link(){}

        rb::math::Matrix4 operator* (Link& lhs, Link& rhs)
        {
            rb::math::Matrix4 result = lhs.tf * rhs.tf;
            return result;
        }

        rb::math::Matrix4& operator*= (rb::math::Matrix4& lhs, Link& rhs)
        {
            lhs *= rhs.tf;
            return lhs;
        }

        rb::math::Matrix4 Link::computeTransform(const double& q, const bool& update)
        {
            rb::math::Matrix4 tf_matrix;

            switch(jt_type)
            {
                case Joint::REVOLUTE:
                    tf_matrix = rb::math::homoTrans(this->a, this->alpha,
                            this->d, q);
                    if(update)
                    {
                        this->tf = tf_matrix;
                        this->theta = q;
                    }
                    break;
                case Joint::PRISMATIC:
                    tf_matrix = rb::math::homoTrans(this->a, this->alpha,
                            q, this->theta);
                    if(update)
                    {
                        this->tf = tf_matrix;
                        this->d = q;
                    }
            }
            return tf_matrix;
        }

    }
}
