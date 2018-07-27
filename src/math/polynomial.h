/*!
 * @file        polynomial.h
 * @brief       A header file for the class of generating polynomial function
 * @author      Chien-Pin Chen
 */

#ifndef RB_POLYNOMIAL_H_
#define RB_POLYNOMIAL_H_
#include <vector>

#include "matrix.h"

namespace rb //! Robot Arm Library namespace
{
namespace math //! Math module namespace
{
/*! @class Polynomial   polynomial.h
 *  @brief A class for constructing polynomial function.
 */
class Polynomial
{
public:
  /*! Default Construction */
  Polynomial() : degree_(0), c_(degree_+1), t_(0.)
  {
    // Construct a straight line as Default.
  };

  /*! Construct polynomial with given degree. */
  Polynomial(const short deg, const double t=0.) :
    degree_(deg), c_(deg+1), t_(t)
  {
  };

  /*!
   * @brief Compute the coefficients of quintic (fifth order) polynomial by giving
   *  initial conditions of start and end.
   * @param start     initial condition of f[0], fd[0], fdd[0].
   * @param end       initial condition of f[T], fd[T], fdd[T].
   * @param T         duration (time interval) of polynomial function.
   */
  void coeffQuintic(
      const std::vector<double>& start,
      const std::vector<double>& end,
      const double& T
      )
  {
    // from initial condition at t=0.
    this->c_[0] = start[0];
    this->c_[1] = start[1];
    this->c_[2] = start[2]/2.;

    // from initial condition at t=T.
    this->t_ = T;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T2 * T2;
    double T5 = T3 * T2;
    double p1 = c_[0] + c_[1] * T + c_[2] * T2;
    double p2 = c_[1] + 2. * c_[2] * T;
    double p3 = 2. * c_[2];


    rb::math::Matrix3 A;
    A << T3    , T4      , T5       ,
      3 * T2 , 4 * T3  , 5 * T4   ,
      6 * T  , 12 * T2 , 20 * T3;

    rb::math::VectorX B(3, 1);
    B << end[0] - p1,
      end[1] - p2,
      end[2] - p3;

    rb::math::VectorX X = A.inverse() * B;

    this->c_[3] = X[0];
    this->c_[4] = X[1];
    this->c_[5] = X[2];
  };

  /*!
   * @brief Get the value (position) of polynomial f(t) at time T
   * @param T the value of certain time.
   * @return  double
   */
  double getPosition(const double& T) const
  {
    rb::math::VectorX vecT(degree_+1);
    for(int i=0; i<vecT.size(); ++i)
    {
      vecT[i] = pow(T, i);
    }
    double position = c_.transpose() * vecT;
    return position;
  };

  /*!
   * @brief Get the 1st order differential value (velocity) of polynomial f(t)
   * at time T
   * @param T the value of a certain time.
   * @return  double
   */
  double getVelocity(const double& T) const
  {
    rb::math::VectorX vecT(degree_);
    for(int i=0; i<vecT.size(); ++i)
    {
      vecT[i] = (i+1) * pow(T, i);
    }

    double vel = c_.tail(degree_).transpose() * vecT;
    return vel;
  }

  /*!
   * @brief Get the degree (order) of the polynomial function.
   * @return short
   */
  short getDegree(void) const
  {
    return this->degree_;
  };

  /*!
   * @brief Get the coefficients of the polynomial function.
   * @return rb::math::VectorX
   */
  rb::math::VectorX getCoeff(void) const
  {
    return this->c_;
  };


protected:
  short degree_;            //!< An integer indicate degree of polynomial.
  rb::math::VectorX c_;     //!< A vector of coefficient of polynomial.
  double t_;                //!< The duration (time) of polynomial function f(t).

private:

};

}       // namespace math
}       // namespace rb
#endif  // RB_POLYNOMIAL_H_
