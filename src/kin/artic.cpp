/*!
 * @file        artic.cpp
 * @brief       General robot arm Kinematic application
 * @author      Chien-Pin Chen,
 */
#include "artic.h"

#include <algorithm>
#include <cmath>


namespace rb
{
namespace kin
{

using rb::math::RAD2DEG;
using rb::math::DEG2RAD;
using rb::math::PI;

using rb::math::Matrix3;
using rb::math::Matrix4;
using rb::math::MatrixX;
using rb::math::Vector3;
using rb::math::Vector4;
using rb::math::VectorX;
using rb::math::AngleAxis;

/* ----------- Robot Arm Kinematics Library --------------*/
Artic::Artic()
{
  // Default as KUKA KR5 ( Using Modified DH-Table)
  //                    Modified DH-Table:     a, alpha,    d, theta,  up, down
  this->links_.push_back(new rb::kin::Link(  0.0, 180., -335.,   0., 170., -170));  // link0
  this->links_.push_back(new rb::kin::Link( 75.0,  90.,    0., -90.,  45., -190));  // link1
  this->links_.push_back(new rb::kin::Link(270.0,   0.,    0.,   0.,  79., -209));  // link2
  this->links_.push_back(new rb::kin::Link( 90.0,  90., -295.,   0., 190., -190));  // link3
  this->links_.push_back(new rb::kin::Link(  0.0, -90.,    0.,   0., 120., -120));  // link4
  this->links_.push_back(new rb::kin::Link(  0.0, -90.,   80.,   0., 350., -350));  // link5
  this->setDOF();

  a.resize(this->getDOF());
  alpha.resize(this->getDOF());
  d.resize(this->getDOF());
  theta.resize(this->getDOF());
  up_lim_.resize(this->getDOF());
  low_lim_.resize(this->getDOF());
  //      <<     j1,    j2,          j3,     j4,    j5,  j6
        a <<    0.0,  75.0,       270.0,   90.0,   0.0,   0.0;
    alpha <<  180.0,  90.0,         0.0,   90.0, -90.0, -90.0;
        d << -335.0,   0.0,         0.0, -295.0,   0.0,  80.0;
    theta <<    0.0, -90.0,         0.0,    0.0,   0.0,   0.0;

   up_lim_ <<  170.,   45.,  169.0-90.0,   190.,  120.,  350.; /* degree */
  low_lim_ << -170., -190., -119.0-90.0,  -190., -120., -350.; /* degree */

  /* Initialize rest variable */
  ini_theta_ = VectorX::Constant(this->getDOF(), 1, 0.0);
  pre_theta_ = VectorX::Constant(this->getDOF(), 1, 0.0);

  base_tcp_tf_ = Matrix4::Identity();

  pre_fit_solution_ = 9;

  /* Initialize private variable */
  tool_tf_ = Matrix4::Identity();

  /* Initialize HT matrix of robot base w.r.t world frame (coordination). */
  base_tf_ = Matrix4::Identity();

  /* Initialize theta angle of each joints */
  for (int i=0; i < theta.size(); ++i)
  {
    ini_theta_[i] = theta[i];
  }

  // Compute transformation of each joint with base and tcp frame
  std::vector<Matrix4> trsf06(this->dof_);
  trsf06[0] = this->links_[0]->tf;
  for(int i=1; i < this->dof_; ++i)
  {
    trsf06[i] = trsf06[i-1] * (*this->links_[i]);   // T01* T12 * T23 ... * T56
  }

  base_tcp_tf_ = trsf06.back() * tool_tf_;    // base_tcp_tf_ is Tool center point HT Matrix w.r.t robot base
  world_tcp_tf_ = base_tf_ * base_tcp_tf_;    // get TCP in work base coordination

  /* calculate xyzabc */
  tcp_pose_.x = world_tcp_tf_(0,3);
  tcp_pose_.y = world_tcp_tf_(1,3);
  tcp_pose_.z = world_tcp_tf_(2,3);
  tr2rpy(world_tcp_tf_, tcp_pose_.a, tcp_pose_.b, tcp_pose_.c);
  tcp_pose_.a *= RAD2DEG;                     // change radian to degree
  tcp_pose_.b *= RAD2DEG;
  tcp_pose_.c *= RAD2DEG;

  /* Initialize frames and copy HTmatrix of all joints and tcp */
  this->frames_.resize(this->getDOF()+1);
  //this->frames_ = std::vector<rb::math::Matrix4*, Eigen::aligned_allocator<rb::math::Matrix4*> >(7);
  for(int i=0; i < this->getDOF(); ++i)
  {
    this->frames_[i] = new Matrix4(this->base_tf_ * trsf06[i]);
  }
  this->frames_.back() = new Matrix4(world_tcp_tf_);
}

Artic::Artic(
    const rb::math::VectorX& a0,
    const rb::math::VectorX& alpha0,
    const rb::math::VectorX& d0,
    const rb::math::VectorX& ini_theta,
    const rb::math::VectorX& uplimit0,
    const rb::math::VectorX& lowlimit0)
{
  // check if size of all paremeter are the smae
  std::vector<VectorX> dh_list = {a0, alpha0, d0, ini_theta, uplimit0, lowlimit0};
  bool is_match = std::all_of(dh_list.begin(), dh_list.end(), [](VectorX& d)
          { return d.size() == 6;});
  if (!is_match)
  {
    assert("Parameters size not match!\n");
    return;
  }

  /*initialize Modified DH-Table*/
  for (int i=0; i < a0.size(); ++i)
  {
    this->links_.push_back(new Link(a0[i], alpha0[i], d0[i], ini_theta[i],
                                    uplimit0[i], lowlimit0[i]));
  }
  this->setDOF();

  this->a = a0;
  this->alpha = alpha0;
  this->d = d0;
  this->theta = ini_theta;
  this->up_lim_ = uplimit0;
  this->low_lim_ = lowlimit0;

  /* Initialize the rest variable */
  ini_theta_ = VectorX::Constant(a0.size(), 1, 0.);
  pre_theta_ = VectorX::Constant(a0.size(), 1, 0.);

  base_tcp_tf_ = Matrix4::Identity();

  pre_fit_solution_ = 9;

  /* Initialize private variable */
  // TCP Home Trans Matrix
  tool_tf_ = Matrix4::Identity();

  /* Initialize HT matrix of base w.r.t world frame (coordination). */
  base_tf_ = Matrix4::Identity();

  /* Initialize theta angle of each joints */
  for (int i=0; i < theta.size(); ++i)
  {
    ini_theta_[i] = theta[i];
  }

  // Compute transformation of each joint with base and tcp frame
  std::vector<Matrix4> trsf06(this->dof_);
  trsf06[0] = this->links_[0]->tf;
  for(int i=1; i < this->dof_; ++i)
  {
    trsf06[i] = trsf06[i-1] * (*this->links_[i]);   // T01* T12 * T23 ... * T56
  }

  base_tcp_tf_ = trsf06.back() * tool_tf_;    // base_tcp_tf_ is Tool center point HT Matrix
  world_tcp_tf_ = base_tf_ * base_tcp_tf_;    // get TCP in work base coordination

  /* calculate xyzabc */
  tcp_pose_.x = world_tcp_tf_(0,3);
  tcp_pose_.y = world_tcp_tf_(1,3);
  tcp_pose_.z = world_tcp_tf_(2,3);
  tr2rpy(world_tcp_tf_, tcp_pose_.a, tcp_pose_.b, tcp_pose_.c);
  tcp_pose_.a *= RAD2DEG;                   // change radian to degree
  tcp_pose_.b *= RAD2DEG;
  tcp_pose_.c *= RAD2DEG;

  /* Initialize frames and copy HT matrix of all joints and tcp */
  this->frames_.resize(this->getDOF()+1);
  for(int i=0; i < this->getDOF(); ++i)
  {
    this->frames_[i] = new Matrix4(this->base_tf_ * trsf06[i]);
  }
  this->frames_.back() = new Matrix4(world_tcp_tf_);
}

/* Destructor */
Artic::~Artic(){}

/* Forward Kinematics */
ArmPose Artic::forwardKin(const rb::math::VectorX& q, const bool update)
{
  // Compute transformation of each joint with base and tcp frame
  std::vector<Matrix4> trsf06(this->getDOF());
  trsf06[0] = this->links_[0]->computeTransform(q[0], update);
  for(int i=1; i < q.size(); ++i)
  {
    trsf06[i] = trsf06[i-1] * this->links_[i]->computeTransform(q[i], update);
  }

  base_tcp_tf_ = trsf06.back() * tool_tf_;     // base_tcp_tf_ is Tool center point HT Matrix
  world_tcp_tf_ = base_tf_ * base_tcp_tf_;  // get TCP in work base coordination

  /* calculate xyzabc */
  tcp_pose_.x = world_tcp_tf_(0, 3);
  tcp_pose_.y = world_tcp_tf_(1, 3);
  tcp_pose_.z = world_tcp_tf_(2, 3);
  tr2rpy(world_tcp_tf_, tcp_pose_.a, tcp_pose_.b, tcp_pose_.c);
  tcp_pose_.a = RAD2DEG * tcp_pose_.a;
  tcp_pose_.b = RAD2DEG * tcp_pose_.b;
  tcp_pose_.c = RAD2DEG * tcp_pose_.c;

  /* update HT matrix of all joints and tcp */
  if (update)
  {
    // storage input angles as previous angles for finding best solution in IK
    pre_theta_ = q;
    pre_fit_solution_ = 9;       // reset previous solution

    for(int i=0; i < this->getDOF(); ++i)
    {
      *(this->frames_[i]) = this->base_tf_ * trsf06[i];
    }
    *(this->frames_.back()) = world_tcp_tf_;
  }

  return tcp_pose_;
}

/* Inverse Kinematics */
IK_RESULT Artic::inverseKin(const rb::math::Matrix4& world_tcp_tf,
    rb::math::VectorX& joints, ArmAxisValue& all_sols)
{
  // reset all possible (8) solutions
  all_sols.axis_value = MatrixX::Constant(8, 6, 0.0);
  size_t num_sols = sizeof(all_sols.limit_check) / sizeof(all_sols.limit_check[0]);
  for(size_t i=0; i < num_sols; ++i)
  {
    all_sols.solution_check[i] = true;
    all_sols.singular_check[i] = true;
    all_sols.limit_check[i] = true;
  }

  // translate work base coordination system to robot coordination.
  Matrix4 world_base_tf_inv = this->base_tf_.inverse();
  Matrix4 base_tcp_tf = world_base_tf_inv * world_tcp_tf;
  // get HT matrix of the robot arm flange
  Matrix4 tool_tf_inv = this->tool_tf_.inverse();
  Matrix4 base_flange_tf = base_tcp_tf * tool_tf_inv;
#ifndef NDEBUG
  std::cout << "\nworld_tcp_tf:\n" << world_tcp_tf;
  std::cout << "\nbase_tf_:\n" << this->base_tf_;
  std::cout << "\nworld_base_tf_inv:\n" << world_base_tf_inv;
  std::cout << "\nbase_tcp_tf:\n" << base_tcp_tf;
#endif

  /* Start to solve IK, get wrist center point Wc (or P0 = A06 * P6
   * from eqn. 2.77 of J.J. Craig's book),  and solve 1st joint. */
  Eigen::Vector4d p0;
  Eigen::Vector4d p6 = {0., 0., -this->d[5], 1.};
  p0 = base_flange_tf * p6;
  // solve joint 1st of solution 1-4
  double theta1 = atan2(-p0[1], p0[0]);
  for(int i=0; i < 4; ++i)
  {
    all_sols.axis_value(i, 0) = theta1;
  }

  /* Solve joint 2nd, 3rd of solutions 1-2, 3-4 */
  std::vector<bool> config = {0, 0, 0};
  solvePitchPitchIK(theta1, p0, config, all_sols);

  if(all_sols.solution_check[0] == true)
  {
    /*solve joint 5,4,6  solution 1-2*/
    solveRowPitchRowIK(theta1, config, base_flange_tf, all_sols);
    /*solve theta 5,4,6  solution 3-4*/
    config = {0, 1, 0};
    solveRowPitchRowIK(theta1, config, base_flange_tf, all_sols);
  }

  /*solve joint 1st backward solution 5-8*/
  config = {1, 0, 0};
  double theta1_b = theta1 + PI;
  // precheck 1st joint value
  preCheck(1, theta1_b);
  for(int i=4; i<8; ++i)
  {
    all_sols.axis_value(i, 0) = theta1_b;
  }

  /*solve joint 2nd 3rd of  solution 5-6, 7-8*/
  solvePitchPitchIK(theta1_b, p0, config, all_sols);

  if(all_sols.solution_check[4] == true)
  {
    /*solve joint 5,4,6  solution 5-6*/
    solveRowPitchRowIK(theta1_b, config, base_flange_tf, all_sols);
    /*solve theta 5,4,6  solution 7-8*/
    config = {1, 1, 0};
    solveRowPitchRowIK(theta1_b, config, base_flange_tf, all_sols);
  }

  // Convert to degree unit before find the most fit solution.
  all_sols.axis_value *= RAD2DEG;
  IK_RESULT check = solutionCheck(all_sols);

  if( check != IK_RESULT::IK_COMPLETE)
  {
    // IK fail. return IK result, but not update joint values
#ifndef NDEBUG
    std::cout << "No Solution!!\n";
#endif
    return check;
  }

  // update the pose of robot arm
  this->tcp_pose_.x = world_tcp_tf(0, 3);
  this->tcp_pose_.y = world_tcp_tf(1, 3);
  this->tcp_pose_.z = world_tcp_tf(2, 3);
  tr2rpy(world_tcp_tf,
      this->tcp_pose_.a,
      this->tcp_pose_.b,
      this->tcp_pose_.c);

  // update pre_theta_ to the most fit solution.
  this->pre_theta_ = all_sols.axis_value.row(all_sols.fit);
  joints = all_sols.axis_value.row(all_sols.fit);

#ifndef NDEBUG
  std::cout << "\np0:\n" << p0 << '\n';
  std::cout << "\n axis_value:\n" << all_sols.axis_value.transpose() << '\n';
  std::cout << "\n The most fit solution: " << all_sols.fit << '\n';
#endif

  return check;
}

IK_RESULT Artic::inverseKin(const double& x, const double& y, const double& z,
    const double& roll, const double& pitch, const double& yaw,
    rb::math::VectorX& joints, ArmAxisValue& all_sols)
{
  // reset all possible (8) solutions
  all_sols.axis_value = MatrixX::Constant(8, 6, 0.0);
  size_t num_sols = sizeof(all_sols.limit_check) / sizeof(all_sols.limit_check[0]);
  for(size_t i=0; i < num_sols; ++i)
  {
    all_sols.solution_check[i] = true;
    all_sols.singular_check[i] = true;
    all_sols.limit_check[i] = true;
  }

  // calculate Transformation matrix of Tool Center Point (TCP)  with
  // respect to world (work base) coordination system.
  Matrix4 world_tcp_tf =  Matrix4::Identity();
  double r_deg = roll;
  double p_deg = pitch;
  double y_deg = yaw;
  rpy2tr(r_deg, p_deg, y_deg, world_tcp_tf);
  world_tcp_tf(0, 3) = x;
  world_tcp_tf(1, 3) = y;
  world_tcp_tf(2, 3) = z;

  // translate work base coordination system to robot coordination.
  Matrix4 world_base_tf_inv = this->base_tf_.inverse();
  Matrix4 base_tcp_tf = world_base_tf_inv * world_tcp_tf;
#ifndef NDEBUG
  std::cout << "\nworld_tcp_tf:\n" << world_tcp_tf;
  std::cout << "\nbase_tf_:\n" << this->base_tf_;
  std::cout << "\nworld_base_tf_inv:\n" << world_base_tf_inv;
  std::cout << "\nbase_tcp_tf:\n" << base_tcp_tf;
#endif

  // get HT matrix of the robot arm flange
  Matrix4 tool_tf_inv = this->tool_tf_.inverse();
  Matrix4 base_flange_tf = base_tcp_tf * tool_tf_inv;

#ifndef NDEBUG
  printf("\nbase_flange_tf:\n");
  printf("[%.4f %.4f %.4f %.4f]\n",base_flange_tf(0, 0),base_flange_tf(0, 1),base_flange_tf(0, 2),base_flange_tf(0, 3));
  printf("[%.4f %.4f %.4f %.4f]\n",base_flange_tf(1, 0),base_flange_tf(1, 1),base_flange_tf(1, 2),base_flange_tf(1, 3));
  printf("[%.4f %.4f %.4f %.4f]\n",base_flange_tf(2, 0),base_flange_tf(2, 1),base_flange_tf(2, 2),base_flange_tf(2, 3));
  printf("[%.4f %.4f %.4f %.4f]\n",base_flange_tf(3, 0),base_flange_tf(3, 1),base_flange_tf(3, 2),base_flange_tf(3, 3));
#endif

  /* Start to solve IK, get wrist center point Wc (or P0 = A06 * P6
   * from eqn. 2.77 of J.J. Craig's book),  and solve 1st joint. */
  Eigen::Vector4d p0;
  Eigen::Vector4d p6 = {0., 0., -this->d[5], 1.};
  p0 = base_flange_tf * p6;
  // solve joint 1st of solution 1-4
  //all_sols.axis_value.block(0, 0, 4, 1).setConstant(RAD2DEG*atan2(-p0[1], p0[0]));
  double theta1 = atan2(-p0[1], p0[0]);
  for(int i=0; i < 4; ++i)
    all_sols.axis_value(i, 0) = theta1;

  /* Solve joint 2nd, 3rd of solutions 1-2, 3-4 */
  std::vector<bool> config = {0, 0, 0};
  solvePitchPitchIK(theta1, p0, config, all_sols);

  if(all_sols.solution_check[0] == true)
  {
    /*solve joint 5,4,6  solution 1-2*/
    solveRowPitchRowIK(theta1, config, base_flange_tf, all_sols);

    /*solve theta 5,4,6  solution 3-4*/
    config = {0, 1, 0};
    solveRowPitchRowIK(theta1, config, base_flange_tf, all_sols);
  }

  /*solve joint 1st backward solution 5-8*/
  config = {1, 0, 0};
  double theta1_b = theta1 + PI;
  // precheck 1st joint value
  preCheck(1, theta1_b);
  //if(theta1_b >= 2*PI)
  //    theta1_b = theta1_b - 2*PI;

  for(int i=4; i<8; ++i)
    all_sols.axis_value(i, 0) = theta1_b;

  /*solve joint 2nd 3rd of  solution 5-6, 7-8*/
  solvePitchPitchIK(theta1_b, p0, config, all_sols);

  if(all_sols.solution_check[4] == true)
  {
    /*solve joint 5,4,6  solution 5-6*/
    solveRowPitchRowIK(theta1_b, config, base_flange_tf, all_sols);

    /*solve theta 5,4,6  solution 7-8*/
    config = {1, 1, 0};
    solveRowPitchRowIK(theta1_b, config, base_flange_tf, all_sols);
  }

  // Convert to degree unit before find the most fit solution.
  all_sols.axis_value *= RAD2DEG;
  IK_RESULT check = solutionCheck(all_sols);

  if( check != IK_RESULT::IK_COMPLETE)
  {
    // IK fail. return IK result, but not update joint values
#ifndef NDEBUG
    std::cout << "No Solution!!\n";
#endif
    return check;
  }

  // update the pose of robot arm
  this->tcp_pose_.x = x;
  this->tcp_pose_.y = y;
  this->tcp_pose_.z = z;
  this->tcp_pose_.a = roll;
  this->tcp_pose_.b = pitch;
  this->tcp_pose_.c = yaw;

  // update pre_theta_ to the most fit solution.
  this->pre_theta_ = all_sols.axis_value.row(all_sols.fit);
  joints = all_sols.axis_value.row(all_sols.fit);

#ifndef NDEBUG
  std::cout << "\np0:\n" << p0 << '\n';
  std::cout << "\n axis_value:\n" << all_sols.axis_value.transpose() << '\n';
  std::cout << "\n The most fit solution: " << all_sols.fit << '\n';
#endif

  return check;
}

void Artic::solvePitchPitchIK(const double& th1, const rb::math::Vector4& p0,
    const std::vector<bool>& config,
    ArmAxisValue& all_sols)
{
  double a1 = this->a[1], a2 = this->a[2], a3 = this->a[3];
  double d0 = this->d[0], d3 = this->d[3];
  double k1 = 2 * a2 * d3;
  double k2 = 2 * a2 * a3;
  double k3 = pow(p0[0], 2) + pow(p0[1], 2) + pow((p0[2] + d0), 2) - 2*a1*p0[0] * cos(th1) +
    2*a1*p0[1]*sin(th1) + pow(a1, 2) - pow(a2, 2) - pow(a3, 2) - pow(d3, 2);
  double ks = pow(k1, 2) + pow(k2, 2) - pow(k3, 2);

  // check if has real solution
  if (ks < 0)
  {
#ifndef NDEBUG
    printf("k1^2+k2^2-k3^2 < 0, solution %d-%d no real solution\n", config[0]*4+1, config[0]*4+4);
#endif
    for (int i = 0 + config[0]*4 ; i < 4 + config[0]*4; ++i)
      all_sols.solution_check[i] = false;
  }else // has real solutions.
  {
    double th3_12 = 2 * atan2(k1-sqrt(ks), k3+k2);      // upper arm
    double th3_34 = 2 * atan2(k1+sqrt(ks), k3+k2);      // lower arm

    // check if need to remap joint value to suitable range
    preCheck(3, th3_12);
    preCheck(3, th3_34);

    all_sols.axis_value(config[0]*4 + 0, 2) = th3_12;
    all_sols.axis_value(config[0]*4 + 1, 2) = th3_12;
    all_sols.axis_value(config[0]*4 + 2, 2) = th3_34;
    all_sols.axis_value(config[0]*4 + 3, 2) = th3_34;

    double u1 = a2 + a3*cos(th3_12) + d3*sin(th3_12);
    double v1 = -a3*sin(th3_12) + d3*cos(th3_12);
    double r1 = p0[0]*cos(th1) - p0[1]*sin(th1) - a1;
    double u2 = a3*sin(th3_12) - d3*cos(th3_12);
    double v2 = u1;
    double r2 = -d0 - p0[2];

    double sin2 = (r1/u1 - r2/u2)/(v1/u1 - v2/u2);
    double cos2 = (r1/v1 - r2/v2)/(u1/v1 - u2/v2);

    double th2_12 = atan2(sin2,cos2);
    preCheck(2, th2_12);
    all_sols.axis_value(config[0]*4 + 0, 1) = th2_12;
    all_sols.axis_value(config[0]*4 + 1, 1) = th2_12;

    u1 = a2 + a3*cos(th3_34) + d3*sin(th3_34);
    v1 = -a3*sin(th3_34) + d3*cos(th3_34);
    r1 = p0[0]*cos(th1) - p0[1]*sin(th1) - a1;
    u2 = a3*sin(th3_34) - d3*cos(th3_34);
    v2 = u1;
    r2 = -d0 - p0[2];

    sin2 = (r1/u1 - r2/u2)/(v1/u1 - v2/u2);
    cos2 = (r1/v1 - r2/v2)/(u1/v1 - u2/v2);

    double th2_34 = atan2(sin2,cos2);
    preCheck(2, th2_34);
    all_sols.axis_value(config[0]*4 + 2, 1) = th2_34;
    all_sols.axis_value(config[0]*4 + 3, 1) = th2_34;
  }

  return;
}

void Artic::solveRowPitchRowIK(const double& th1, const std::vector<bool>& config,
    const rb::math::Matrix4& flange_tr,
    ArmAxisValue& all_sols)
{
  double config_start = config[0]*4 + config[1]*2;
  double th2up_rad = all_sols.axis_value( config_start, 1);
  double th3up_rad = all_sols.axis_value( config_start, 2);
  Matrix4 tr01 = homoTrans(a[0], alpha[0], d[0], th1 * RAD2DEG);
  Matrix4 tr12 = homoTrans(a[1], alpha[1], d[1], th2up_rad * RAD2DEG);
  Matrix4 tr23 = homoTrans(a[2], alpha[2], d[2], th3up_rad * RAD2DEG);
  Matrix4 tr03 = tr01 * tr12 * tr23;
  //Matrix4 tr03_inv =tr03.inverse();
  Matrix4 tr36 = tr03.inverse() * flange_tr;
  double eps = rb::math::EPSILON;	// to check if close to zero
  double th5_1, th5_2, th4_1, th4_2, th6_1, th6_2;

  // check if in singular point first
  if(fabs(tr36(0,2)) < eps && fabs(tr36(2, 2)) < eps)
  {
#ifndef NDEBUG
    printf("Solution 1-2 Reach sigular position\n");
#endif
    for(int i = config_start; i<config_start+2; ++i)
      all_sols.singular_check[i] = false;

    th5_1 = th5_2 = 0;
    double sum_theta = atan2(tr36(0, 1), tr36(0, 0));
    // TODO: may use pre_theta
    th4_1 = th4_2 = sum_theta;
    th6_1 = th6_2 = 0;
  }else
  {
    //solution 1 of theta 5,4,6
    double c4, s4, c6, s6;

    th5_1 = acos(tr36(1, 2));
    c4 = -tr36(0, 2) / sin(th5_1);
    s4 = -tr36(2, 2) / sin(th5_1);
    th4_1 = atan2(s4, c4);
    preCheck(4, th4_1);

    c6 = tr36(1, 0) / sin(th5_1);
    s6 = -tr36(1, 1) / sin(th5_1);
    th6_1 = atan2(s6,c6);
    preCheck(6, th6_1);

    //solution 2 of theta 5,4,6
    th5_2 = -th5_1;
    c4 = -tr36(0, 2) / sin(th5_2);
    s4 = -tr36(2, 2) / sin(th5_2);
    th4_2 = atan2(s4, c4);
    preCheck(4, th4_2);

    c6 = tr36(1, 0) / sin(th5_2);
    s6 = -tr36(1, 1) / sin(th5_2);
    th6_2 = atan2(s6,c6);
    preCheck(6, th6_2);
  }

  all_sols.axis_value(config_start, 3) = th4_1;
  all_sols.axis_value(config_start+1, 3) = th4_2;

  all_sols.axis_value(config_start, 4) = th5_1;
  all_sols.axis_value(config_start+1, 4) = th5_2;

  all_sols.axis_value(config_start, 5) = th6_1;
  all_sols.axis_value(config_start+1, 5) = th6_2;

  return;
}

IK_RESULT Artic::solutionCheck(ArmAxisValue& sols)
{
  // First initial result as complete (find a fit solution
  IK_RESULT check = IK_RESULT::IK_COMPLETE;
  if( std::all_of(sols.solution_check.begin(), sols.solution_check.end(),
        [](bool i){return i==false;} ))
  {
    check = IK_RESULT::IK_NO_SOLUTION;
    return check;
  }

  /* check joint limit first*/
  int n_limit = 0;
  for (int i = 0; i < 8; ++i)
  {
    if (sols.solution_check[i] == true)
    {
      for(int j = 0; j < 6; ++j)
      {
        if (sols.axis_value(i, j)  > up_lim_[j] ||
            sols.axis_value(i, j) < low_lim_[j])
        {
          sols.limit_check[i] = false;
#ifndef NDEBUG
          printf ("Solution %d, Joint %d overlimit.\n", i+1, j+1 );
#endif
        }
      }
      if (sols.limit_check[i] == false)
        n_limit = n_limit + 1;
    }
  }

  bool check_front4 = std::all_of(sols.solution_check.begin(),
      sols.solution_check.begin()+4, [](bool i){return i==false;});
  bool check_back4 = std::all_of(sols.solution_check.begin()+4,
      sols.solution_check.end(), [](bool i){return i==false;});
  // TODO: add functions to check if reach joint limits.

  // using cosine similarity to check which solutions is most closest to previous step join value
  sols.fit = 0;
  std::array<double, 8> cosine_sim = {{-2.}};   // initialize smaller valus;
  for(int i=0; i < sols.axis_value.rows(); ++i)
  {
    if(sols.solution_check[i] == true && sols.limit_check[i] == true)
    {
      Eigen::VectorXd sol_theta = sols.axis_value.row(i);
      Eigen::VectorXd pre_theta = pre_theta_;
      cosine_sim[i] = sol_theta.dot(pre_theta) /
        (sol_theta.norm() * pre_theta.norm());

    }
  }
  auto iter_cosine = std::max_element(cosine_sim.begin(),
      cosine_sim.end());
  if(iter_cosine != cosine_sim.end())
    sols.fit = iter_cosine - cosine_sim.begin();
  else
  {
#ifndef NDEBUG
    printf("Note Cosine Similarity found no_solution!\n");
#endif
    check = IK_RESULT::IK_NO_SOLUTION;
  }

  // check if it's a singular solution.
  if(sols.solution_check[sols.fit] == false)
  {
    check = IK_RESULT::IK_SINGULAR;
    sols.fit = 0;
    pre_fit_solution_ = 9;    // set for no previous solutions
    return check;
  }

  return check;
}

void Artic::preCheck(const int& njoint, double& rad)
{
  int idx = njoint - 1;
  double pre_theta = this->pre_theta_[idx];
  double deg = rad * RAD2DEG;
  std::array<double, 3> poss_rad = {{rad, rad + 2*PI, rad - 2*PI}};
  std::array<double, 3> poss_deg = {{deg, deg + 360., deg - 360.}};

  for(size_t i=1; i < poss_deg.size(); ++i)
  {
    bool in_range = poss_deg[i] <= this->up_lim_[idx] &&
      poss_deg[i] >= this->low_lim_[idx];
    bool is_closer = pow(poss_deg[i] - pre_theta, 2) <=
      pow(poss_deg[0] - pre_theta, 2);
    if( in_range && is_closer)
    {
      rad = poss_rad[i];
    }
  }
  return;
}

ArmPose Artic::getArmPose(void) const
{
  return this->tcp_pose_;
}

rb::math::VectorX Artic::getA(void) const
{
  return this->a;
}

rb::math::VectorX Artic::getAlpha(void) const
{
  return this->alpha;
}

rb::math::VectorX Artic::getD(void) const
{
  return this->d;
}

rb::math::VectorX Artic::getTheta(void) const
{
  return this->theta;
}

void Artic::setUpLimit(rb::math::VectorX& up_lim)
{
  this->up_lim_ = up_lim;
  return;
}

rb::math::VectorX Artic::getUpLimit(void) const
{
  return this->up_lim_;
}

void Artic::setLowLimit(rb::math::VectorX& low_lim)
{
  this->low_lim_ = low_lim;
  return;
}

rb::math::VectorX Artic::getLowLimit(void) const
{
  return this->low_lim_;
}

/********************************************************************************/
/** \brief Rotation Matrix to Roll Pitch Yaw
 * A function to get roll, pitch, yaw from rotation matrix
 * \return N/A
 */
void Artic::tr2rpy(const Matrix4& m, double& roll_z, double& pitch_y, double& yaw_x)
{
  double eps = rb::math::EPSILON;     // to check if close to zero
  if(fabs(m(0,0)) < eps && fabs(m(1,0)) < eps)
  {
    roll_z  = 0;
    pitch_y = atan2(-m(2,0), m(0,0));
    yaw_x   = atan2(-m(1,2), m(1,1));
  }
  else
  {
    roll_z  = atan2(m(1,0), m(0,0));
    double sr = sin(roll_z);
    double cr = cos(roll_z);
    pitch_y = atan2(-m(2,0), cr * m(0,0) + sr * m(1,0));
    yaw_x   = atan2(sr * m(0,2) - cr * m(1,2), cr * m(1,1) - sr * m(0,1));
  }
  return;
}


/********************************************************************************/
/** \brief Roll Pitch Yaw to Rotation Matrix
 * A function to get roll, pitch, yaw from rotation matrix
 * \return N/A
 */
void Artic::rpy2tr(double& roll_z, double& pitch_y, double& yaw_x, Matrix4& tool_mat)
{
  Matrix4 mat_z = rotateZ(roll_z);
  Matrix4 mat_y = rotateY(pitch_y);
  Matrix4 mat_x = rotateX(yaw_x);
  tool_mat = mat_z * mat_y * mat_x;
  return;
}


Matrix4 Artic::rotateX(const double& deg)
{
  Matrix3 m33;
  m33 = AngleAxis(deg * DEG2RAD, Vector3::UnitX());
  Matrix4 matrix = Matrix4::Identity();
  matrix.topLeftCorner(3, 3) << m33;

  return matrix;
}


Matrix4 Artic::rotateY(const double& deg)
{
  Matrix3 m33;
  m33 = AngleAxis(deg * DEG2RAD, Vector3::UnitY());
  Matrix4 matrix = Matrix4::Identity();
  matrix.topLeftCorner(3,3) << m33;
  return matrix;
}

Matrix4 Artic::rotateZ(const double& deg)
{
  Matrix3 m33;
  m33 = AngleAxis(deg * DEG2RAD, Vector3::UnitZ());
  Matrix4 matrix = Matrix4::Identity();
  matrix.topLeftCorner(3,3) << m33;
  return matrix;
}

}   // namespace kin
}   // namespace rb
