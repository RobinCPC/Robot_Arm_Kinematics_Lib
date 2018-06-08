/*--------------------------------------------------------------------------------
 * rakl.cpp
 * Author             Chien-Pin Chen,
 * Description        General robot arm Kinematic application
 *--------------------------------------------------------------------------------*/
#include "rakl.h"

#include <iomanip>


/*-DEFINES---------------------------------------------------------------------*/
#define R2D 57.295779513082320876798154814105   //!< constant to present converting from radius to degree
#define D2R  0.01745329251994329576923690768489 //!< constant to present  converting from degree to radius


std::ostream& RA::operator<< (std::ostream& ost, const RA::ARM_POS& pose)
{
    auto rw12 = [&ost](auto vec_inp){   // use lambda to handle
        ost << '\n';
        for(auto& inp : vec_inp)
            ost << std::right << std::setw(12) << inp;
    };

    rw12( std::vector<char>({'x', 'y', 'z'}) );
    rw12( std::vector<double>({pose.x, pose.y, pose.z}) );
    rw12( std::vector<char>({'a', 'b', 'c'}) );
    rw12( std::vector<double>({pose.a, pose.b, pose.c}) );
    return ost;
}

/* ----------- Robot Arm Kinematics Library --------------*/
rakl::rakl()
{
    /* Initialize */
    m_ini_theta = Array6d::Constant(0.0);
    m_pre_theta = Array6d::Constant(0.0);

    m_T_act = Matrix4d::Constant(0.0);
    m_tool_T = Matrix4d::Constant(0.0);

    pre_fit_solution = 9;

    // Default as KUKA KR5 ( Using Modified DH-Table)
    //    <<     j1,    j2,    j3,     j4,    j5,   j6
        a <<    0.0,  75.0, 270.0,   90.0,   0.0,   0.0;
    alpha <<  180.0,  90.0,   0.0,   90.0, -90.0, -90.0;
        d << -335.0,   0.0,   0.0, -295.0,   0.0,  80.0;
    theta <<    0.0, -90.0,   0.0,    0.0,   0.0,   0.0;

     uplimit <<  170.,   45.,  169.0-90.0,  190.,  120.,  350.; /* degree */
    lowlimit << -170., -190., -119.0-90.0, -190., -120., -350.; /* degree */

    /* Initialize private variable */
    m_tool_T = Matrix4d::Identity();

    /* Initialize work_base HT matrix */
    work_base_T = Matrix4d::Identity();

    /* Initialize theta angle of each joints */
    for (int i=0; i < theta.size(); ++i)
    {
        m_ini_theta[i] = theta[i];
    }

    //Matrix4d T1, T2, T3, T4, T5, T6;
    Matrix4d T1 = Homo_trans(a[0], alpha[0], d[0], m_ini_theta[0]); // A01
    Matrix4d T2 = Homo_trans(a[1], alpha[1], d[1], m_ini_theta[1]); // A12
    Matrix4d T3 = Homo_trans(a[2], alpha[2], d[2], m_ini_theta[2]); // A23
    Matrix4d T4 = Homo_trans(a[3], alpha[3], d[3], m_ini_theta[3]); // A34
    Matrix4d T5 = Homo_trans(a[4], alpha[4], d[4], m_ini_theta[4]); // A45
    Matrix4d T6 = Homo_trans(a[5], alpha[5], d[5], m_ini_theta[5]); // A56

    //Matrix4d T12, T13, T14, T15, T16;
    Matrix4d T12 = T1 * T2;             // A02 = A01 * A12
    Matrix4d T13 = T12 * T3;
    Matrix4d T14 = T13 * T4;
    Matrix4d T15 = T14 * T5;
    Matrix4d T16 = T15 * T6;
    m_T_act = T16 * m_tool_T;           // m_T_act is Tool center point HT Matrix
    work_base = m_T_act * work_base_T;  // get TCP in work base coordination

    /* calculate xyzabc */
    m_pos_act.x = work_base(0,3);
    m_pos_act.y = work_base(1,3);
    m_pos_act.z = work_base(2,3);
    tr2rpy(work_base, m_pos_act.a, m_pos_act.b, m_pos_act.c);
    m_pos_act.a *= R2D;                 // change radius to degree
    m_pos_act.b *= R2D;
    m_pos_act.c *= R2D;

    /* copy HTmatrix */
    m_pos_act.T[0] = T1;
    m_pos_act.T[1] = T12;
    m_pos_act.T[2] = T13;
    m_pos_act.T[3] = T14;
    m_pos_act.T[4] = T15;
    m_pos_act.T[5] = m_T_act;
    m_pos_act.T[6] = work_base;
}

rakl::rakl(
        const Array6d& a0,
        const Array6d& alpha0,
        const Array6d& d0,
        const Array6d& ini_theta,
        const Array6d& uplimit0,
        const Array6d& lowlimit0)
{
    /* Initialize */
    m_ini_theta = Array6d::Constant(0.0);
    m_pre_theta = Array6d::Constant(0.0);

    m_T_act = Matrix4d::Constant(0.0);
    m_tool_T = Matrix4d::Constant(0.0);

    pre_fit_solution = 9;

    /*initialize Modified DH-Table*/
    this->a = a0;
    this->alpha = alpha0;
    this->d = d0;
    this->theta = ini_theta;
    this->uplimit = uplimit0;
    this->lowlimit = lowlimit0;

    /* Initialize private variable */
    // TCP Home Trans Matrix
    m_tool_T = Matrix4d::Identity();

    /* Initialize work_base HT matrix */
    work_base_T = Matrix4d::Identity();

    /* Initialize theta angle of each joints */
    for (int i=0; i < theta.size(); ++i)
    {
        m_ini_theta[i] = theta[i];
    }

    //Matrix4d T1, T2, T3, T4, T5, T6;
    Matrix4d T1 = Homo_trans(a[0], alpha[0], d[0], m_ini_theta[0]); // A01
    Matrix4d T2 = Homo_trans(a[1], alpha[1], d[1], m_ini_theta[1]); // A12
    Matrix4d T3 = Homo_trans(a[2], alpha[2], d[2], m_ini_theta[2]); // A23
    Matrix4d T4 = Homo_trans(a[3], alpha[3], d[3], m_ini_theta[3]); // A34
    Matrix4d T5 = Homo_trans(a[4], alpha[4], d[4], m_ini_theta[4]); // A45
    Matrix4d T6 = Homo_trans(a[5], alpha[5], d[5], m_ini_theta[5]); // A56

    //Matrix4d T12, T13, T14, T15, T16;
    Matrix4d T12 = T1 * T2;             // A02 = A01 * A12
    Matrix4d T13 = T12 * T3;
    Matrix4d T14 = T13 * T4;
    Matrix4d T15 = T14 * T5;
    Matrix4d T16 = T15 * T6;
    m_T_act = T16 * m_tool_T;           // m_T_act is Tool center point HT Matrix
    work_base = m_T_act * work_base_T;  // get TCP in work base coordination

    /* calculate xyzabc */
    m_pos_act.x = work_base(0,3);
    m_pos_act.y = work_base(1,3);
    m_pos_act.z = work_base(2,3);
    tr2rpy(work_base, m_pos_act.a, m_pos_act.b, m_pos_act.c);
    m_pos_act.a *= R2D;                 // change radius to degree
    m_pos_act.b *= R2D;
    m_pos_act.c *= R2D;

    /* copy HTmatrix */
    m_pos_act.T[0] = T1;
    m_pos_act.T[1] = T12;
    m_pos_act.T[2] = T13;
    m_pos_act.T[3] = T14;
    m_pos_act.T[4] = T15;
    m_pos_act.T[5] = m_T_act;
    m_pos_act.T[6] = work_base;
}

/* Destructor */
rakl::~rakl(){}

/* Forward Kinematics */
RA::ARM_POS rakl::forwardKin(const Array6d& q)
{
    // storage input angles as previous angles for finding best solution in IK
    m_pre_theta = q;

    // calculate homogeneous matrix for each joint
    Eigen::Array<Matrix4d, 6, 1> T;
    for(int i=0; i < q.size(); ++i)
        T[i] = Homo_trans(a[i], alpha[i], d[i], q[i]);

    //Matrix4d T12, T13, T14, T15, T16;
    Matrix4d T12 = T[0] * T[1];             // A02 = A01 * A12
    Matrix4d T13 = T12 * T[2];
    Matrix4d T14 = T13 * T[3];
    Matrix4d T15 = T14 * T[4];
    Matrix4d T16 = T15 * T[5];
    m_T_act = T16 * m_tool_T;           // m_T_act is Tool center point HT Matrix
    work_base = m_T_act * work_base_T;  // get TCP in work base coordination

    /* calculate xyzabc */
    m_pos_act.x = work_base(0, 3);
    m_pos_act.y = work_base(1, 3);
    m_pos_act.z = work_base(2, 3);
    tr2rpy(work_base, m_pos_act.a, m_pos_act.b, m_pos_act.c);
    m_pos_act.a = R2D * m_pos_act.a;
    m_pos_act.b = R2D * m_pos_act.b;
    m_pos_act.c = R2D * m_pos_act.c;

    /* copy m_T_act */
    m_pos_act.T[0] = T[0];
    m_pos_act.T[1] = T12;
    m_pos_act.T[2] = T13;
    m_pos_act.T[3] = T14;
    m_pos_act.T[4] = T15;
    m_pos_act.T[5] = m_T_act;
    m_pos_act.T[6] = work_base;

    pre_fit_solution = 9;       // reset previous solution
    return m_pos_act;
}


RA::ARM_POS rakl::getArmPos(void)
{
    return this->m_pos_act;
}


bool rakl::setToolOffset(RA::ARM_POS tool_offset)
{
    double roll = D2R * tool_offset.a;
    double pitch = D2R * tool_offset.b;
    double yaw = D2R * tool_offset.c;

    // compute rpy matrix and assign it to offset matrix of the tool.
    rpy2tr(roll, pitch, yaw, this->m_tool_T);
    this->m_tool_T(0,3) = tool_offset.x;
    this->m_tool_T(1,3) = tool_offset.x;
    this->m_tool_T(2,3) = tool_offset.x;

    return true;
}

void rakl::setBase(Matrix4d& base)
{
    this->work_base_T = base;
    return;
}

Matrix4d rakl::getBase(void)
{
    return this->work_base_T;
}

Array6d rakl::getA(void)
{
    return this->a;
}

Array6d rakl::getAlpha(void)
{
    return this->alpha;
}

Array6d rakl::getD(void)
{
    return this->d;
}

Array6d rakl::getTheta(void)
{
    return this->theta;
}

void rakl::setUpLimit(Array6d& up_lim)
{
   this->uplimit = up_lim;
   return;
}

Array6d rakl::getUpLimit(void)
{
    return this->uplimit;
}

void rakl::setLowLimit(Array6d& low_lim)
{
   this->lowlimit = low_lim;
   return;
}

Array6d rakl::getLowLimit(void)
{
    return this->lowlimit;
}


/********************************************************************************/
/** \brief Building HT matrix
* A function to build homogeneous transformation matrix for each link.
* \return Matrix4d
*/
Matrix4d rakl::Homo_trans(double& A, double& alpha, double& D, const double& theta)
{
    double ct = cos(D2R * theta);
    double st = sin(D2R * theta);
    double ca = cos(D2R * alpha);
    double sa = sin(D2R * alpha);

    Matrix4d T;
    T << ct,   -st,   0,     A,
      st*ca, ct*ca, -sa, -sa*D,
      st*sa, ct*sa,  ca,  ca*D,
          0,     0,   0,     1;
    return T;
}


/********************************************************************************/
/** \brief Rotation Matrix to Roll Pitch Yaw
* A function to get roll, pitch, yaw from rotation matrix
* \return N/A
*/
void rakl::tr2rpy(const Matrix4d& m, double& roll_z, double& pitch_y, double& yaw_x)
{
    double eps=2.22044604925031e-5;
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
void rakl::rpy2tr(double& roll_z, double& pitch_y, double& yaw_x, Matrix4d& tool_mat)
{
    Matrix4d mat_z = rotateZ(roll_z);
    Matrix4d mat_y = rotateY(pitch_y);
    Matrix4d mat_x = rotateX(yaw_x);
    tool_mat = mat_z * mat_y * mat_x;
    return;
}


Matrix4d rakl::rotateX(const double& deg)
{
    Matrix3d m33;
    m33 = AngleAxisd(deg * D2R, Vector3d::UnitX());
    Matrix4d matrix = Matrix4d::Identity();
    matrix.topLeftCorner(3, 3) << m33;

    return matrix;
}


Matrix4d rakl::rotateY(const double& deg)
{
    Matrix3d m33;
    m33 = AngleAxisd(deg * D2R, Vector3d::UnitY());
    Matrix4d matrix = Matrix4d::Identity();
    matrix.topLeftCorner(3,3) << m33;
    return matrix;
}

Matrix4d rakl::rotateZ(const double& deg)
{
    Matrix3d m33;
    m33 = AngleAxisd(deg * D2R, Vector3d::UnitZ());
    Matrix4d matrix = Matrix4d::Identity();
    matrix.topLeftCorner(3,3) << m33;
    return matrix;
}
