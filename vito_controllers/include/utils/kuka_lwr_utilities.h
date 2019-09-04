
#ifndef KUKA_LWR_UTILITIES_H
#define KUKA_LWR_UTILITIES_H

#include <kdl/kdl.hpp>
#include <Eigen/Core>
#include "pseudo_inversion.h"

inline void saturateJointPositions( KDL::JntArray &q, double soft_limit = 2.0 * M_PI / 180.0 )
{

    const double deg2rad = M_PI  / 180.0;

    std::vector<double> pos_limit;
    pos_limit.push_back ( (170.0 * deg2rad) - soft_limit );
    pos_limit.push_back ( (120.0 * deg2rad) - soft_limit );
    pos_limit.push_back ( (170.0 * deg2rad) - soft_limit );
    pos_limit.push_back ( (120.0 * deg2rad) - soft_limit );
    pos_limit.push_back ( (170.0 * deg2rad) - soft_limit );
    pos_limit.push_back ( (120.0 * deg2rad) - soft_limit );
    pos_limit.push_back ( (170.0 * deg2rad) - soft_limit );



    for (unsigned i = 0; i < q.rows(); ++i) {
        if ( std::abs(q(i)) >= pos_limit[i] )
        {
            q(i) = std::copysign(pos_limit[i], q(i));
            // std::cout << "Joint Position Limit on Joint: " << i  << " set to: " << q(i) << " (rad)" << std::endl;
        }
    }

}

inline void saturateJointVelocities( KDL::JntArray &qp, double percentage = 0.7)
{

    const double deg2rad = M_PI  / 180.0;

    std::vector<double> vel_limits;
    vel_limits.push_back (110.0 * deg2rad * percentage);
    vel_limits.push_back (110.0 * deg2rad * percentage);
    vel_limits.push_back (128.0 * deg2rad * percentage);
    vel_limits.push_back (128.0 * deg2rad * percentage);
    vel_limits.push_back (204.0 * deg2rad * percentage);
    vel_limits.push_back (184.0 * deg2rad * percentage);
    vel_limits.push_back (184.0 * deg2rad * percentage);

    for (unsigned i = 0; i < qp.rows(); ++i) {
        if ( std::abs(qp(i)) >= vel_limits[i] )
        {
            qp(i) = std::copysign(vel_limits[i], qp(i));
            // std::cout << "Joint Speed Limit on Joint: " << i << " set to: " << qp(i) * (1.0 / deg2rad) << std::endl;
        }
    }

}


Eigen::Matrix<double, 3, 3> inline getVectorHat(Eigen::Matrix<double, 3, 1> vector_in)
{
    Eigen::Matrix<double, 3, 3> vector_hat = Eigen::Matrix<double, 3, 3>::Zero();

    vector_hat << 0, -vector_in(2, 0), vector_in(1, 0),
               vector_in(2, 0), 0, -vector_in(0, 0),
               -vector_in(1, 0), vector_in(0, 0), 0;
    return vector_hat;
}

Eigen::Matrix<double, 6, 6> inline getAdjointT( KDL::Frame Frame_in)
{
    Eigen::Matrix<double, 6, 6> Adjoint_local = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 3, 3> rotation_local(Frame_in.M.data);
    Eigen::Matrix<double, 3, 1> position_local(Frame_in.p.data);

    Adjoint_local.block<3, 3>(0, 0) = rotation_local.transpose();
    Adjoint_local.block<3, 3>(3, 3) = rotation_local.transpose();
    Adjoint_local.block<3, 3>(3, 0) = -rotation_local.transpose() * getVectorHat(position_local);

    return Adjoint_local;
}

double inline VelocityLimit(KDL::Twist x_dot_d, double limit = 1.0)
{
    Eigen::Matrix<double, 3, 1> x_dot_d_local = Eigen::Matrix<double, 3, 1>::Zero();
    x_dot_d_local << x_dot_d.vel.data[0], x_dot_d.vel.data[1], x_dot_d.vel.data[2];

    double den = std::sqrt(x_dot_d_local.transpose() * x_dot_d_local);
    double temp = 1.0;

    if (den != 0.0)
    {
        temp = limit / den;
    }

    return std::min(1.0, temp);
}

inline bool checkStatesforNan( KDL::JntArray &q)
{

    for (unsigned int i = 0; i < q.rows(); ++i) {
        if (std::isnan(q(i)))
        {
            return true;
        }
    }

    return false;
}


inline Eigen::Matrix<double, 7, 1> CF_JS_CentralJointAngles(KDL::JntArray q, KDL::JntArray min, KDL::JntArray max,  KDL::JntArray center, double gain = 1.0)
{
  double sum = 0;
  double temp;
  int N = q.data.size();

  Eigen::Matrix<double, 7, 1> tempret =  Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, 7, 1> weights =  Eigen::Matrix<double, 7, 1>::Zero();
  weights << 1, 1, 1, 10, 1, 1, 1;

  for (int i = 0; i < N; i++)
  {
    temp = weights(i) * ((q(i) - center(i)) / (max(i) - min(i)));
// sum += temp*temp;
    sum += temp;
  }

  sum /= 2 * N;

  for (int i = 0; i < N; i++)
  {
    tempret(i) = sum;
  }

  return -gain *   tempret;

}

inline Eigen::Matrix<double, 7, 1> CF_JS_JointLimitAvoidance(KDL::JntArray q, KDL::JntArray min, KDL::JntArray max,  KDL::JntArray center, double gain = 1.0)
{ // This function implements joint limit avoidance usung the penalty function V = \sum\limits_{i=1}^n\frac{1}{s^2} s = q_{l_1}-|q_i|
  Eigen::Matrix<double, 7, 1> tau_limit_avoidance = Eigen::Matrix<double, 7, 1>::Zero();
  double s, potential, treshold, q_abs;

  for (unsigned int i = 0; i < q.data.size(); i++)
  {
    treshold = 5.0 * M_PI / 180.0;
    q_abs = std::fabs(q.data[i]);
    s = max(i) - q_abs;


    if (  s < treshold )
    {
      ROS_WARN("Joint limit %d", i);
      potential = 0.5 * std::pow((1 / s - 1 / treshold), 2);
      tau_limit_avoidance(i) = - gain *  KDL::sign(q.data[i]) * potential;
    }
    else
    {
      tau_limit_avoidance(i) = 0.0;
    }
  }

  return tau_limit_avoidance;
}


inline Eigen::Matrix<double, 7, 1>  getRepulsiveJointVelocity( KDL::Jacobian &J, unsigned int n_joint, Eigen::Matrix<double, 6, 1> F)
{

  KDL::Jacobian J_local;
  J_local.resize(7);
  J_local.data = J.data;

  for (unsigned int i = 6; i > n_joint - 1; i--)
  {
    J_local.setColumn(i, KDL::Twist::Zero());
  }

  Eigen::MatrixXd J_pinv_n;
  pseudo_inverse(J_local.data, J_pinv_n);

  return J_pinv_n * F;
}


#endif