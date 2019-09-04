// Author: Enrico Corvaglia
// pseudo_inverse() computes the pseudo inverse of matrix M_ using SVD decomposition (can choose between damped and not)
// returns the pseudo inverted matrix M_pinv_

#ifndef PSEUDO_INVERSION_H
#define PSEUDO_INVERSION_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
using namespace Eigen;

inline void pseudo_inverse(const Eigen::MatrixXd &M_, Eigen::MatrixXd &M_pinv_, bool damped = true)
{
  double lambda_max = damped ? 1.0e-3 : 0.0;

  JacobiSVD<MatrixXd>::SingularValuesType sing_vals_;
  JacobiSVD<MatrixXd> svd(M_, ComputeFullU | ComputeFullV);
  sing_vals_ = svd.singularValues();
  MatrixXd S_ = M_; // copying the dimensions of M_, its content is not needed.

  double lambda_quad = 0;
  double epsilon = 1e-4; //

  if ( sing_vals_(sing_vals_.size()-1) < epsilon )
  {
    lambda_quad = ( 1 - std::pow( (sing_vals_(sing_vals_.size()-1) / epsilon), 2) ) * std::pow(lambda_max, 2);
  }

  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
  {
    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_quad);
  }

  M_pinv_ = MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

#endif
