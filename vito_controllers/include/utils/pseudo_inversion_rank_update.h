// Author: Enrico Corvaglia
// pseudo_inverse() computes the pseudo inverse of matrix M_ using SVD decomposition (can choose between damped and not)
// returns the pseudo inverted matrix M_pinv_

#ifndef PSEUDO_INVERSION_RANK_UPDATE_H
#define PSEUDO_INVERSION_RANK_UPDATE_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
using namespace Eigen;

inline void pseudo_inverse_rank_update(const Eigen::MatrixXd &B, Eigen::MatrixXd &M, bool damped = false)
{
  double lambda_max = damped ? 1.0e-3 : 0.0;

  JacobiSVD<MatrixXd>::SingularValuesType sing_vals_;
  JacobiSVD<MatrixXd> svd(B, ComputeFullU | ComputeFullV);
  sing_vals_ = svd.singularValues();
  MatrixXd S_ = B; // copying the dimensions of M_, its content is not needed.

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

  Eigen::MatrixXd A = MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());


  MatrixXd x;
  x = MatrixXd::Identity(B.rows(), 1);

  MatrixXd Ax = A*x;
  MatrixXd P = MatrixXd::Identity(B.rows(), B.rows()) - B*A;
  MatrixXd Px = P*x;
  double alpha = (x.transpose() * P * x)(0);
  MatrixXd b;
  if (alpha < 1e-9)
  {
    double eta = (Ax.transpose() * Ax)(0);
    b = A.transpose() * ( Ax * (1 / (1+eta)) ) ;
  }
  else
  {
    b = Px/alpha;
  }

  M = A - Ax*b.transpose();
  // M=A;

}

#endif
