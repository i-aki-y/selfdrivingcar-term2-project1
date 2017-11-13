#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  long n = x_.size();
  MatrixXd I = MatrixXd::Identity(n, n);
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd hx = Tools::CalculatePolar(x_);

  VectorXd y = z - hx;

  // normalize phi within [-pi : pi]
  double phi = y[1];
  if(phi > M_PI){
    phi = -(2*M_PI - phi);
  }else if (phi < - M_PI){
    phi = 2*M_PI + phi;
  }
  y[1] = phi;

  MatrixXd Hjt = H_.transpose();
  MatrixXd S = H_ * P_ * Hjt + R_;
  MatrixXd K = P_ * Hjt * S.inverse();

  long n = x_.size();
  MatrixXd I = MatrixXd::Identity(n, n);
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}
