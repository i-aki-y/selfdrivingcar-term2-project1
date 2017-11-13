#include <iostream>
#include "tools.h"

using Eigen::ArrayXd;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  size_t est_size = estimations.size();
  size_t gt_size = ground_truth.size();
  if( (est_size == 0) || (gt_size == 0)) {
    cout << "size of estimation and ground truth must be larger than 0" << endl;
    return rmse;
  }

  if(est_size != gt_size){
    cout << "sizes of estimations and ground_truth should be same" << endl;
    return rmse;
  }

  //accumulate squared residuals
  ArrayXd accum = ArrayXd(4);
  accum << 0, 0, 0, 0;
  for(int i=0; i < estimations.size(); ++i){
    VectorXd delta = ground_truth[i] - estimations[i];
    accum += delta.array() * delta.array();
  }

  //calculate the mean
  VectorXd mean = accum/estimations.size();

  //calculate the squared root
  rmse = mean.array().sqrt().matrix();

  //return the result
  return rmse;



}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj(3,4);
  Hj << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

  float eps = 0.000001;

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float rho = sqrt(px * px + py * py);
  float rho2 = px * px + py * py;
  float rho3 = rho * rho2;

  if(rho2 < eps){
    cout << "zero division;" << endl;
    // use default Hj.
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << px/rho, py/rho, 0, 0,
          -py/rho2, px/rho2, 0, 0,
          py*(vx*py - vy*px)/rho3, px*(vy*px - vx*py)/rho3, px/rho, py/rho;
  return Hj;
}


VectorXd Tools::CalculatePolar(const VectorXd &x_state) {
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float rho = sqrt(px * px + py * py);
  float phi = atan2(py, px);
  float rho_dot = (px * vx + py * vy) / rho;

  VectorXd hx = VectorXd(3);
  hx << rho, phi, rho_dot;

  return hx;
}
