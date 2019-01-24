#include "kalman_filter.h"
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict()
{
  /**
  Predict the state.
  */

  x_ = F_ * x_;
  P_ = F_ * P_.transpose() * F_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
  /**
  Update the state using Kalman Filter equations.
  */

  y_ = z - H_ * x_;
  S_ = H_ * P_ * H_.transpose() + R_;
  K_ = P_ * H_.transpose() * S_.inverse();

  x_ = x_ + K_ * y_;
  P_ = (I_ - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
  Update the state by using Extended Kalman Filter equations.
  */

  float p_x = x_[0];
  float p_y = x_[1];
  float v_x = x_[2];
  float v_y = x_[3];

  float rho_sq = max(pow(p_x, 2) + pow(p_y, 2), 0.001);
  float rho    = sqrt(rho_sq);

  y_ = VectorXd(3);
  y_ << rho, atan2(p_y, p_x), (p_x*v_x + p_y*v_y) / rho;
  y_ = z - y_;

  y_[1] = fmod(y_[1] + M_PI, 2*M_PI) - M_PI;

  // Calculate Jacobian
  H_ = MatrixXd(3, 4);
  H_ << p_x/rho,                                p_y/rho,                                0,       0,
        -p_y/rho_sq,                            p_x/rho_sq,                             0,       0,
        p_y*(v_x*p_y-v_y*p_x)/pow(rho_sq, 1.5), p_x*(v_y*p_x-v_x*p_y)/pow(rho_sq, 1.5), p_x/rho, p_y/rho;

  S_ = H_ * P_ * H_.transpose() + R_;
  K_ = P_ * H_.transpose() * S_.inverse();

  x_ = x_ + K_ * y_;
  P_ = (I_ - K_ * H_) * P_;
}
