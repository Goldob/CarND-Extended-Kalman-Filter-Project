#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

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
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
