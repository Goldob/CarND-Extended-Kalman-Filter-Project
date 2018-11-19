#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  int num_measurements = estimations.size();
  VectorXd residuals(estimations[0].size());

  for (int i=0; i<num_measurements; i++) {
    residuals += estimations[i] - ground_truth[i];
  }

  return residuals / num_measurements;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  return x_state;
}
