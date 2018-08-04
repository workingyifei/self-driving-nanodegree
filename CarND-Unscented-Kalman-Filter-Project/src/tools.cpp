#include <iostream>
#include "tools.h"

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
  // define a 4x4 vector and initialize with 0
  VectorXd rmse(4)
  rmse << 0, 0, 0, 0;

  // check the validity of the estimations
  if(estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    std::cout << "resulted estimations are invalid!" << std::endl;
    return rmse;
  }

  // calculate accumulate squared error
  for(unsigned int i=0; i<estimations.size(); i++) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse /= estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;

}