#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  // set state dimension
  n_x_ = 5;

  // set augmented state dimension
  n_aug_ = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  n_z_laser_ = 2;

  //set measurement dimension, radar can measure r, phi, and r_dot
  n_z_radar_ = 3;

  //define design or spreading parameter lambda
  lambda_ = 3 - n_x_;

  //weights for calculating predicted mean and covariance
  weights = VectorXd(2 * n_aug_ + 1);
  weights(0) = lambda_ / (lambda_ + n_aug_);
  for (int i=1; i<2 * n_aug_ + 1; i++) {
    weights(i) = 0.5/(n_aug_+lambda_);
  }

  //create sigma point matrix
  MatrixXd Xsig_ = MatrixXd(n_x_, 2 * n_x_ + 1);

  //create augmented sigma point matrix
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_+ 1);

  //create predicted sigma points matrix
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  MatrixXd H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
          0, 1, 0, 0;

  //no need to define H_radar,
  //since radar measurement function is non-linear,
  // we use unscented transformation (approxiamted by sigma points),
  // to generate predicted radar measurement

  MatrixXd R_laser_ = MatrixXd(n_z_laser_, n_z_laser_);
  R_laser_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

  MatrixXd R_radar_ = MatrixXd(n_z_radar_,n_z_radar_);
  R_radar_ << std_radr_*std_radr_, 0, 0,
             0, std_radphi_*std_radphi_, 0,
             0, 0,std_radrd_*std_radrd_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * init state vector x_ and covariance P
   */
  if (!is_initialized_) {

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
//      float rho_dot = meas_package.raw_measurements_[2];

      double px = rho * cos(phi);
      double py = rho * sin(phi);
      x_ << px, py, 0, 0, 0;
    }

    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }

    time_us_ = meas_package.timestamp_;

    // needs to be further tuned to get a working solution
    P_ = MatrixXd::Identity(5, 5);

    is_initialized_ = true;
  }

  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);

  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  }
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  //Generate sigma points
  GenerateSigmaPoints();

  //Create augmented mean state, augmented covariance matrix and augmented sigma points
  AugmentedSigmaPoints();

  //predict sigma points
  SigmaPointPrediction(delta_t);

  //predict mean and covariance matrix
  PredictMeanAndCovariance();
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  // incoming lidar measurement
  VectorXd z_ = meas_package.raw_measurements_;

  VectorXd z_pred = H_laser_ * x_;
  VectorXd y = z_ - z_pred;
  MatrixXd Ht = H_laser_.transpose();
  MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_laser_) * P_;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
 * Following code implements two functions:
 * 1. Predict Measurement
 * 2. Update State
 */

  //create matrix for sigma points in measurement space
  MatrixXd Zsig_ = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);

  for (int j = 0; j < 2 * n_aug_ + 1; ++j) {
    // extract values for better readibility
    double p_x = Xsig_pred_(0, j);
    double p_y = Xsig_pred_(1, j);
    double v = Xsig_pred_(2, j);
    double yaw = Xsig_pred_(3, j);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig_(0, j) = sqrt(p_x * p_x + p_y * p_y);                          //r
    if (Zsig_(0, j) < 0.00001)
      Zsig_(0, j) = 0.00001;
    Zsig_(1, j) = atan2(p_y, p_x);                                      //phi
    Zsig_(2, j) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);  //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred_ = VectorXd(n_z_radar_);

  z_pred_.fill(0.0);
  for (int k=0; k < 2*n_aug_+1; k++) {
    z_pred_ += weights(k) * Zsig_.col(k);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_,n_z_radar_);
  S.fill(0.0);
  for (int l = 0; l < 2 * n_aug_ + 1; l++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig_.col(l) - z_pred_;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(l) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_radar_;

  // incoming radar measurement
  VectorXd z_ = meas_package.raw_measurements_;

  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
  Tc.fill(0.0);
  for (int m = 0; m < 2 * n_aug_ + 1; ++m) {
    //residual
    VectorXd z_diff = Zsig_.col(m) - z_pred_;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(m) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(m) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z_ - z_pred_;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

}

/**
 * Helper functions
 */

void UKF::GenerateSigmaPoints() {
  // calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  Xsig_.col(0) = x_;
  for (int i = 0; i < n_x_; ++i) {
    Xsig_.col(i+1)       = x_ + sqrt(lambda_+ n_x_) * A.col(i);
    Xsig_.col(i+1+n_x_)  = x_ - sqrt(lambda_+ n_x_) * A.col(i);
  }
}

void UKF::AugmentedSigmaPoints() {
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug_.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
}

void UKF::SigmaPointPrediction(double delta_t) {
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    //extract values for better readability
    double p_x = Xsig_aug_(0, i);
    double p_y = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
    double yawd = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd > 0.001)) {
      px_p = p_x + v/yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v/yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
}

void UKF::PredictMeanAndCovariance() {

  //predicted state mean
  x_.fill(0.0);
  for (int k = 0; k < 2 * n_aug_ + 1; ++k) {
    x_ += weights(k) * Xsig_pred_.col(k);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int l = 0; l < 2 * n_aug_ + 1; ++l) {
    // state difference
    VectorXd x_diff_ = Xsig_pred_.col(l) - x_;
    //angle normalization
    while (x_diff_(3) > M_PI) x_diff_(3) -= 2. * M_PI;
    while (x_diff_(3) < M_PI) x_diff_(3) += 2. * M_PI;

    //predicted covariance matrix
    P_ += weights(l) * x_diff_ * x_diff_.transpose();
  }
}