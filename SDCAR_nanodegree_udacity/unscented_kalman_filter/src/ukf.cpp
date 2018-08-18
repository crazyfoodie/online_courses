#include "ukf.h"
#include <iostream>

using namespace std;
//using std::vector;

UKF::UKF() {
  // set debug to true to print information
  debug_ = false;

  // not initialised yet
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // UKF constants
  // State dimension
  n_x_ = 5;
  // Augmented state dimension
  n_aug_ = 7;
  // Sigma point spreading parameter
  lambda_ = 3.0 - n_aug_;

  // initial state vector
  x_ = VectorXd(n_x_);

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_); // could tunning 
  P_ << 0.1,      0.,       0.,            0.,            0.,
        0.,       0.1,      0.,            0.,            0., 
        0.,       0.,       0.1,           0.,            0.,
        0.,       0.,       0.,            0.1,           0.,
        0.,       0.,       0.,            0.,            0.1;
  
  // Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i = 1; i<2*n_aug_+1; ++i) {
    weights_(i) = 0.5/(lambda_+n_aug_);
  }

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;     // could tunning 

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6; // could tunning 

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

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    cout << "UKF start initialization" << endl;

    // Initialize the state x_ with the first measurement.
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates
      double r = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      x_ <<   r * cos(phi),
              r * sin(phi),
              0.0, // assume velosity is zero
              0.0, // assume angle is zero
              0.0; // assume angle not change
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      double p_x = meas_package.raw_measurements_(0);
      double p_y = meas_package.raw_measurements_(1);
      x_ << p_x,
            p_y,
            0.0, 
            0.0,
            0.0; 
    }


    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    if (debug_) {
      cout << "x_ = " << endl << x_ << endl;
      cout << "P_ = " << endl << P_ << endl;
    }
    return;
  }

  // skip measurement if measurements need to be ignored
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_) {
    cout << "skipping one RADAR data" << " at "<< meas_package.timestamp_<< endl;
    return;
  }
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_) {
    cout << "skipping one LIDAR data" << " at "<< meas_package.timestamp_<< endl;
    return;
  }

  //compute the time elapsed between the current and previous measurements
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;     //dt - expressed in seconds
  time_us_ = meas_package.timestamp_;
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  Prediction(dt); 
  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar update
    cout << "updating radar measurement" << endl;
    UpdateRadar(meas_package);
    cout << "radar update done" << endl;
  } else {
    // Laser update
    cout << "updating laser measurement" << endl;
    UpdateLidar(meas_package);
    cout << "laser update done" << endl;
  }

  // print the output
  if (debug_) {
    cout << "x_ = " << endl << x_ << endl;
    cout << "P_ = " << endl <<  P_ << endl;
  }
}

void UKF::Prediction(double delta_t) {
  // get augmented sigma points vectors
  MatrixXd Xsig_aug = GetAugmentedSigmaPoints();
  // sigma points prediction
  SigmaPointPrediction(Xsig_aug, delta_t); 
  // predict mean and covariance
  PredictMeanAndCovariance();

  if (debug_) {
    cout << "Xsig_aug = " << endl << Xsig_aug << endl;
    cout << "Xsig_pred = " << endl << Xsig_pred_ << endl;
    cout << "x_ = " << endl << x_ << endl;
    cout << "P_ = " << endl << P_ << endl;
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  // lidar measurement dimensionality: px, py
  int n_z = 2;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z,n_z);

  // calculate Zsig, z_pred and S based on Xsig_pred_ from prediction step
  PredictLidarMeasurement(n_z, Zsig, z_pred, S);

  // update state and covariance using UKF equations
  VectorXd z = meas_package.raw_measurements_;
  cout << "z lidar = " << endl << z << endl;
  UpdateState(n_z, Zsig, z_pred, S, z);

  // calculate NIS
  VectorXd z_diff = z - z_pred;
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  cout << "NIS lidar = " << NIS_laser_ << endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // radar measurement dimensionality: rho, phi, r_dot
  int n_z = 3;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z,n_z);

  // calculate Zsig, z_pred and S based on Xsig_pred_ from prediction step
  PredictRadarMeasurement(n_z, Zsig, z_pred, S);

  // update state and covariance using UKF equations
  VectorXd z = meas_package.raw_measurements_;
  NormalizeAngle(z(1));
  cout << "z radar = " << endl << z << endl;
  UpdateState(n_z, Zsig, z_pred, S, z);

  // calculate NIS
  VectorXd z_diff = z - z_pred;
  NormalizeAngle(z_diff(1));
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  cout << "NIS radar = " << NIS_radar_ << endl;
}

MatrixXd UKF::GetAugmentedSigmaPoints() {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0.;
  x_aug(6) = 0.;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  return Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd &Xsig_aug, double delta_t) {

  double delta_t_2 = delta_t * delta_t;

  //predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v / yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v / yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
    } else {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t_2 * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t_2 * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t_2;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
}

void UKF::PredictMeanAndCovariance() {
  //predict state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_+1; ++i) {
    x_ += weights_(i)*Xsig_pred_.col(i);
  }

  //predict state covariance matrix to mitigate non-positive-semi-definite issues
  P_.fill(0.0);
  for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    NormalizeAngle(x_diff(3));

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::NormalizeAngle(double &angle) {
  while (angle > M_PI) angle -= 2.*M_PI;
  while (angle < -M_PI) angle += 2.*M_PI;
}

void UKF::PredictLidarMeasurement(int n_z, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) {
  //transform sigma points into LIDAR measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) { 
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;
  }

  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    // state difference
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S += weights_(i) * z_diff * z_diff.transpose();
  }
  
  // add measurement noise
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_laspx_*std_laspx_, 0.,
          0.,                    std_laspy_*std_laspy_;

  S = S + R;

  if (debug_) {
    cout << "Zsig lidar = " << endl << Zsig << endl;
    cout << "z_pred lidar = " << endl << z_pred << endl;
    cout << "S lidar = " << endl << S << endl;
  }
}

void UKF::PredictRadarMeasurement(int n_z, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) {

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // calculate r
    Zsig(0,i) = sqrt(p_x * p_x + p_y * p_y);

    // calculate phi
    if (abs(p_x)>1e-6) {
      Zsig(1,i) = atan2(p_y, p_x);
    } else {
      Zsig(1,i) = M_PI/2.; 
    }
    // calculate r_dot
    if (abs(Zsig(0,i)) < 1e-6) {
      Zsig(2,i) = 0.0; 
    } else {
      Zsig(2,i) = (p_x * v1 + p_y * v2 ) / Zsig(0,i); // (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y)
    }
  }

  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    // state difference
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    NormalizeAngle(z_diff(1));

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0.,                      0.,
          0.,                  std_radphi_*std_radphi_, 0.,
          0.,                  0.,                      std_radrd_*std_radrd_;

  S = S + R;

  if (debug_) {
    cout << "Zsig lidar = " << endl << Zsig << endl;
    cout << "z_pred lidar = " << endl << z_pred << endl;
    cout << "S lidar = " << endl << S << endl;
  }

}

void UKF::UpdateState(int n_z, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S, VectorXd &z) {
  //matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization for radar 
    if (n_z == 3) 
      NormalizeAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    NormalizeAngle(x_diff(3));

    Tc += weights_(i)*(x_diff)*(z_diff.transpose());
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;
  if (n_z == 3) 
    NormalizeAngle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  NormalizeAngle(x_(3));
  P_ = P_ - K * S * K.transpose();
}