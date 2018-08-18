#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include "tools.h"
//#include <string>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
 public:
  // for debug purpose
  bool debug_; 

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);


/////////////////////////////////////////////////////////////////////////////////

private:
  /**
   * Get augmented sigma points matrix
   * @param Xsig_out pointer to sigma points matrix where the result is written
   */
  MatrixXd GetAugmentedSigmaPoints();

  /**
   * Transforms augmented sigma points using process equations
   * Will change the value of Xsig_pred_ 
   * @param Xsig_aug  augmented sigma points matrix  
   * @param delta_t   time in seconds
   */
  void SigmaPointPrediction(MatrixXd &Xsig_aug, double delta_t);

  /**
   * Predict mean and covariance from predicted sigma points
   * Will change the value of x_ and P_ based on Xsig_pred_ 
   */
  void PredictMeanAndCovariance();

  /**
   * Normalize angle to between 0 and 2*PI
   * @param angle  input angle to be normalized
   */
  void NormalizeAngle(double &angle);
  
  /**
   * Predict RADAR measurement mean z_pred and covariance S based on predicted sigma points
   * @param n_z     dimensions of RADAR measurements
   * @param &Zsig   predicted sigma points matrix
   * @param &z_pred predicted measurement
   * @param &S      measurement covariance
   */
  void PredictRadarMeasurement(int n_z, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S);
  /**
   * Predict LIDAR measurement mean z_pred and covariance S from predicted sigma points
   * @param n_z     dimensions of LIDAR measurements
   * @param &Zsig   predicted sigma points matrix
   * @param &z_pred predicted measurement from sigma points
   * @param &S      measurement covariance
   */
  void PredictLidarMeasurement(int n_z, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S);

  /**
   * Update state x_ and covariance P_ from measurement.
   * @param n_z     dimensions of measurement space
   * @param &Zsig   sigma points matrix
   * @param &z_pred predicted measurment from sigma points
   * @param &S      measurement covariance
   * @param &z      actual measurment
   */
  void UpdateState(int n_z, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S, VectorXd &z);

};

#endif /* UKF_H */
