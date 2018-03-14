#ifndef KF_OBSERVER_H
#define KF_OBSERVER_H

#include <Eigen/Core>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

#include <firefly_controller/kfObserverState.h>

namespace mav_control {
class KFObserver
{
 public:

  KFObserver();
  void resetX(const double& initial_position);
  void resetY(const double& initial_position);

  //Feeding
  void feedAttitudeCommand(const double attitude_cmd);
//  void feedVelocityMeasurement(const double velocity);
//  void feedPositionMeasurement(const double position);
  void feedMeasurement(const double position, const double velocity, const double attitude);

  bool updateEstimatorX();
  bool updateEstimatorY();

  void getEstimatedStateX(Eigen::VectorXd* estimated_state_x) const;
  void getEstimatedStateY(Eigen::VectorXd* estimated_state_y) const;

  virtual ~KFObserver();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  static constexpr int kStateSize = 4;
  static constexpr int kMeasurementSize = 3;
  static constexpr double kGravity = 9.8066;
  static constexpr int kInputSize = 1;

  bool initialized_;
  Eigen::Matrix<double, kStateSize, 1> state_x_;
  Eigen::Matrix<double, kStateSize, 1> state_y_;

  Eigen::Matrix<double, kStateSize, 1> predicted_state_;
  Eigen::Matrix<double, kMeasurementSize, 1> measurement_;
  double attitude_cmd_;


  Eigen::Matrix<double, kStateSize, kStateSize> state_covariance_x_;
  Eigen::Matrix<double, kStateSize, kStateSize> state_covariance_y_;

  Eigen::Matrix<double, kStateSize, kStateSize> process_noise_covariance_;
  Eigen::Matrix<double, kStateSize, kStateSize> initial_state_covariance_; // P0
  Eigen::Matrix<double, kMeasurementSize, kMeasurementSize> measurement_covariance_;


  Eigen::Matrix<double, kStateSize, kStateSize> A_; // System dynamics matrix.
  Eigen::Matrix<double, kStateSize, kMeasurementSize> K_; // Kalman gain matrix.
  Eigen::Matrix<double, kMeasurementSize, kStateSize> C_; // Measurement matrix.
  Eigen::Matrix<double, kStateSize, kInputSize> B_;


  double sampling_time_;

  ros::NodeHandle observer_nh_;

  ros::Publisher observer_state_x_pub_;
  ros::Publisher observer_state_y_pub_;

  void initialize();
};
}

#endif // KF_OBSERVER_H
