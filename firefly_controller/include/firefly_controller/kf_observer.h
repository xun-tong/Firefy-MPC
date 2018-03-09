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
  void reset(const double& initial_position);

  void getEstimatedState(Eigen::VectorXd* estimated_state) const;

  //Feeding
  void feedAttitudeCommand(const double attitude_cmd);
  void feedVelocityMeasurement(const double velocity);
  void feedPositionMeasurement(const double position);

  bool updateEstimator();

  virtual ~KFObserver();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  static constexpr int kStateSize = 5;
  static constexpr int kMeasurementSize = 1;
  static constexpr double kGravity = 9.8066;
  static constexpr int kInputSize = 1;

  bool initialized_;
  Eigen::Matrix<double, kStateSize, 1> state_;
  Eigen::Matrix<double, kStateSize, 1> predicted_state_;
  Eigen::Matrix<double, kMeasurementSize, 1> measurements_;
  double attitude_cmd_;
  double measurement_;

  Eigen::Matrix<double, kStateSize, kStateSize> process_noise_covariance_;
  Eigen::Matrix<double, kStateSize, kStateSize> state_covariance_;
  Eigen::Matrix<double, kStateSize, kStateSize> initial_state_covariance_; // P0
  double measurement_covariance_;


  Eigen::Matrix<double, kStateSize, kStateSize> A_; // System dynamics matrix.
  Eigen::Matrix<double, kStateSize, kMeasurementSize> K_; // Kalman gain matrix.
  Eigen::Matrix<double, kMeasurementSize, kStateSize> C_; // Measurement matrix.
  Eigen::Matrix<double, kStateSize, kInputSize> B_;


  double sampling_time_;

  ros::NodeHandle observer_nh_;

  ros::Publisher observer_state_pub_;

  void initialize();
};
}

#endif // KF_OBSERVER_H
