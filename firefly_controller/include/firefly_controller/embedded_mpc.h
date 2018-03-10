#ifndef EMBEDDED_MPC_H
#define EMBEDDED_MPC_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

#include <firefly_controller/kf_observer.h>



namespace mav_control {

class embeddedMPC
{
 public:

  embeddedMPC();
  ~embeddedMPC();

  // set odom and commands
  void setOdometry(const nav_msgs::Odometry& odometry);
  void setCommandTrajectoryPoint(const trajectory_msgs::MultiDOFJointTrajectory& command_trajectory);

  // compute control input
  void calculateRollPitchYawrateThrustCommand(Eigen::Vector4d *ref_attitude_thrust);


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:

  // constants
  static constexpr int kStateSize = 5;
  static constexpr int kInputSize = 1;
  static constexpr int kMeasurementSize = 1;
  static constexpr int kPredictionHorizonSteps = 200;
  static constexpr int kReducedHorizonSteps = 20;
  static constexpr double kGravity = 9.8066;


  //initialize parameters
  void initializeParameters();
  bool initialized_parameters_;

  double max_speed_;
  double sampling_time_;
  double Q11_;
  double S11_;
  double P_;

  double mass_;

  double roll_limit_;
  double pitch_limit_;

  double thrust_min_;
  double thrust_max_;

  double yaw_rate_limit_;

  double Kp_z_;
  double Kd_z_;

  double K_yaw_;

  // system model parameters
  Eigen::Matrix<double, kStateSize, kStateSize> model_A_;   //dynamics matrix
  Eigen::Matrix<double, kStateSize, kInputSize> model_B_;   //transfer matrix

  // controller parameters
  Eigen::MatrixXd A_roof;
  Eigen::MatrixXd U_;
  Eigen::MatrixXd B_roof;
  Eigen::MatrixXd B_roof_reduced;
  Eigen::Matrix<double, kStateSize, kStateSize> S_;
  Eigen::Matrix<double, kStateSize, kStateSize> Q_;

  Eigen::MatrixXd Q_roof;
  Eigen::MatrixXd P_roof;
  Eigen::MatrixXd H_inv;

  // reference queue
  Eigen::MatrixXd x_reference_;
  Eigen::MatrixXd y_reference_;
  Eigen::MatrixXd x_allstate_reference_;
  Eigen::MatrixXd y_allstate_reference_;

  double z_reference_;

//  // controller parameters
//  Eigen::Matrix<double, kPredictionHorizonSteps * kStateSize, kStateSize> A_roof;
//  Eigen::Matrix<double, kPredictionHorizonSteps, kReducedHorizonSteps> U_;
//  Eigen::Matrix<double, kPredictionHorizonSteps * kStateSize, kPredictionHorizonSteps> B_roof;
//  Eigen::Matrix<double, kPredictionHorizonSteps * kStateSize, kReducedHorizonSteps> B_roof_reduced;
//  Eigen::Matrix<double, kStateSize, kStateSize> S_;
//  Eigen::Matrix<double, kStateSize, kStateSize> Q_;
//  double P_;
//  Eigen::Matrix<double, kPredictionHorizonSteps * kStateSize, kPredictionHorizonSteps * kStateSize> Q_roof;
//  Eigen::Matrix<double, kReducedHorizonSteps, kReducedHorizonSteps> P_roof;
//  Eigen::Matrix<double, kReducedHorizonSteps, kReducedHorizonSteps> H_inv;

//  // reference queue
//  Eigen::Matrix<double, kPredictionHorizonSteps, 1> x_reference_;
//  Eigen::Matrix<double, kPredictionHorizonSteps, 1> y_reference_;
//  Eigen::Matrix<double, kPredictionHorizonSteps * kStateSize, 1> x_allstate_reference_;
//  Eigen::Matrix<double, kPredictionHorizonSteps * kStateSize, 1> y_allstate_reference_;



  // disturbance observer
  KFObserver kf_observer_;

  // commands
  Eigen::Vector4d command_roll_pitch_yaw_thrust_;

  // most recent odometry information
  mav_msgs::EigenOdometry odometry_;
};

}  // end namespace mav_control

#endif // EMBEDDED_MPC_H
