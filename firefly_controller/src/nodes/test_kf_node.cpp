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

class testKFnode
{
public:
  testKFnode();
  ~testKFnode();

protected:
  KFObserver kf_observer_;

  Eigen::Vector4d command_roll_pitch_yaw_thrust_;
  mav_msgs::EigenOdometry odometry_;

  ros::NodeHandle nh;

  ros::Subscriber odometry_sub_;
  ros::Subscriber cmd_sub_;

  void setOdometry(const nav_msgs::Odometry &odometry_nav);
  void setCommand(const mav_msgs::RollPitchYawrateThrust &cmd);
};

testKFnode::testKFnode():
  command_roll_pitch_yaw_thrust_(0, 0, 0, 0)
{
  odometry_sub_ = nh.subscribe("odometry", 1, &testKFnode::setOdometry, this);
  cmd_sub_ = nh.subscribe("command/roll_pitch_yawrate_thrust", 1, &testKFnode::setCommand, this);
}

testKFnode::~testKFnode(){}

void testKFnode::setCommand(const mav_msgs::RollPitchYawrateThrust &cmd){
  command_roll_pitch_yaw_thrust_(0) = cmd.roll;
  command_roll_pitch_yaw_thrust_(1) = cmd.pitch;
  command_roll_pitch_yaw_thrust_(2) = cmd.yaw_rate;
  command_roll_pitch_yaw_thrust_(3) = cmd.thrust.z;
}


void testKFnode::setOdometry(const nav_msgs::Odometry &odometry_nav){
  mav_msgs::EigenOdometry odometry;
  mav_msgs::eigenOdometryFromMsg(odometry_nav, &odometry);
  static mav_msgs::EigenOdometry previous_odometry = odometry;

  if (odometry.position_W.allFinite() == false) {
    odometry_.position_W = previous_odometry.position_W;
    ROS_WARN("Odometry.position has a non finite element");
  } else {
    odometry_.position_W = odometry.position_W;
    previous_odometry.position_W = odometry.position_W;
  }

  if (odometry.velocity_B.allFinite() == false) {
    odometry_.velocity_B = previous_odometry.velocity_B;
    ROS_WARN("Odometry.velocity has a non finite element");
  } else {
    odometry_.velocity_B = odometry.velocity_B;
    previous_odometry.velocity_B = odometry.velocity_B;
  }

  if (odometry.angular_velocity_B.allFinite() == false) {
    odometry_.angular_velocity_B = previous_odometry.angular_velocity_B;
    ROS_WARN("Odometry.angular_velocity has a non finite element");
  } else {
    odometry_.angular_velocity_B = odometry.angular_velocity_B;
    previous_odometry.angular_velocity_B = odometry.angular_velocity_B;
  }

  odometry_.orientation_W_B = odometry.orientation_W_B;
  previous_odometry.orientation_W_B = odometry.orientation_W_B;



  Eigen::Vector3d velocity_W = odometry_.getVelocityWorld();

  Eigen::Vector3d current_rpy;
  odometry_.getEulerAngles(&current_rpy);
  double roll = current_rpy(0);
  double pitch = current_rpy(1);
  double yaw = current_rpy(2);


  // x-pitch
  Eigen::VectorXd KF_estimated_state_x;

  // body frame to world frame
  double cmd_pitch_W = cos(yaw) * command_roll_pitch_yaw_thrust_(1) + sin(yaw) * command_roll_pitch_yaw_thrust_(0);
  double pitch_W = cos(yaw) * pitch + sin(yaw) * roll;

  // update the disturbance observer
  kf_observer_.feedAttitudeCommand(cmd_pitch_W);
//  kf_observer_.feedVelocityMeasurement(velocity_W(0));
//  kf_observer_.feedPositionMeasurement(odometry_.position_W(0));
  kf_observer_.feedMeasurement(odometry_.position_W(0), velocity_W(0), pitch_W);

  bool observer_update_successful = kf_observer_.updateEstimatorX();

  ROS_INFO_STREAM("observer_update_successful x : " << observer_update_successful);

  if (!observer_update_successful) {
    // reset the disturbance observer
    kf_observer_.resetX(odometry_.position_W(0));
  }

  kf_observer_.getEstimatedStateX(&KF_estimated_state_x);

  ROS_INFO_STREAM("KF_estimated_state x : \n" << KF_estimated_state_x);
  ROS_INFO_STREAM("ground truth x : " << odometry_.position_W(0));
  ROS_INFO_STREAM("ground truth vx : " << velocity_W(0));
  ROS_INFO_STREAM("ground truth pitch_W : " << pitch_W);


  // y-roll
  Eigen::VectorXd KF_estimated_state_y;

  // body frame to world frame
  double cmd_roll_W = -sin(yaw) * command_roll_pitch_yaw_thrust_(1) + cos(yaw) * command_roll_pitch_yaw_thrust_(0);
  double roll_W = -sin(yaw) * pitch + cos(yaw) * roll;

  // update the disturbance observer
  kf_observer_.feedAttitudeCommand(- cmd_roll_W);   // minus for roll
//  kf_observer_.feedVelocityMeasurement(velocity_W(1));
//  kf_observer_.feedPositionMeasurement(odometry_.position_W(1));
  kf_observer_.feedMeasurement(odometry_.position_W(1), velocity_W(1), - roll_W);   // minus for roll

  observer_update_successful = false;
  observer_update_successful = kf_observer_.updateEstimatorY();

  ROS_INFO_STREAM("observer_update_successful y : " << observer_update_successful);

  if (!observer_update_successful) {
    // reset the disturbance observer
    kf_observer_.resetY(odometry_.position_W(1));
  }

  kf_observer_.getEstimatedStateY(&KF_estimated_state_y);

  ROS_INFO_STREAM("KF_estimated_state y : \n" << KF_estimated_state_y);
  ROS_INFO_STREAM("ground truth y : " << odometry_.position_W(1));
  ROS_INFO_STREAM("ground truth vy : " << velocity_W(1));
  ROS_INFO_STREAM("ground truth -roll_W : " << -roll_W);
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_kf_node");

  mav_control::testKFnode test_kf_node;

  ros::spin();

  return 0;
}
