#include <mav_msgs/default_topics.h>
#include "embedded_mpc_node.h"

namespace mav_control {

EmbeddedMpcNode::EmbeddedMpcNode():
  receivedReference_(false)
{
  ros::NodeHandle nh;
  reference_sub_ = nh.subscribe("command/trajectory", 1, &EmbeddedMpcNode::setReference, this);
  odometry_sub_ = nh.subscribe("odometry", 1, &EmbeddedMpcNode::setOdometry, this);
  cmd_pub_ = nh.advertise<mav_msgs::RollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 1);
}

EmbeddedMpcNode::~EmbeddedMpcNode(){}

void EmbeddedMpcNode::setReference(const trajectory_msgs::MultiDOFJointTrajectory& reference)
{
  embedded_mpc_.setCommandTrajectoryPoint(reference);
  receivedReference_ = true;
  ROS_INFO_ONCE("embedded MPC: got first reference message");
}

void EmbeddedMpcNode::setOdometry(const nav_msgs::Odometry &odometry)
{
  embedded_mpc_.setOdometry(odometry);

  if(receivedReference_ == true){
    Eigen::Vector4d rpy_thrust;
    embedded_mpc_.calculateRollPitchYawrateThrustCommand(&rpy_thrust);
    mav_msgs::RollPitchYawrateThrust attitude_thrust_command;
    attitude_thrust_command.pitch = rpy_thrust(0);
    attitude_thrust_command.roll = rpy_thrust(1);

    attitude_thrust_command.yaw_rate = rpy_thrust(2);
    attitude_thrust_command.thrust.z = rpy_thrust(3);

    attitude_thrust_command.thrust.x = 0;
    attitude_thrust_command.thrust.y = 0;

    cmd_pub_.publish(attitude_thrust_command);
  }
}

}  // end namespace mav_control

int main(int argc, char** argv)
{
  ros::init(argc, argv, "EmbeddedMpcNode");

  mav_control::EmbeddedMpcNode embedded_mpc_node;

  ros::spin();

  return 0;
}
