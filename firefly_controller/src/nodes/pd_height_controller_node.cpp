#include <boost/bind.hpp>
#include <stdio.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

class pdHeightControllerNode
{
public:
  pdHeightControllerNode();
  ~pdHeightControllerNode();

  void InitializeParams();
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

protected:
  double gravity_N_;
  double height_;
  double velocity_;
  double position_gain_;
  double velocity_gain_;
  double desired_height_;
  double desired_velocity_;

  ros::NodeHandle nh_;
  ros::NodeHandle nhLocal_;

  ros::Subscriber odometry_sub_;
  ros::Publisher thrust_pub_;

  std::string odometrySubTopic;
  std::string thrustPubTopic;
};

pdHeightControllerNode::pdHeightControllerNode():
  nhLocal_("~"),
  odometrySubTopic("ground_truth/odometry"),
  thrustPubTopic("cmd_RPYthrust")
{
  gravity_N_ = 9.80665;
  position_gain_ = 3.8;
  velocity_gain_ = 3;
  desired_height_ = 0.5;
  desired_velocity_ = 0;

  InitializeParams();
}

pdHeightControllerNode::~pdHeightControllerNode(){}

void pdHeightControllerNode::InitializeParams(){
  nhLocal_.param("position_gain", position_gain_, position_gain_);
  nhLocal_.param("velocity_gain", velocity_gain_, velocity_gain_);
  nhLocal_.param("desired_height", desired_height_, desired_height_);
  nhLocal_.param("desired_velocity", desired_velocity_, desired_velocity_);

  nhLocal_.param("odometry_sub_topic", odometrySubTopic, odometrySubTopic);
  nhLocal_.param("thrust_pub_topic", thrustPubTopic, thrustPubTopic);
  odometry_sub_ = nh_.subscribe(odometrySubTopic, 1, &pdHeightControllerNode::OdometryCallback, this);
  thrust_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>(thrustPubTopic, 1);
}

void pdHeightControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg){
  ROS_INFO_ONCE("HeightController got first odometry message.");
  height_ = odometry_msg->pose.pose.position.z;
  velocity_ =odometry_msg->twist.twist.linear.z;
  double position_error = height_ - desired_height_;
  double velocity_error = velocity_ - desired_velocity_;
  double thrust = gravity_N_*1.56779 - position_error*position_gain_ - velocity_error*velocity_gain_;     // mass: 1.56779

  mav_msgs::RollPitchYawrateThrustPtr msg(new mav_msgs::RollPitchYawrateThrust);
  msg->header.stamp = odometry_msg->header.stamp;
  msg->thrust.x = 0;
  msg->thrust.y = 0;
  msg->thrust.z = thrust;
  msg->roll = 0;
  msg->pitch = 0;
  msg->yaw_rate = 0;

  thrust_pub_.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "PD_height_controller_node");

  pdHeightControllerNode PD_height_controller_node;

  ros::spin();

  return 0;
}
