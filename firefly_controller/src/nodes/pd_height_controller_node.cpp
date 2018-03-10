#include <boost/bind.hpp>
#include <stdio.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <firefly_controller/PDheightConfig.h>

class pdHeightControllerNode
{
public:
  pdHeightControllerNode();
  ~pdHeightControllerNode();

protected:
  bool softLanding_;
  bool receivedReference_;

  double mass_;

  double gravity_N_;
  double height_;
  double velocity_;
  double position_gain_;
  double velocity_gain_;
  double desired_height_;
  double desired_velocity_;

  ros::NodeHandle nh_;
  ros::NodeHandle nhLocal_;

  ros::Subscriber reference_sub_;
  ros::Subscriber odometry_sub_;
  ros::Publisher thrust_pub_;

  std::string referenceSubTopic;
  std::string odometrySubTopic;
  std::string thrustPubTopic;

//  void InitializeParams();
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
  void ReferenceCallback(const trajectory_msgs::MultiDOFJointTrajectory& reference);
  void DynConfigCallback(firefly_controller::PDheightConfig &config, uint32_t level);

  dynamic_reconfigure::Server<firefly_controller::PDheightConfig> dyn_config_server_;
};

pdHeightControllerNode::pdHeightControllerNode():
  nhLocal_("~"),
  receivedReference_(false)
{
  mass_ = 1.56779;
  gravity_N_ = 9.80665;

  dynamic_reconfigure::Server<firefly_controller::PDheightConfig>::CallbackType f;
  f = boost::bind(&pdHeightControllerNode::DynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);

  reference_sub_ = nh_.subscribe(referenceSubTopic, 1, &pdHeightControllerNode::ReferenceCallback, this);
  odometry_sub_ = nh_.subscribe(odometrySubTopic, 1, &pdHeightControllerNode::OdometryCallback, this);
  thrust_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>(thrustPubTopic, 1);

//  InitializeParams();
}

pdHeightControllerNode::~pdHeightControllerNode(){}

void pdHeightControllerNode::DynConfigCallback(firefly_controller::PDheightConfig &config, uint32_t level){
  position_gain_ = config.position_gain;
  velocity_gain_ = config.velocity_gain;
  desired_velocity_ = config.desired_velocity;

  softLanding_ =  config.softLanding;

  odometrySubTopic = config.odometry_sub_topic;
  thrustPubTopic = config.thrust_pub_topic;
  referenceSubTopic = config.reference_sub_topic;
}

void pdHeightControllerNode::ReferenceCallback(const trajectory_msgs::MultiDOFJointTrajectory& reference){
  desired_height_ = reference.points[0].transforms[0].translation.z;
  receivedReference_ = true;
  ROS_INFO_ONCE("HeightController got first reference message");
}

void pdHeightControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg){
  ROS_INFO_ONCE("HeightController got first odometry message.");
  height_ = odometry_msg->pose.pose.position.z;
  velocity_ =odometry_msg->twist.twist.linear.z;


  if(receivedReference_ == true){
    mav_msgs::RollPitchYawrateThrustPtr msg(new mav_msgs::RollPitchYawrateThrust);
    if(softLanding_ == false){
      double position_error = height_ - desired_height_;
      double velocity_error = velocity_ - desired_velocity_;
      double thrust = gravity_N_*mass_ - position_error*position_gain_ - velocity_error*velocity_gain_;
      msg->thrust.z = thrust;
    }
    else {
      msg->thrust.z = 15.3;
    }
    msg->thrust.x = 0;
    msg->thrust.y = 0;
    msg->roll = 0;
    msg->pitch = 0;
    msg->yaw_rate = 0;
    thrust_pub_.publish(msg);
  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "PD_height_controller_node");

  pdHeightControllerNode PD_height_controller_node;

  ros::spin();

  return 0;
}
