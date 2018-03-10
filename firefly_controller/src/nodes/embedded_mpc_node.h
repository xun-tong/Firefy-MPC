#ifndef EMBEDDED_MPC_NODE_H
#define EMBEDDED_MPC_NODE_H


#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

//ros
#include <ros/ros.h>
#include <firefly_controller/embedded_mpc.h>


namespace mav_control {

class EmbeddedMpcNode
{
 public:
  EmbeddedMpcNode();
  ~EmbeddedMpcNode();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  embeddedMPC embedded_mpc_;

  ros::Subscriber reference_sub_;
  ros::Subscriber odometry_sub_;

  ros::Publisher cmd_pub_;

  bool receivedReference_;

  virtual void setReference(const trajectory_msgs::MultiDOFJointTrajectory &reference);

  virtual void setOdometry(const nav_msgs::Odometry &odometry);
  };

}

#endif // EMBEDDED_MPC_NODE_H
