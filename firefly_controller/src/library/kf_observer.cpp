#include <firefly_controller/kf_observer.h>

namespace mav_control {

// http://stackoverflow.com/questions/24424996/allocating-an-object-of-abstract-class-type-error
constexpr int KFObserver::kStateSize;
constexpr int KFObserver::kMeasurementSize;
constexpr double KFObserver::kGravity;
constexpr int KFObserver::kInputSize;

KFObserver::KFObserver()
    : initialized_(false)
{
  initialize();
}


void KFObserver::initialize()
{
  ROS_INFO("start initializing observer:KF");

  observer_state_x_pub_ = observer_nh_.advertise<firefly_controller::kfObserverState>("observer_state_x", 1);
  observer_state_y_pub_ = observer_nh_.advertise<firefly_controller::kfObserverState>("observer_state_y", 1);

  std::vector<double> diag_process_noise_covariance, diag_initial_state_covariance, diag_measurement_covariance;
  ros::NodeHandle pnh("~");
  if (!pnh.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time in kf_observer is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("diag_initial_state_covariance", diag_initial_state_covariance)) {
    ROS_ERROR("diag_initial_state_covariance is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("diag_process_noise_covariance", diag_process_noise_covariance)) {
    ROS_ERROR("diag_process_noise_covariance is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("measurement_covariance", diag_measurement_covariance)) {
    ROS_ERROR("measurement_covariance is not loaded from ros parameter server");
    abort();
  }

  double A_vel_, A_acc_u_, B_acc_u_;

  if (!pnh.getParam("A_vel", A_vel_)) {
    ROS_ERROR("A_vel in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("A_acc_u",A_acc_u_)) {
    ROS_ERROR("A_acc_u in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("B_acc_u", B_acc_u_)) {
    ROS_ERROR("B_acc_u in embedded_mpc is not loaded from ros parameter server");
    abort();
  }


  initial_state_covariance_.setZero();
  for (int i = 0; i < kStateSize; i++) {
    initial_state_covariance_(i, i) = diag_initial_state_covariance.at(i);
  }

  ROS_INFO_STREAM("initial_state_covariance_: \n" <<initial_state_covariance_);

  state_covariance_x_ = initial_state_covariance_;
  state_covariance_y_ = initial_state_covariance_;

  process_noise_covariance_.setZero();
  for (int i = 0; i < kStateSize; i++) {
    process_noise_covariance_(i, i) = diag_process_noise_covariance.at(i);
  }

  measurement_covariance_.setZero();
  for (int i = 0; i < kMeasurementSize; i++) {
    measurement_covariance_(i, i) = diag_measurement_covariance.at(i);
  }

  ROS_INFO_STREAM("process_noise_covariance_: \n" <<process_noise_covariance_);
  ROS_INFO_STREAM("measurement_covariance_: \n" <<measurement_covariance_);



  A_ << 1,  0.09995,    0.043,   0.00494836,
        0,  0.999,      0.81,    0.0999505,
        0,  0,          0.673,   0,
        0,  0,          0,       1;

  B_ << 0.0052,  0.152,  0.295,  0;

  C_ << 1,  0,  0,  0,
        0,  1,  0,  0,
        0,  0,  1,  0;

  ROS_INFO_STREAM("kf A_: \n" <<A_);
  ROS_INFO_STREAM("kf B_: \n" <<B_);
  ROS_INFO_STREAM("kf C_: \n" <<C_);

  state_x_.setZero();
  state_y_.setZero();
  predicted_state_.setZero();

  initialized_ = true;

  ROS_INFO("Kalman Filter Initialized!");

}


void KFObserver::resetX(const double& initial_position)
{
  state_covariance_x_ = initial_state_covariance_;

  state_x_.setZero();

  state_x_(0) = initial_position;
}

void KFObserver::resetY(const double& initial_position)
{
  state_covariance_y_ = initial_state_covariance_;

  state_y_.setZero();

  state_y_(0) = initial_position;
}



void KFObserver::feedAttitudeCommand(const double attitude_cmd)
{
  this->attitude_cmd_ = attitude_cmd;
}

//void KFObserver::feedVelocityMeasurement(const double velocity)
//{
//  this->measurement_ = velocity;
//}

//void KFObserver::feedPositionMeasurement(const double position)
//{
//  this->measurement_ = position;
//}

void KFObserver::feedMeasurement(const double position, const double velocity, const double attitude)
{
  this->measurement_(0) = position;
  this->measurement_(1) = velocity;
  this->measurement_(2) = attitude;
}

// x-pitch
bool KFObserver::updateEstimatorX()
{
  if (initialized_ == false)
    return false;

  ROS_INFO_ONCE("KF of x is updated for first time.");

  // recompute covariance
  state_covariance_x_ = A_ * state_covariance_x_ * A_.transpose();
  state_covariance_x_ += process_noise_covariance_;

  //predict state
  predicted_state_ = A_ * state_x_ + B_*attitude_cmd_;

  // compute kalman gain
  // K = covariance*C'*((C*covariance*C' + Q)^-1)
  Eigen::Matrix<double, kMeasurementSize, kMeasurementSize> tmp = C_ * state_covariance_x_ * C_.transpose() + measurement_covariance_;
  K_ = state_covariance_x_ * C_.transpose() * tmp.inverse();

  //Update with measurements
  state_x_ = predicted_state_ + K_ * (measurement_ - C_ * state_x_);

  //Update covariance
  // covariance = (eye(n) - K*C)*covariance
  state_covariance_x_ = (Eigen::Matrix<double, kStateSize, kStateSize>::Identity() - K_ * C_) * state_covariance_x_;


  if (observer_state_x_pub_.getNumSubscribers() > 0) {
    firefly_controller::kfObserverStatePtr msg_x(new firefly_controller::kfObserverState);
      msg_x->header.stamp = ros::Time::now();
      msg_x->position = state_x_(0);
      msg_x->velocity = state_x_(1);
      msg_x->attitude = state_x_(2);
      msg_x->disturb = state_x_(3);

    observer_state_x_pub_.publish(msg_x);
  }
  return true;
}


void KFObserver::getEstimatedStateX(Eigen::VectorXd* estimated_state_x) const
{
  assert(estimated_state_x);
  assert(initialized_);

  estimated_state_x->resize(kStateSize);
  *estimated_state_x = this->state_x_;
}


// y-roll
bool KFObserver::updateEstimatorY()
{
  if (initialized_ == false)
    return false;

  ROS_INFO_ONCE("KF of y is updated for first time.");

  // recompute covariance
  state_covariance_y_ = A_ * state_covariance_y_ * A_.transpose();
  state_covariance_y_ += process_noise_covariance_;

  //predict state
  predicted_state_ = A_ * state_y_ + B_*attitude_cmd_;

  // compute kalman gain
  // K = covariance*C'*((C*covariance*C' + Q)^-1)
  Eigen::Matrix<double, kMeasurementSize, kMeasurementSize> tmp = C_ * state_covariance_y_ * C_.transpose() + measurement_covariance_;
  K_ = state_covariance_y_ * C_.transpose() * tmp.inverse();

  //Update with measurements
  state_y_ = predicted_state_ + K_ * (measurement_ - C_ * state_y_);

  //Update covariance
  // covariance = (eye(n) - K*C)*covariance
  state_covariance_y_ = (Eigen::Matrix<double, kStateSize, kStateSize>::Identity() - K_ * C_) * state_covariance_y_;


  if (observer_state_y_pub_.getNumSubscribers() > 0) {
    firefly_controller::kfObserverStatePtr msg_y(new firefly_controller::kfObserverState);
      msg_y->header.stamp = ros::Time::now();
      msg_y->position = state_y_(0);
      msg_y->velocity = state_y_(1);
      msg_y->attitude = state_y_(2);
      msg_y->disturb = state_y_(3);

    observer_state_y_pub_.publish(msg_y);
  }
  return true;
}


void KFObserver::getEstimatedStateY(Eigen::VectorXd* estimated_state_y) const
{
  assert(estimated_state_y);
  assert(initialized_);

  estimated_state_y->resize(kStateSize);
  *estimated_state_y = this->state_y_;
}

KFObserver::~KFObserver()
{
}

}
