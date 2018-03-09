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

  observer_state_pub_ = observer_nh_.advertise<firefly_controller::kfObserverState>("observer_state", 1);

  std::vector<double> diag_process_noise_covariance, diag_initial_state_covariance;
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

  if (!pnh.getParam("measurement_covariance", measurement_covariance_)) {
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

  state_covariance_ = initial_state_covariance_;

  process_noise_covariance_.setZero();
  for (int i = 0; i < kStateSize; i++) {
    process_noise_covariance_(i, i) = diag_process_noise_covariance.at(i);
  }

  ROS_INFO_STREAM("process_noise_covariance_: \n" <<process_noise_covariance_);


  A_ << 1,  sampling_time_,    0,                 0,          0,
        0,  1,                 sampling_time_,    0,          0,
        0,  0,                 0,                 1,          1,
        0,  A_vel_,            0,                 A_acc_u_,   0,
        0,  0,                 0,                 0,          1;

  B_ << 0,  0,  0,  B_acc_u_,  0;

  C_ << 1,  0,  0,  0,  0;

  ROS_INFO_STREAM("kf A_: \n" <<A_);
  ROS_INFO_STREAM("kf B_: \n" <<B_);
  ROS_INFO_STREAM("kf C_: \n" <<C_);

  state_.setZero();
  predicted_state_.setZero();

  initialized_ = true;

  ROS_INFO("Kalman Filter Initialized!");

}


void KFObserver::reset(const double& initial_position)
{

  state_covariance_ = initial_state_covariance_;

  state_.setZero();

  state_(0) = initial_position;
}


void KFObserver::feedAttitudeCommand(const double attitude_cmd)
{
  this->attitude_cmd_ = attitude_cmd;
}

void KFObserver::feedVelocityMeasurement(const double velocity)
{
  this->measurement_ = velocity;
}

void KFObserver::feedPositionMeasurement(const double position)
{
  this->measurement_ = position;
}


bool KFObserver::updateEstimator()
{
  if (initialized_ == false)
    return false;

  ROS_INFO_ONCE("KF is updated for first time.");

  // recompute covariance
  state_covariance_ = A_ * state_covariance_ * A_.transpose();
  state_covariance_ += process_noise_covariance_;

  //predict state
  predicted_state_ = A_ * state_ + B_*attitude_cmd_;

  // compute kalman gain
  // K = covariance*C'*((C*covariance*C' + Q)^-1)
  double tmp = C_ * state_covariance_ * C_.transpose() + measurement_covariance_;
  K_ = state_covariance_ * C_.transpose() / tmp;

  //Update with measurements
  state_ = predicted_state_ + K_ * (measurement_ - C_ * state_);

  //Update covariance
  // covariance = (eye(n) - K*C)*covariance
  state_covariance_ = (Eigen::Matrix<double, kStateSize, kStateSize>::Identity() - K_ * C_) * state_covariance_;


  if (observer_state_pub_.getNumSubscribers() > 0) {
    firefly_controller::kfObserverStatePtr msg(new firefly_controller::kfObserverState);
      msg->header.stamp = ros::Time::now();
      msg->position = state_(0);
      msg->velocity = state_(1);
      msg->acceleration = state_(2);
      msg->control_acc = state_(3);
      msg->disturb_acc = state_(4);

    observer_state_pub_.publish(msg);
  }
  return true;
}


void KFObserver::getEstimatedState(Eigen::VectorXd* estimated_state) const
{
  assert(estimated_state);
  assert(initialized_);

  estimated_state->resize(kStateSize);
  *estimated_state = this->state_;
}

KFObserver::~KFObserver()
{
}

}
