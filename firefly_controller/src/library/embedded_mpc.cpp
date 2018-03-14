#include <firefly_controller/embedded_mpc.h>

namespace mav_control {

constexpr int embeddedMPC::kStateSize;
constexpr int embeddedMPC::kInputSize;
constexpr int embeddedMPC::kMeasurementSize;
constexpr double embeddedMPC::kGravity;
constexpr int embeddedMPC::kPredictionHorizonSteps;
constexpr int embeddedMPC::kReducedHorizonSteps;

embeddedMPC::embeddedMPC()
    : initialized_parameters_(false),
      command_roll_pitch_yaw_thrust_(0, 0, 0, 0)   //actual command
{
  initializeParameters();
}

embeddedMPC::~embeddedMPC()
{
}

void embeddedMPC::initializeParameters()
{

  ros::NodeHandle pnh("~");

  if (!pnh.getParam("max_speed", max_speed_)) {
    ROS_ERROR("max_speed in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  double Q_pos, Q_vel, Q_att, Q_dis;
  if (!pnh.getParam("Q_pos", Q_pos)) {
    ROS_ERROR("Q_pos in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("Q_vel", Q_vel)) {
    ROS_ERROR("Q_vel in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("Q_att", Q_att)) {
    ROS_ERROR("Q_att in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("Q_dis", Q_dis)) {
    ROS_ERROR("Q_dis in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("P", P_)) {
    ROS_ERROR("P in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("roll_limit",roll_limit_)) {
    ROS_ERROR("roll_limit in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("pitch_limit", pitch_limit_)) {
    ROS_ERROR("pitch_limit in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("mass", mass_)) {
    ROS_ERROR("mass in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("Kp_z", Kp_z_)) {
    ROS_ERROR("Kp_z in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("Kd_z", Kd_z_)) {
    ROS_ERROR("Kd_z in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("thrust_min", thrust_min_)) {
    ROS_ERROR("thrust_min in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("thrust_max", thrust_max_)) {
    ROS_ERROR("thrust_max in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("yaw_rate_limit", yaw_rate_limit_)) {
    ROS_ERROR("yaw_rate_limit in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  if (!pnh.getParam("K_yaw", K_yaw_)) {
    ROS_ERROR("K_yaw in embedded_mpc is not loaded from ros parameter server");
    abort();
  }

  model_A_ << 1,  0.09995,    0.043,   0.00494836,
              0,  0.999,      0.81,    0.0999505,
              0,  0,          0.673,   0,
              0,  0,          0,       1;

  model_B_ << 0.0052,  0.152,  0.295,  0;

  ROS_INFO_STREAM("model_A_: \n" <<model_A_);
  ROS_INFO_STREAM("model_B_: \n" <<model_B_);


  // A_roof = [A;
  //           A^2;
  //           ...
  //           A^n]
  A_roof = Eigen::MatrixXd::Zero(kPredictionHorizonSteps * kStateSize, kStateSize);
  A_roof.block(0, 0, kStateSize, kStateSize) = model_A_;
  for(int i=1; i<kPredictionHorizonSteps; i++){
    A_roof.block(i*kStateSize, 0, kStateSize, kStateSize) = A_roof.block((i-1)*kStateSize, 0, kStateSize, kStateSize) * model_A_;
  }

  ROS_INFO_STREAM("A_roof rows: " << A_roof.rows());
  ROS_INFO_STREAM("A_roof cols: " << A_roof.cols());

  // U_   for reducing the complexity by move blocking
  U_ = Eigen::MatrixXd::Zero(kPredictionHorizonSteps, kReducedHorizonSteps);
  U_.block(0, 0, 10, 10) = Eigen::Matrix<double, 10, 10>::Identity();
  for(int i=1; i<=9; i++){
    U_.block(10*i, i+9, 10, 1) = Eigen::MatrixXd::Ones(10, 1);
  }
  U_.block(100, 19, 100, 1) = Eigen::MatrixXd::Ones(100, 1);

  ROS_INFO_STREAM("U_ rows: " << U_.rows());
  ROS_INFO_STREAM("U_ cols: " << U_.cols());

  // B_roof = [B,        0,        0,   0;
  //           AB,       B,        0,   0;
  //           A^2B,     AB,       B,   0;
  //           ...;
  //           A^(n-2)B, A^(n-1)B, ..., 0;
  //           A^(n-1)B, A^(n-2)B, ..., B]
  B_roof = Eigen::MatrixXd::Zero(kPredictionHorizonSteps * kStateSize, kPredictionHorizonSteps);
  for(int i=1; i<=kPredictionHorizonSteps; i++){
    for(int j=1; j<=i; j++){
      if(i>j){
        Eigen::Matrix<double, kStateSize, kStateSize> temp_A = model_A_;
        int temp_j = j+1;
        while(i>temp_j){
          temp_A = temp_A * model_A_;
          temp_j++;
        }
        B_roof.block((i-1)*kStateSize, (j-1)*kInputSize, kStateSize, kInputSize) = temp_A * model_B_;
      }
      else{
        B_roof.block((i-1)*kStateSize, (j-1)*kInputSize, kStateSize, kInputSize) = model_B_;
      }
    }
  }

  ROS_INFO_STREAM("B_roof rows: " << B_roof.rows());
  ROS_INFO_STREAM("B_roof cols: " << B_roof.cols());

  // B_roof_reduced
  B_roof_reduced = B_roof * U_;

  ROS_INFO_STREAM("B_roof_reduced rows: " << B_roof_reduced.rows());
  ROS_INFO_STREAM("B_roof_reduced cols: " << B_roof_reduced.cols());

  // Q_
  Q_ <<  Q_pos, 0,      0,      0,
         0,     Q_vel,  0,      0,
         0,     0,      Q_att,  0,
         0,     0,      0,      Q_dis;

  ROS_INFO_STREAM("Q_: \n" <<Q_);

  // S_
  //Compute terminal cost
  //Q_final(k+1) = A'*Q_final(k)*A - (A'*Q_final(k)*B)*inv(B'*Q_final(k)*B+R)*(B'*Q_final(k)*A)+ Q;
  S_ = Q_;                                                // Q_final: terminal state error penalty   by Xun
  for (int i = 0; i < 1000; i++) {
    double temp = (model_B_.transpose() * S_ * model_B_ + P_);
    S_ = model_A_.transpose() * S_ * model_A_
        - (model_A_.transpose() * S_ * model_B_) / temp
            * (model_B_.transpose() * S_ * model_A_) + Q_;
  }
  ROS_INFO_STREAM("S_: \n" <<S_);

//  S_ << S11_, 0, 0, 0, 0,
//           0, 0, 0, 0, 0,
//           0, 0, 0, 0, 0,
//           0, 0, 0, 0, 0,
//           0, 0, 0, 0, 0;

  // Q_roof = [Q,   0,   ...,  0;
  //           0,   Q,   ...,  0;
  //           ..., ..., Q,    0;
  //           0,   ..., ...,  S]
  Q_roof = Eigen::MatrixXd::Zero(kPredictionHorizonSteps * kStateSize, kPredictionHorizonSteps * kStateSize);
  for(int i=1; i<=kPredictionHorizonSteps; i++){
    Q_roof.block((i-1)*kStateSize, (i-1)*kStateSize, kStateSize, kStateSize) = Q_;
  }
  Q_roof.block((kPredictionHorizonSteps-1) * kStateSize, (kPredictionHorizonSteps-1) * kStateSize, kStateSize, kStateSize) = S_;

  ROS_INFO_STREAM("Q_roof rows: " << Q_roof.rows());
  ROS_INFO_STREAM("Q_roof cols: " << Q_roof.cols());

  // P_roof
  P_roof = Eigen::MatrixXd::Zero(kReducedHorizonSteps, kReducedHorizonSteps);
  for(int i=0; i<kReducedHorizonSteps; i++){
    P_roof(i, i) = P_;
  }
  for(int i =10; i<19; i++){
    P_roof(i, i) = 10 * P_;
  }
  P_roof(19, 19) = 100 * P_;

  ROS_INFO_STREAM("P_roof rows: " << P_roof.rows());
  ROS_INFO_STREAM("P_roof cols: " << P_roof.cols());

  // H_inv
  Eigen::MatrixXd H_(kReducedHorizonSteps, kReducedHorizonSteps);
  H_ = B_roof_reduced.transpose()*Q_roof*B_roof_reduced + P_roof;
  H_inv = (0.5*H_).inverse();
//  H_inv = H_.inverse();

  ROS_INFO_STREAM("H_inv rows: " << H_inv.rows());
  ROS_INFO_STREAM("H_inv cols: " << H_inv.cols());

  initialized_parameters_ = true;

  ROS_INFO("embedded MPC: initialized correctly");
}


void embeddedMPC::setOdometry(const nav_msgs::Odometry& odometry_nav)
{
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
}

void embeddedMPC::setCommandTrajectoryPoint(
    const trajectory_msgs::MultiDOFJointTrajectory& command_trajectory)
{
  x_reference_ = Eigen::MatrixXd::Ones(kPredictionHorizonSteps, 1);
  x_reference_ *= command_trajectory.points[0].transforms[0].translation.x;

  y_reference_ = Eigen::MatrixXd::Ones(kPredictionHorizonSteps, 1);
  y_reference_ *= command_trajectory.points[0].transforms[0].translation.y;

  z_reference_ = command_trajectory.points[0].transforms[0].translation.z;


//  ROS_INFO_STREAM("x_reference_ length: " << x_reference_.rows());
//  ROS_INFO_STREAM("x_reference_(0,0): " << x_reference_(0,0));
//  ROS_INFO_STREAM("y_reference_ length: " << y_reference_.rows());
//  ROS_INFO_STREAM("y_reference_(0,0): " << y_reference_(0,0));
//  ROS_INFO_STREAM("z_reference_: " << z_reference_);
}


void embeddedMPC::calculateRollPitchYawrateThrustCommand(
    Eigen::Vector4d *ref_attitude_thrust)
{
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

  // filter the reference
  x_allstate_reference_ = Eigen::MatrixXd::Zero(kPredictionHorizonSteps * kStateSize, 1);
  x_allstate_reference_(0) = KF_estimated_state_x(0);

  double diff_limit = max_speed_ * sampling_time_;
  for(int i = 2; i <= kPredictionHorizonSteps; i++){
    double difference = x_allstate_reference_(kStateSize*(i-2)) - x_reference_(i-1);
    if(difference > diff_limit){
      difference = diff_limit;
    }
    else if(difference < -diff_limit){
      difference = -diff_limit;
    }
    x_allstate_reference_(kStateSize*(i-1)) = x_allstate_reference_(kStateSize*(i-2)) - difference;
  }

  // calculate MPC
  Eigen::MatrixXd temp_vector1(kPredictionHorizonSteps * kStateSize, 1);
  temp_vector1 = A_roof*KF_estimated_state_x - x_allstate_reference_;
  for(int i = 0; i < kPredictionHorizonSteps * kStateSize; i++){
    temp_vector1(i, 0) = temp_vector1(i, 0) * Q_roof(i, i);
  }
  Eigen::MatrixXd input_opt(kReducedHorizonSteps, 1);
  input_opt = -H_inv * (temp_vector1.transpose() * B_roof_reduced).transpose() * 0.5;
//  input_opt = -H_inv * (temp_vector1.transpose() * B_roof_reduced).transpose();

  double pitch_cmd = input_opt(0);


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

  // filter the reference
  y_allstate_reference_ = Eigen::MatrixXd::Zero(kPredictionHorizonSteps * kStateSize, 1);
  y_allstate_reference_(0) = KF_estimated_state_y(0);

  for(int i = 2; i <= kPredictionHorizonSteps; i++){
    double difference = y_allstate_reference_(kStateSize*(i-2)) - y_reference_(i-1);
    if(difference > diff_limit){
      difference = diff_limit;
    }
    else if(difference < -diff_limit){
      difference = -diff_limit;
    }
    y_allstate_reference_(kStateSize*(i-1)) = y_allstate_reference_(kStateSize*(i-2)) - difference;
  }

  // calculate MPC
  temp_vector1 = A_roof*KF_estimated_state_y - y_allstate_reference_;
  for(int i = 0; i < kPredictionHorizonSteps * kStateSize; i++){
    temp_vector1(i, 0) = temp_vector1(i, 0) * Q_roof(i, i);
  }
  input_opt = -H_inv * (temp_vector1.transpose() * B_roof_reduced).transpose() * 0.5;
//  input_opt = -H_inv * (temp_vector1.transpose() * B_roof_reduced).transpose();

  double roll_cmd = -input_opt(0);     // minus for roll

  ROS_INFO_STREAM("roll_cmd in world after mpc : " << roll_cmd);
  ROS_INFO_STREAM("pitch_cmd in world after mpc : " << pitch_cmd);

  // limits
  if(roll_cmd > roll_limit_){
    roll_cmd = roll_limit_;
  }
  else if(roll_cmd < -roll_limit_){
    roll_cmd = -roll_limit_;
  }

  if(pitch_cmd > pitch_limit_){
    pitch_cmd = pitch_limit_;
  }
  else if(pitch_cmd < -pitch_limit_){
    pitch_cmd = -pitch_limit_;
  }



  // height thrust control
  command_roll_pitch_yaw_thrust_(3) = kGravity * mass_ - (odometry_.position_W(2) - z_reference_) * Kp_z_ - velocity_W(2) * Kd_z_;

  command_roll_pitch_yaw_thrust_(3) = command_roll_pitch_yaw_thrust_(3)/(cos(roll) * cos(pitch));   //project to z axis of body frame

  if(command_roll_pitch_yaw_thrust_(3) < thrust_min_){
    command_roll_pitch_yaw_thrust_(3) = thrust_min_;
  }
  else if(command_roll_pitch_yaw_thrust_(3) > thrust_max_){
    command_roll_pitch_yaw_thrust_(3) = thrust_max_;
  }

  // feed forward compensation
  pitch_cmd = pitch_cmd * (kGravity * mass_ / command_roll_pitch_yaw_thrust_(3));
  roll_cmd = roll_cmd * (kGravity * mass_ / command_roll_pitch_yaw_thrust_(3));

  ROS_INFO_STREAM("roll_cmd in world before limit : " << roll_cmd);
  ROS_INFO_STREAM("pitch_cmd in world before limit : " << pitch_cmd);


  // world frame to body frame
  command_roll_pitch_yaw_thrust_(0) = pitch_cmd * sin(yaw) + roll_cmd * cos(yaw);
  command_roll_pitch_yaw_thrust_(1) = pitch_cmd * cos(yaw) - roll_cmd * sin(yaw);


  // yaw controller
  double yaw_error = - yaw;

  if (std::abs(yaw_error) > 3.14) {
    if (yaw_error > 0.0) {
      yaw_error = yaw_error - 2.0 * 3.14;
    } else {
      yaw_error = yaw_error + 2.0 * 3.14;
    }
  }

  command_roll_pitch_yaw_thrust_(2) = K_yaw_ * yaw_error; // feed-forward yaw_rate cmd

  if (command_roll_pitch_yaw_thrust_(2) > yaw_rate_limit_) {
    command_roll_pitch_yaw_thrust_(2) = yaw_rate_limit_;
  }

  if (command_roll_pitch_yaw_thrust_(2) < -yaw_rate_limit_) {
    command_roll_pitch_yaw_thrust_(2) = -yaw_rate_limit_;
  }


  ROS_INFO_STREAM("yaw : " << yaw);
  ROS_INFO_STREAM("command_roll_pitch_yaw_thrust_ : \n" << command_roll_pitch_yaw_thrust_);

  *ref_attitude_thrust = Eigen::Vector4d(command_roll_pitch_yaw_thrust_(0),
                                         command_roll_pitch_yaw_thrust_(1),
                                         command_roll_pitch_yaw_thrust_(2),
                                         command_roll_pitch_yaw_thrust_(3));
}

}
