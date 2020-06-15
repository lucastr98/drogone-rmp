#include <drogone_motion_planner/rmp_planner.h>

namespace drogone_rmp_planner {

RMPPlanner::RMPPlanner(std::string name, ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  as_(nh, name, boost::bind(&RMPPlanner::server_callback, this, _1), false),
  action_name_(name),
  first_odom_cb_(true),
  f_x_(1140),
  f_y_(1140),
  u_0_(0),
  v_0_(0) {

    // publisher for trajectory to drone
    trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);

    // publisher for trajectory to drone
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        mav_msgs::default_topics::COMMAND_POSE, 1);

    // publisher for trajectory to drone
    u_v_pub_ = nh_.advertise<drogone_msgs_rmp::CameraUV>("/u_v", 0);

    f_u_v_pub_ = nh.advertise<drogone_msgs_rmp::AccFieldUV>("/f_u_v", 0);

    // subscriber for Odometry from drone
    sub_odom_ =
        nh.subscribe("uav_pose", 1, &RMPPlanner::uavOdomCallback, this);

    as_.start();
}

// Callback to get current Pose of UAV
void RMPPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  // calculate the time between the 2 measurements based on stamps
  double duration = odom->header.stamp.sec + odom->header.stamp.nsec/1e9 - old_stamp_;
  uav_state_.timestamp = odom->header.stamp;

  // store pose
  Eigen::Affine3d current_pose;
  tf::poseMsgToEigen(odom->pose.pose, current_pose);

  uav_state_.position = current_pose.translation();
  uav_state_.orientation = current_pose.linear();
  uav_state_.yaw = std::atan2(2 * (uav_state_.orientation.w() * uav_state_.orientation.z() +
                              uav_state_.orientation.x() * uav_state_.orientation.y()),
                              1 - 2 * (uav_state_.orientation.y() * uav_state_.orientation.y() +
                              uav_state_.orientation.z() * uav_state_.orientation.z()));
  uav_state_.yaw_vel = odom->twist.twist.angular.z;

  // store current_velocity_ from before, before changing it
  Eigen::Vector3d v0, v1;
  v0 = uav_state_.velocity;

  // store current velocity (after transforming it from msg to an Eigen vector)
  tf::vectorMsgToEigen(odom->twist.twist.linear, uav_state_.velocity);
  uav_state_.velocity = uav_state_.orientation.toRotationMatrix() * uav_state_.velocity;

  double t = duration;
  v1 = uav_state_.velocity;

  // calculate acceleration if it shoule be calc by odom; a = (v1 - v0) / t
  if(first_odom_cb_){
    uav_state_.acceleration << 0.0, 0.0, 0.0;
    first_odom_cb_ = false;
  }
  else{
    uav_state_.acceleration = (v1 - v0) / t;
  }

  // store stamp of current msg
  old_stamp_ = odom->header.stamp.sec + odom->header.stamp.nsec/1e9;
}

Eigen::Matrix<double, 2, 1> RMPPlanner::get_u_v(Eigen::Vector3d target, bool use_odom, bool is_vel){
  // camera frame has  x in the horizontal, y in the vertical and z out of the camera
  // define roll, pitch, yaw angles of the camera coordinates w.r.t. the body coordinates
  double roll_C_B, pitch_C_B, yaw_C_B;
  roll_C_B = 0 * M_PI / 180;
  pitch_C_B = 0 * M_PI / 180;
  yaw_C_B = 0 * M_PI / 180;

  // define translational offset of the camera from body origin
  Eigen::Vector3d t_C_B;
  t_C_B << 0.0, 0.0, 0.0;

  // define roll, pitch, yaw angles of the body coordinates w.r.t. the world coordinates
  double roll_B_W, pitch_B_W, yaw_B_W;
  if(use_odom){
    Eigen::Quaterniond q = uav_state_.orientation;
    roll_B_W = atan2(2.0 * (q.x() * q.y() + q.w() * q.z()),
    q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    pitch_B_W = asin(-2.0 * (q.x() * q.z() - q.w() * q.y()));
    yaw_B_W = atan2(2.0 * (q.y() * q.z() + q.w() * q.x()),
    q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
  }
  else{
    roll_B_W = cur_roll_;
    pitch_B_W = cur_pitch_;
    yaw_B_W = cur_yaw_;
  }

  // define translational offset of the body from world origin
  Eigen::Vector3d t_B_W;
  if(use_odom){
    t_B_W = uav_state_.position;
  }
  else{
    t_B_W = cur_pos_;
  }

  // define Rotation matrices
  Eigen::Matrix3d R_C_B, R_B_W;

  R_C_B << cos(pitch_C_B) * cos(yaw_C_B),
            cos(pitch_C_B) * sin(yaw_C_B),
            -sin(pitch_C_B),
           sin(roll_C_B) * sin(pitch_C_B) * cos(yaw_C_B) - cos(roll_C_B) * sin(yaw_C_B),
            sin(roll_C_B) * sin(pitch_C_B) * sin(yaw_C_B) + cos(roll_C_B) * cos(yaw_C_B),
            sin(roll_C_B) * cos(pitch_C_B),
           cos(roll_C_B) * sin(pitch_C_B) * cos(yaw_C_B) + sin(roll_C_B) * sin(yaw_C_B),
            cos(roll_C_B) * sin(pitch_C_B) * sin(yaw_C_B) - sin(roll_C_B) * cos(yaw_C_B),
            cos(roll_C_B) * cos(pitch_C_B);

  R_B_W << cos(pitch_B_W) * cos(yaw_B_W),
            cos(pitch_B_W) * sin(yaw_B_W),
            -sin(pitch_B_W),
           sin(roll_B_W) * sin(pitch_B_W) * cos(yaw_B_W) - cos(roll_B_W) * sin(yaw_B_W),
            sin(roll_B_W) * sin(pitch_B_W) * sin(yaw_B_W) + cos(roll_B_W) * cos(yaw_B_W),
            sin(roll_B_W) * cos(pitch_B_W),
           cos(roll_B_W) * sin(pitch_B_W) * cos(yaw_B_W) + sin(roll_B_W) * sin(yaw_B_W),
            cos(roll_B_W) * sin(pitch_B_W) * sin(yaw_B_W) - sin(roll_B_W) * cos(yaw_B_W),
            cos(roll_B_W) * cos(pitch_B_W);

  Eigen::Vector3d target_C;

  if(!is_vel){
    // define translational parts of Transformation matrices
    Eigen::Vector3d trans_C_B = -R_C_B * t_C_B;
    Eigen::Vector3d trans_B_W = -R_B_W * t_B_W;

    // define Transformation matrices
    Eigen::Affine3d T_C_B, T_B_W;
    T_C_B.linear() = R_C_B;
    T_C_B.translation() = trans_C_B;
    T_B_W.linear() = R_B_W;
    T_B_W.translation() = trans_B_W;

    // calculate target pose in camera frame
    Eigen::Affine3d target_pose_W, target_pose_C;
    target_pose_W.translation() = target;
    target_pose_C = T_C_B * T_B_W * target_pose_W;
    target_C = target_pose_C.translation();
  }
  else{
    target_C = R_C_B * R_B_W * target;
  }

  // set up camera matrix
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(3, 3);
  K(0, 0) = f_x_;
  K(1, 1) = f_y_;
  K(2, 2) = 1;
  K(0, 2) = u_0_;
  K(1, 2) = v_0_;

  // get u, v from camera matrix and target position in camera frame
  Eigen::Vector3d u_v;
  u_v = K * target_C;

  // take normalization from u_v position
  if(!is_vel){
    normalize_u_v_ = u_v[2];
  }
  else{
    u_v[2] = normalize_u_v_;
  }

  // normalize u and v
  Eigen::Matrix<double, 2, 1> u_v_normalized;
  u_v_normalized[0] = u_v[0] / u_v[2];
  u_v_normalized[1] = u_v[1] / u_v[2];

  // std::cout << "(" << u_v_normalized[0] << ", " << u_v_normalized[1] << ")" << std::endl;

  return u_v_normalized;
}

bool RMPPlanner::TakeOff(){
  ROS_WARN_STREAM("MP ----- TAKE OFF");

  Eigen::Vector3d take_off_pos;
  take_off_pos << 0.0, 0.0, 10.0;

  geometry_msgs::PoseStamped take_off_pose_msg;
  take_off_pose_msg.pose.position.x = take_off_pos[0];
  take_off_pose_msg.pose.position.y = take_off_pos[1];
  take_off_pose_msg.pose.position.z = take_off_pos[2];
  take_off_pose_msg.pose.orientation.x = 0.0;
  take_off_pose_msg.pose.orientation.y = 0.0;
  take_off_pose_msg.pose.orientation.z = 0.0;
  take_off_pose_msg.pose.orientation.w = 1.0;
  take_off_pose_msg.header.stamp = ros::Time::now();

  pose_pub_.publish(take_off_pose_msg);

  if(accuracy_reached(take_off_pos, 10)){
    ROS_WARN_STREAM("MP ----- TAKE OFF POSITION REACHED");
    return true;
  }
  else{
    ROS_WARN_STREAM("MP ----- TAKE OFF POSITION NOT REACHED AFTER 10 SECONDS");
    return false;
  }
}

bool RMPPlanner::Follow(){
  ROS_WARN_STREAM("MP ----- FOLLOW");
  stop_sub_ = false;
  sub_follow_ = nh_.subscribe("victim_trajectory", 10, &RMPPlanner::follow_callback, this);
  while((!(as_.isPreemptRequested())) && ros::ok() && !stop_sub_){}
  sub_follow_.shutdown();
  if(as_.isPreemptRequested()){
    as_.setPreempted();
  }
  if(stop_sub_){
    return true;
  }
}

void RMPPlanner::follow_callback(const trajectory_msgs::MultiDOFJointTrajectory& victim_traj){
  // get target position and target velocity
  Eigen::Vector3d target_pos, target_pos_2, target_vel;
  tf::vectorMsgToEigen(victim_traj.points[0].transforms[0].translation, target_pos);
  tf::vectorMsgToEigen(victim_traj.points[1].transforms[0].translation, target_pos_2);
  target_vel = target_pos_2 - target_pos;
  target_vel /= victim_traj.points[1].time_from_start.toSec() - victim_traj.points[0].time_from_start.toSec();

  // define geometry
  using geometry = rmpcpp::CartesianCameraGeometry;

  // create a variable for the task space geometry in camera coordinates
  geometry task_space_geometry;

  // create a container for all policies in the geometry of the task space
  rmpcpp::PolicyContainer<geometry> container(task_space_geometry);

  // create simple target policy in task space with a target defined with theta, rho, z
  const int task_space_dimension = geometry::K;
  Eigen::Matrix<double, task_space_dimension, 1> target;
  target[0] = 0;
  target[1] = 0;
  double alpha, beta, c;
  alpha = 1.0;
  beta = 3.0;
  c = 0.05;
  rmpcpp::SimpleTargetPolicy<task_space_dimension> target_policy(target, alpha, beta, c);

  // create a metric a for the target policy
  geometry::MatrixX A_target(geometry::MatrixX::Zero());

  // fill in the metric A and set it in the policy
  A_target(0, 0) = 1.0;
  A_target(1, 1) = 1.0;
  target_policy.setA(A_target);

  // add the policy into the container
  container.addPolicy(&target_policy);

  // create a trapezoidal integrator with the container
  rmpcpp::TrapezoidalIntegrator<geometry> integrator(container);

  // reset integrator with current pos and vel for x, y, z, yaw
  const int config_space_dimension = geometry::D;
  Eigen::Matrix<double, config_space_dimension, 1> pos, vel;
  pos[0] = uav_state_.position[0];
  pos[1] = uav_state_.position[1];
  pos[2] = uav_state_.position[2];
  pos[3] = uav_state_.yaw;
  vel[0] = uav_state_.velocity[0];
  vel[1] = uav_state_.velocity[1];
  vel[2] = uav_state_.velocity[2];
  vel[3] = uav_state_.yaw_vel;
  integrator.resetTo(pos, vel);

  // create msg and define sampling interval & MPC_horizon
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  double dt = 0.01;
  double t = 0;
  double MPC_horizon = 2.0;
  Eigen::Matrix<double, config_space_dimension, 1> traj_pos, traj_vel, traj_acc;

  // set K matrix elements for jacobian calculation
  Eigen::Matrix<double, 4, 1> elems;
  elems[0] = f_x_;
  elems[1] = f_y_;
  elems[2] = u_0_;
  elems[3] = v_0_;
  task_space_geometry.SetK(elems);

  // set target position in world frame for jacobian calculation
  task_space_geometry.SetTargetPos(target_pos);

  // set current position in Q jacobian calculation
  task_space_geometry.setQ(pos);

  Eigen::Matrix<double, task_space_dimension, 1> u_v, u_v_old, u_v_dot, f_u_v;
  // integrate until drone is at rest and store pos, vel & acc in msg


  double xy_distance = sqrt((target_pos - uav_state_.position)[0] * (target_pos - uav_state_.position)[0] +
                            (target_pos - uav_state_.position)[1] * (target_pos - uav_state_.position)[1]);
  if(xy_distance > 0.01){
    // integrate 2s into the future
    for(double t = 0.0; t <= MPC_horizon + dt; t += dt){
      bool use_odom, is_vel;
      Eigen::Vector3d relative_vel;
      if(t == 0){
        use_odom = true;
        relative_vel = target_vel - uav_state_.velocity;
      }
      else{
        // update Q in the Jacobian
        task_space_geometry.setQ(traj_pos);

        // update current state for calculation of u and v
        update_cur_state(traj_pos, traj_acc);

        // calculate u and v
        use_odom = false;

        // calculate relative velocity
        Eigen::Vector3d cur_vel;
        cur_vel[0] = traj_vel[0];
        cur_vel[1] = traj_vel[1];
        cur_vel[2] = traj_vel[2];
        relative_vel = target_vel - cur_vel;
      }

      is_vel = false;
      u_v = get_u_v(target_pos, use_odom, is_vel);
      is_vel = true;
      u_v_dot = get_u_v(target_vel, use_odom, is_vel);

      integrator.setX(u_v, u_v_dot);
      integrator.forwardIntegrate(dt);
      integrator.getState(&traj_pos, &traj_vel, &traj_acc);
      this->create_traj_point(t, traj_pos, traj_vel, traj_acc, &trajectory_msg);
      this->publish_u_v_viz_msg(target_policy.getAccField(), u_v, u_v_dot, t);
    }

    // publish trajectory
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_pub_.publish(trajectory_msg);
  }
  else{
    stop_sub_ = true;
  }
}

void RMPPlanner::publish_u_v_viz_msg(Eigen::Matrix<double, 2, 1> f_u_v, Eigen::Matrix<double, 2, 1> u_v, Eigen::Matrix<double, 2, 1> u_v_dot, double t){
  drogone_msgs_rmp::AccFieldUV msg;
  msg.f_u = f_u_v[0];
  msg.f_v = f_u_v[1];
  msg.x_u = u_v[0];
  msg.x_v = u_v[1];
  msg.x_dot_u = u_v_dot[0];
  msg.x_dot_v = u_v_dot[1];
  msg.header.stamp = ros::Time(t);
  f_u_v_pub_.publish(msg);
}

void RMPPlanner::create_traj_point(double t, Eigen::Matrix<double, 4, 1> traj_pos, Eigen::Matrix<double, 4, 1> traj_vel,
                                    Eigen::Matrix<double, 4, 1> traj_acc, trajectory_msgs::MultiDOFJointTrajectory *msg){
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
  geometry_msgs::Vector3 pos_msg, vel_msg, acc_msg, ang_vel_msg, ang_acc_msg;
  geometry_msgs::Quaternion quat_msg;
  pos_msg.x = traj_pos[0];
  pos_msg.y = traj_pos[1];
  pos_msg.z = traj_pos[2];
  vel_msg.x = traj_vel[0];
  vel_msg.y = traj_vel[1];
  vel_msg.z = traj_vel[2];
  acc_msg.x = traj_acc[0];
  acc_msg.y = traj_acc[1];
  acc_msg.z = traj_acc[2];
  Eigen::Quaterniond quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(traj_pos[3], Eigen::Vector3d::UnitZ());
  quat_msg.x = quat.x();
  quat_msg.y = quat.y();
  quat_msg.z = quat.z();
  quat_msg.w = quat.w();
  ang_vel_msg.x = 0;
  ang_vel_msg.y = 0;
  ang_vel_msg.z = traj_vel[3];
  ang_acc_msg.x = 0;
  ang_acc_msg.y = 0;
  ang_acc_msg.z = traj_acc[3];

  geometry_msgs::Transform transform_pos_msg;
  geometry_msgs::Twist twist_vel_msg, twist_acc_msg;
  transform_pos_msg.translation = pos_msg;
  twist_vel_msg.linear = vel_msg;
  twist_vel_msg.angular = ang_vel_msg;
  twist_acc_msg.linear = acc_msg;
  twist_acc_msg.angular = ang_acc_msg;
  trajectory_point_msg.transforms.push_back(transform_pos_msg);
  trajectory_point_msg.velocities.push_back(twist_vel_msg);
  trajectory_point_msg.accelerations.push_back(twist_acc_msg);
  trajectory_point_msg.time_from_start = ros::Duration(t);
  msg->points.push_back(trajectory_point_msg);
}

void RMPPlanner::update_cur_state(Eigen::Matrix<double, 4, 1> pos, Eigen::Matrix<double, 4, 1> acc){
  double yaw = pos[3];

  double a_x_B = acc[0] * cos(yaw) + acc[1] * sin(yaw);
  double a_y_B = acc[1] * cos(yaw) - acc[0] * sin(yaw);
  double a_z_B = acc[2];

  // double roll = atan2(a_y_B, a_z_B + 9.81);
  // double pitch = atan2(a_x_B, a_z_B + 9.81);
  double roll = 0;
  double pitch = 0;
  // ROS_WARN_STREAM(roll * 180 / M_PI);
  // ROS_WARN_STREAM(pitch * 180 / M_PI);
  // ROS_WARN_STREAM(" ");

  // std::cout << "pitch:        " << pitch * 180 / M_PI << std::endl;
  // std::cout << "x-pos:        " << pos[0] << std::endl;

  cur_roll_ = roll;
  cur_pitch_ = pitch;
  cur_yaw_ = yaw;
  cur_pos_[0] = pos[0];
  cur_pos_[1] = pos[1];
  cur_pos_[2] = pos[2];
}

bool RMPPlanner::Land(){
  ROS_WARN_STREAM("MP ----- LAND");

  Eigen::Vector3d land_pos;
  land_pos << 0.0, 0.0, 2.0;

  geometry_msgs::PoseStamped land_pose_msg;
  land_pose_msg.pose.position.x = land_pos[0];
  land_pose_msg.pose.position.y = land_pos[1];
  land_pose_msg.pose.position.z = land_pos[2];
  land_pose_msg.header.stamp = ros::Time::now();

  pose_pub_.publish(land_pose_msg);

  if(accuracy_reached(land_pos, 10)){
    ROS_WARN_STREAM("MP ----- LAND POSITION REACHED");
    return true;
  }
  else{
    ROS_WARN_STREAM("MP ----- LAND POSITION NOT REACHED AFTER 10 SECONDS");
    return false;
  }
}

void RMPPlanner::server_callback(const drogone_action_rmp::FSMGoalConstPtr& goal){

  std::string goal_mode = goal->new_mode;

  if (goal_mode == "TakeOff"){
    // if user has cancelled/preempted this
    if(as_.isPreemptRequested() || !ros::ok()){
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
    }

    if(this->TakeOff()){
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded();
    }
    else{
      ROS_INFO("%s: Aborted", action_name_.c_str());
      as_.setAborted();
    }
  }

  if (goal_mode == "Follow"){
    // if user has cancelled/preempted this
    if(as_.isPreemptRequested() || !ros::ok()){
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
    }

    if(this->Follow()){
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded();
    }
  }

  if (goal_mode == "Land")
  {
    // if user has cancelled/preempted this
    if(as_.isPreemptRequested() || !ros::ok()){
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
    }

    if(this->Land()){
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded();
    }
    else{
      ROS_INFO("%s: Aborted", action_name_.c_str());
      ROS_WARN("LANDING ABORTED: LAND MANUALY");
      as_.setAborted();
    }
  }
}

bool RMPPlanner::accuracy_reached(const Eigen::Vector3d& goal_pos, double waiting_time){
  // determine position where drone currently is
  Eigen::Vector3d position_now;
  position_now = uav_state_.position;

  // starting time to later watch how much time passed from here
  ros::Time begin = ros::Time::now();
  double begin_secs = begin.toSec();

  double distance = std::sqrt(std::pow((goal_pos[0] - position_now[0]), 2) +
                              std::pow((goal_pos[1] - position_now[1]), 2) +
                              std::pow((goal_pos[2] - position_now[2]), 2));

  // while loop which ends, if we reach the desired position
  while(distance > accuracy_){
    // check how much time passed since we started. if >10 return false (abbort)
    ros::spinOnce();
    ros::Time end = ros::Time::now();
    double end_secs = end.toSec();
    double duration_secs = end_secs - begin_secs;
    if(duration_secs > waiting_time){
      return false;
      break;
    }
    else{
      position_now = uav_state_.position;
      distance = std::sqrt(std::pow((goal_pos[0] - position_now[0]), 2) +
                           std::pow((goal_pos[1] - position_now[1]), 2) +
                           std::pow((goal_pos[2] - position_now[2]), 2));
      continue;
    }
  }
  // return true (succeed) when we break out of the while loop and haven't returned false (abborted) already
  return true;
}

} // namespace drogone_rmp_planner
