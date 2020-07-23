#include <drogone_motion_planner/rmp_planner.h>

namespace drogone_rmp_planner {

RMPPlanner::RMPPlanner(std::string name, ros::NodeHandle nh, ros::NodeHandle nh_private):
  as_(nh, name, boost::bind(&RMPPlanner::server_callback, this, _1), false),
  action_name_(name),
  nh_(nh),
  nh_private_(nh_private),
  first_odom_cb_(true) {

    // publisher for trajectory to drone
    pub_traj_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);

    // publisher for trajectory to drone
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(
        mav_msgs::default_topics::COMMAND_POSE, 1);

    // publisher for f_u_v for visualization
    pub_analyzation_ = nh_.advertise<drogone_msgs_rmp::AccFieldWithState>("acc_field_analyzation", 0);

    // // publisher for image acc plotting
    // new_temporary_pub_ = nh_.advertise<geometry_msgs::PoseArray>("plot_image_acc", 0);

    // subscriber to Odometry for physical uav state
    sub_odom_ =
        nh_.subscribe("uav_pose", 1, &RMPPlanner::uavOdomCallback, this);

    // subscriber to Trajectory for trajectory uav state
    sub_traj_ =
        nh_.subscribe("command/trajectory", 1, &RMPPlanner::uavTrajCallback, this);

    // load params
    if(!nh_private_.getParam("f_x", pinhole_constants_.f_x)){
      ROS_ERROR("failed to load f_x");
    }
    if(!nh_private_.getParam("f_y", pinhole_constants_.f_y)){
      ROS_ERROR("failed to load f_y");
    }
    if(!nh_private_.getParam("u_0", pinhole_constants_.u_0)){
      ROS_ERROR("failed to load u_0");
    }
    if(!nh_private_.getParam("v_0", pinhole_constants_.v_0)){
      ROS_ERROR("failed to load v_0");
    }

    if(!nh_private_.getParam("roll", camera_mounting_.roll)){
      ROS_ERROR("failed to load roll");
    }
    if(!nh_private_.getParam("pitch", camera_mounting_.pitch)){
      ROS_ERROR("failed to load pitch");
    }
    if(!nh_private_.getParam("yaw", camera_mounting_.yaw)){
      ROS_ERROR("failed to load yaw");
    }

    std::vector<double> cam_trans;
    if (!nh_private_.getParam("translation", cam_trans)) {
      ROS_ERROR("failed to load translation.");
    }
    camera_mounting_.translation << cam_trans[0], cam_trans[1], cam_trans[2];

    if(!nh_private_.getParam("A_d", A_d_)){
      ROS_ERROR("failed to load A_d");
    }
    if(!nh_private_.getParam("A_u", A_u_)){
      ROS_ERROR("failed to load A_u");
    }
    if(!nh_private_.getParam("A_v", A_v_)){
      ROS_ERROR("failed to load A_v");
    }

    if(!nh_private_.getParam("frequency", frequency_)){
      ROS_ERROR("failed to load frequency");
    }
    if(!nh_private_.getParam("MPC_horizon", MPC_horizon_)){
      ROS_ERROR("failed to load MPC_horizon");
    }
    if(!nh_private_.getParam("sampling_interval", sampling_interval_)){
      ROS_ERROR("failed to load sampling_interval");
    }

    if(!nh_private_.getParam("a_max", a_max_W_)){
      ROS_ERROR("failed to load a_max");
    }

    transformer_.setCameraConfig(pinhole_constants_, camera_mounting_);

    as_.start();
}

// Callback to get current Pose of UAV
void RMPPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  // calculate the time between the 2 measurements based on stamps
  double duration = odom->header.stamp.sec + odom->header.stamp.nsec/1e9 - old_stamp_;
  physical_uav_state_.timestamp = odom->header.stamp;

  // store pose
  Eigen::Affine3d current_pose;
  tf::poseMsgToEigen(odom->pose.pose, current_pose);

  physical_uav_state_.position = current_pose.translation();
  physical_uav_state_.orientation = current_pose.linear();
  physical_uav_state_.yaw = std::atan2(2.0 * (physical_uav_state_.orientation.w() * physical_uav_state_.orientation.z() +
                                       physical_uav_state_.orientation.x() * physical_uav_state_.orientation.y()),
                                       1.0 - 2.0 * (physical_uav_state_.orientation.y() * physical_uav_state_.orientation.y() +
                                       physical_uav_state_.orientation.z() * physical_uav_state_.orientation.z()));

  // store current_velocity_ from before, before changing it
  Eigen::Vector3d v0, v1;
  double v0_yaw, v1_yaw;
  v0 = physical_uav_state_.velocity;
  v0_yaw = physical_uav_state_.yaw_vel;

  // store current velocity (after transforming it from msg to an Eigen vector)
  tf::vectorMsgToEigen(odom->twist.twist.linear, physical_uav_state_.velocity);
  physical_uav_state_.velocity = physical_uav_state_.orientation.toRotationMatrix() * physical_uav_state_.velocity;
  physical_uav_state_.yaw_vel = odom->twist.twist.angular.z;

  double t = duration;
  v1 = physical_uav_state_.velocity;
  v1_yaw = physical_uav_state_.yaw_vel;


  // calculate acceleration if it shoule be calc by odom; a = (v1 - v0) / t
  if(first_odom_cb_){
    physical_uav_state_.acceleration << 0.0, 0.0, 0.0;
    physical_uav_state_.yaw_acc = 0.0;
    first_odom_cb_ = false;
  }
  else{
    physical_uav_state_.acceleration = (v1 - v0) / t;
    physical_uav_state_.yaw_acc = (v1_yaw - v0_yaw) / t;
  }

  // store stamp of current msg
  old_stamp_ = odom->header.stamp.sec + odom->header.stamp.nsec/1e9;
}

void RMPPlanner::uavTrajCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& traj){
  int point_num = 1 / frequency_ / sampling_interval_ - 1;
  tf::vectorMsgToEigen(traj->points[point_num].transforms[0].translation, trajectory_uav_state_.position);
  tf::vectorMsgToEigen(traj->points[point_num].velocities[0].linear, trajectory_uav_state_.velocity);
  tf::vectorMsgToEigen(traj->points[point_num].accelerations[0].linear, trajectory_uav_state_.acceleration);
  // tf::quaternionMsgToEigen(traj->points[point_num].transforms[0].rotation, trajectory_uav_state_.orientation);
  trajectory_uav_state_.yaw = std::atan2(2.0 * (trajectory_uav_state_.orientation.w() * trajectory_uav_state_.orientation.z() +
                                         trajectory_uav_state_.orientation.x() * trajectory_uav_state_.orientation.y()),
                                         1.0 - 2.0 * (trajectory_uav_state_.orientation.y() * trajectory_uav_state_.orientation.y() +
                                         trajectory_uav_state_.orientation.z() * trajectory_uav_state_.orientation.z()));
  trajectory_uav_state_.yaw_vel = traj->points[point_num].velocities[0].angular.z;
  trajectory_uav_state_.yaw_acc = traj->points[point_num].accelerations[0].angular.z;

  double roll, pitch, yaw;
  yaw = trajectory_uav_state_.yaw;
  double a_x_B = trajectory_uav_state_.acceleration[0] * cos(yaw) + trajectory_uav_state_.acceleration[1] * sin(yaw);
  double a_y_B = trajectory_uav_state_.acceleration[1] * cos(yaw) - trajectory_uav_state_.acceleration[0] * sin(yaw);
  double a_z_B = trajectory_uav_state_.acceleration[2];
  roll = atan2(a_y_B, a_z_B + 9.81);
  pitch = atan2(a_x_B, a_z_B + 9.81);
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  trajectory_uav_state_.orientation = rollAngle * yawAngle * pitchAngle;
};



bool RMPPlanner::TakeOff(){
  ROS_WARN_STREAM("MP ----- TAKE OFF");

  Eigen::Vector3d take_off_pos;
  take_off_pos << 0.0, 0.0, 10.0;

  geometry_msgs::PoseStamped take_off_pose_msg;
  take_off_pose_msg.pose.position.x = take_off_pos[0];
  take_off_pose_msg.pose.position.y = take_off_pos[1];
  take_off_pose_msg.pose.position.z = take_off_pos[2];
  take_off_pose_msg.header.stamp = ros::Time::now();

  pub_pose_.publish(take_off_pose_msg);

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
  follow_counter_ = 0;
  sub_follow_ = nh_.subscribe("victim_pos", 10, &RMPPlanner::follow_callback, this);

  // lock the mutex and wait until it is unlocked (in catch_traj, once last traj is calculated)
  std::unique_lock<std::mutex> uLock(mutex_);
  cond_var_.wait(uLock);
  return true;
}

void RMPPlanner::follow_callback(const drogone_msgs_rmp::target_detection& victim_pos){
  chrono_t1_ = std::chrono::high_resolution_clock::now();
  ROS_WARN_STREAM("NEW REPLAN");

  follow_counter_ += 1;
  UAVState follow_uav_state;
  if(follow_counter_ == 1){
    follow_uav_state = physical_uav_state_;
    // ROS_WARN_STREAM("MP ----- USING PHYSICAL FOR UAV STATE");
  }
  else{
    follow_uav_state = trajectory_uav_state_;
    // follow_uav_state = physical_uav_state_;
    // ROS_WARN_STREAM("MP ----- USING TRAJECTORY FOR UAV STATE");
  }

  // define geometries
  using x_geometry = rmpcpp::XGeom;
  using y_geometry = rmpcpp::YGeom;

  // create a variables for the task space geometries
  x_geometry task_space_x_geometry;
  y_geometry task_space_y_geometry;

  // create containers for policies in the same geometry of the task spaces
  rmpcpp::PolicyContainer<x_geometry> x_container(task_space_x_geometry);
  rmpcpp::PolicyContainer<y_geometry> y_container(task_space_y_geometry);

  // create simple target policy in camera task space
  Eigen::Matrix<double, 1, 1> x_target;
  x_target[0] = 3;
  double x_alpha, x_beta, x_c;
  x_alpha = 1.0;
  x_beta = 0.5;
  x_c = 0.05;
  rmpcpp::SimpleXYTargetPolicy<1> x_target_policy(x_target, x_alpha, x_beta, x_c);

  // create simple target policy in distance task space
  Eigen::Matrix<double, 1, 1> y_target;
  y_target[0] = 3;
  double y_alpha, y_beta, y_c;
  y_alpha = 1.0;
  y_beta = 0.5;
  y_c = 0.05;
  rmpcpp::SimpleXYTargetPolicy<1> y_target_policy(y_target, y_alpha, y_beta, y_c);

  // create metrics for the target policies
  x_geometry::MatrixX A_target_x(x_geometry::MatrixX::Zero());
  y_geometry::MatrixX A_target_y(y_geometry::MatrixX::Zero());

  // fill the metrics and set them in the policies
  A_target_x(0, 0) = 5.0;
  x_target_policy.setA(A_target_x);
  A_target_y(0, 0) = 1.0;
  y_target_policy.setA(A_target_y);

  // add the policies into the container
  x_container.addPolicy(&x_target_policy);
  y_container.addPolicy(&y_target_policy);

  // create a trapezoidal integrator with the containers
  rmpcpp::TrapezoidalIntegrator<x_geometry, y_geometry> integrator(x_container, y_container);

  // reset integrator with current pos and vel for x, y, z, yaw
  const int config_space_dimension = x_geometry::D;
  Eigen::Matrix<double, config_space_dimension, 1> pos, vel, acc;
  pos[0] = follow_uav_state.position[0];
  pos[1] = follow_uav_state.position[1];
  pos[2] = follow_uav_state.position[2];
  pos[3] = follow_uav_state.yaw;
  vel[0] = follow_uav_state.velocity[0];
  vel[1] = follow_uav_state.velocity[1];
  vel[2] = follow_uav_state.velocity[2];
  vel[3] = follow_uav_state.yaw_vel;
  acc[0] = follow_uav_state.acceleration[0];
  acc[1] = follow_uav_state.acceleration[1];
  acc[2] = follow_uav_state.acceleration[2];
  acc[3] = follow_uav_state.yaw_acc;
  integrator.resetTo(pos, vel, acc);

  // create msg and define sampling interval & MPC_horizon
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  double t = 0;
  Eigen::Matrix<double, config_space_dimension, 1> traj_pos, traj_vel, traj_acc;
  Eigen::Matrix<double, 1, 1> x, x_dot, f_x;
  Eigen::Matrix<double, 1, 1> y, y_dot, f_y;

  x[0] = follow_uav_state.position[0];
  y[0] = follow_uav_state.position[1];
  x_dot[0] = follow_uav_state.velocity[0];
  y_dot[0] = follow_uav_state.velocity[1];

  // integrate 2s into the future
  for(double t = 0.0; t <= MPC_horizon_ + sampling_interval_; t += sampling_interval_){
    if(t > 0.0){
      x[0] = traj_pos[0];
      y[0] = traj_pos[1];
      x_dot[0] = traj_vel[0];
      y_dot[0] = traj_vel[1];
    }

    // integrate the policy with the (u, v, d) calculated
    integrator.setX(x, x_dot, y, y_dot);
    integrator.forwardIntegrate(sampling_interval_);
    integrator.getState(&traj_pos, &traj_vel, &traj_acc);

    // create trajectory point from current state and append it to the trajectory msg
    this->create_traj_point(t, traj_pos, traj_vel, traj_acc, &trajectory_msg);

    // // publish acc field and state for analyzation purposes
    // double t_for_msg = victim_pos.header.stamp.toSec() + t;
    // this->publish_analyzation_msg(camera_target_policy.getAccField(), u_v, u_v_dot,
    //                               distance_target_policy.getAccField(), d, d_dot, t_for_msg);
  }

  // publish trajectory
  trajectory_msg.header.stamp = ros::Time::now();
  pub_traj_.publish(trajectory_msg);

  chrono_t2_ = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(chrono_t2_ - chrono_t1_).count();
  // std::cout << "Duration: " << duration << " microseconds" << std::endl;

  if(follow_counter_ * 0.1 > 20){
    ros::Duration(t).sleep();

    // take ownership of the mutex (unlock mutex from the lock in Follow())
    std::lock_guard<std::mutex> guard(mutex_);
    // notify the condition variable to stop waiting (in Follow())
    cond_var_.notify_one();
    // stop subscriber
    sub_follow_.shutdown();
  }

}

void RMPPlanner::create_traj_point(double t, Eigen::Matrix<double, 4, 1> traj_pos,
                                   Eigen::Matrix<double, 4, 1> traj_vel,
                                   Eigen::Matrix<double, 4, 1> traj_acc,
                                   trajectory_msgs::MultiDOFJointTrajectory *msg){
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
  Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(traj_pos[3], Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quat = rollAngle * yawAngle * pitchAngle;
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
  transform_pos_msg.rotation = quat_msg;
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

void RMPPlanner::publish_analyzation_msg(Eigen::Matrix<double, 2, 1> f_u_v,
                                         Eigen::Matrix<double, 2, 1> u_v,
                                         Eigen::Matrix<double, 2, 1> u_v_dot,
                                         Eigen::Matrix<double, 1, 1> f_d,
                                         Eigen::Matrix<double, 1, 1> d,
                                         Eigen::Matrix<double, 1, 1> d_dot,
                                         double t){
  drogone_msgs_rmp::AccFieldWithState msg;
  msg.f_u = f_u_v[0];
  msg.f_v = f_u_v[1];
  msg.f_d = f_d[0];
  msg.x_u = u_v[0];
  msg.x_v = u_v[1];
  msg.x_d = d[0];
  msg.x_dot_u = u_v_dot[0];
  msg.x_dot_v = u_v_dot[1];
  msg.x_dot_d = d_dot[0];
  msg.header.stamp = ros::Time(t);
  pub_analyzation_.publish(msg);
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

  pub_pose_.publish(land_pose_msg);

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

bool RMPPlanner::accuracy_reached(const Eigen::Vector3d& goal_pos, double max_time){
  // create ros rate with the frequency to check whether position was reached
  ros::Rate r(10);

  // check if position was reached until max time
  double time_since_pub = 0;
  while(time_since_pub < max_time){
    // determine position where drone currently is
    Eigen::Vector3d position_now;
    position_now = physical_uav_state_.position;

    // determine current distance to target
    double distance = std::sqrt(std::pow((goal_pos[0] - position_now[0]), 2) +
                                std::pow((goal_pos[1] - position_now[1]), 2) +
                                std::pow((goal_pos[2] - position_now[2]), 2));

    // check if we're already close enough
    if(distance < accuracy_){
      return true;
    }
    r.sleep();
    time_since_pub += 0.1;
  }

  // if we come outta while without having returned true already, we didn't reach the point in time
  return false;
}

} // namespace drogone_rmp_planner
