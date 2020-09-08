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
    if(!nh_private_.getParam("w", image_width_px_)){
      ROS_ERROR("failed to load w");
    }
    if(!nh_private_.getParam("h", image_height_px_)){
      ROS_ERROR("failed to load h");
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

  physical_uav_state_.pose = current_pose;
  Eigen::Quaterniond cur_ori(physical_uav_state_.pose.linear());
  physical_uav_state_.yaw = std::atan2(2.0 * (cur_ori.w() * cur_ori.z() +
                                       cur_ori.x() * cur_ori.y()),
                                       1.0 - 2.0 * (cur_ori.y() * cur_ori.y() +
                                       cur_ori.z() * cur_ori.z()));

  // store current_velocity_ from before, before changing it
  Eigen::Vector3d v0, v1;
  double v0_yaw, v1_yaw;
  v0 = physical_uav_state_.velocity;
  v0_yaw = physical_uav_state_.yaw_vel;

  // store current velocity (after transforming it from msg to an Eigen vector)
  tf::vectorMsgToEigen(odom->twist.twist.linear, physical_uav_state_.velocity);
  physical_uav_state_.velocity = physical_uav_state_.pose.linear() * physical_uav_state_.velocity;
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
  Eigen::Vector3d position;
  tf::vectorMsgToEigen(traj->points[point_num].transforms[0].translation, position);
  trajectory_uav_state_.pose.translation() = position;
  tf::vectorMsgToEigen(traj->points[point_num].velocities[0].linear, trajectory_uav_state_.velocity);
  tf::vectorMsgToEigen(traj->points[point_num].accelerations[0].linear, trajectory_uav_state_.acceleration);
  Eigen::Quaterniond cur_ori(trajectory_uav_state_.pose.linear());
  trajectory_uav_state_.yaw = std::atan2(2.0 * (cur_ori.w() * cur_ori.z() +
                                         cur_ori.x() * cur_ori.y()),
                                         1.0 - 2.0 * (cur_ori.y() * cur_ori.y() +
                                         cur_ori.z() * cur_ori.z()));
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
  trajectory_uav_state_.pose.linear() = (rollAngle * yawAngle * pitchAngle).toRotationMatrix();
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

bool RMPPlanner::SubDetection(){
  first_detection_ = true;
  sub_follow_ = nh_.subscribe("victim_pos", 10, &RMPPlanner::detection_callback, this);

  // lock the mutex and wait until it is unlocked (in catch_traj, once last traj is calculated)
  std::unique_lock<std::mutex> uLock(mutex_);
  cond_var_.wait(uLock);
  return true;
}

void RMPPlanner::detection_callback(const drogone_msgs_rmp::target_detection& victim_pos){
  // store the correct uav state in planning_uav_state_
  if(first_detection_){
    planning_uav_state_ = physical_uav_state_;
  }
  else{
    planning_uav_state_ = trajectory_uav_state_;
  }

  // stop planning if goal is reached
  if(victim_pos.d < 0.5){
    // take ownership of the mutex (unlock mutex from the lock in Follow())
    std::lock_guard<std::mutex> guard(mutex_);
    // notify the condition variable to stop waiting (in Follow())
    cond_var_.notify_one();
    // stop subscriber
    sub_follow_.shutdown();

    return;
  }

  // set target_passed_ false again if the target is passed the replanning stops anyway
  target_passed_ = false;

  // store time of detection
  time_of_last_detection_ = victim_pos.header.stamp;

  // calculate current target position
  Eigen::Affine3d uav_pose;
  uav_pose = physical_uav_state_.pose;
  transformer_.setMatrices(uav_pose);
  Eigen::Matrix<double, 3, 1> detection;
  detection << victim_pos.u, victim_pos.v, victim_pos.d;
  cur_target_pos_ = transformer_.PosImage2World(detection);

  // calculate approx velocity from current and last position
  if(first_detection_){
    cur_target_vel_ << 0.0, 0.0, 0.0;
  }
  else{
    cur_target_vel_ = (cur_target_pos_ - last_target_pos_) / (1 / frequency_);
  }

  /* SET WEIGHTS */
  // calculate u, v and d with roll = pitch = 0 assumption
  Eigen::Quaterniond q(planning_uav_state_.pose.linear());
  double roll, pitch, yaw;
  yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
              1.0 - 2.0 * (q.y() * q.y() - q.z() * q.z()));
  roll = 0;
  pitch = 0;
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  planning_uav_state_.pose.linear() = (rollAngle * yawAngle * pitchAngle).toRotationMatrix();   // update planning_uav_state_ with roll = pitch = 0
  transformer_.setMatrices(planning_uav_state_.pose);
  std::pair<Eigen::Matrix<double, 3, 1>, double> image_pair =
           transformer_.PosWorld2Image(cur_target_pos_);
  double u = abs(image_pair.first[0]);
  double v = abs(image_pair.first[1]);
  double d = abs(image_pair.first[2]);

  // default values
  a_u_ = 1.0;
  a_v_ = 1.0;
  // a_d_ = 2.0;
  a_d2g_ = 0.0;

  if(u < 150){
    a_d_ = 10.0;
    d_target_ = 0.0;
  }
  else{
    if(a_d_ == 10.0){
      if(u > 800){
        a_d_ = 2.0;
        d_target_ = 2.0;
      }
    }
    else{
      a_d_ = 2.0;
      d_target_ = 2.0;
    }
  }

  if(planning_uav_state_.pose.translation()[2] < 2){
    a_d2g_ = 100;
  }
  else{
    a_d2g_ = 0;
  }

  /* SET THE POLICY VARIABLES */
  // assuming beta = 3 is optimal for a target vel of 2 or smaller
  // and beta = 1 is optimal for a target vel of 5 or bigger
  // and in between the dependency is linear.
  // calculate beta as a linear interpolation of this
  if(cur_target_vel_.norm() <= 2){
    uv_beta_ = 3.0;
  }
  else if(cur_target_vel_.norm() >= 5){
    uv_beta_ = 1.0;
  }
  else{
    uv_beta_ = 3 + (cur_target_vel_.norm() - 2) * (1 - 3) / (5 - 2);
  }
  u_target_ = 0.0;
  v_target_ = 0.0;
  uv_c_ = 0.05;
  // d_target_ = 2.0;
  d_beta_ = 1.6;
  d_c_ = 0.5;
  d2g_alpha_ = 3;
  d2g_beta_ = 0.2 * d2g_alpha_;

  // plan Trajectory with current weights
  this->planTrajectory();

  // store current target position in the last target position variable for the next iteration
  last_target_pos_ = cur_target_pos_;

  // during the first detection, store that it's not the first detection anymore
  if(first_detection_){
    first_detection_ = false;
  }
}

void RMPPlanner::planTrajectory(){
  chrono_t1_ = std::chrono::high_resolution_clock::now();

  // define geometries
  using camera_geometry = rmpcpp::CartesianCameraGeometry;
  using distance_geometry = rmpcpp::DistanceGeometry;
  using distance2ground_geometry = rmpcpp::Distance2GroundGeometry;

  // create a variables for the task space geometries
  camera_geometry task_space_camera_geometry;
  distance_geometry task_space_distance_geometry;
  distance2ground_geometry task_space_distance2ground_geometry;

  // create containers for policies in the same geometry of the task spaces
  rmpcpp::PolicyContainer<camera_geometry> camera_container(task_space_camera_geometry);
  rmpcpp::PolicyContainer<distance_geometry> distance_container(task_space_distance_geometry);
  rmpcpp::PolicyContainer<distance2ground_geometry> distance2ground_container(task_space_distance2ground_geometry);

  // create target policy in camera task space
  const int task_space_camera_dimension = camera_geometry::K;
  Eigen::Matrix<double, task_space_camera_dimension, 1> camera_target;
  camera_target[0] = u_target_;
  camera_target[1] = v_target_;
  rmpcpp::CameraTargetPolicy<task_space_camera_dimension> camera_policy(camera_target, uv_beta_, uv_c_);

  // create target policy in distance task space
  const int task_space_distance_dimension = distance_geometry::K;
  Eigen::Matrix<double, task_space_distance_dimension, 1> distance_target;
  distance_target[0] = d_target_;
  rmpcpp::DistanceTargetPolicy<task_space_distance_dimension> distance_policy(distance_target, d_beta_, d_c_);

  // create target policy in distance2ground task space
  const int task_space_distance2ground_dimension = distance2ground_geometry::K;
  rmpcpp::Distance2GroundPolicy<task_space_distance2ground_dimension> distance2ground_policy(d2g_alpha_, d2g_beta_);

  // create metrics for the target policies
  camera_geometry::MatrixX A_camera(camera_geometry::MatrixX::Zero());
  distance_geometry::MatrixX A_distance(distance_geometry::MatrixX::Zero());
  distance2ground_geometry::MatrixX A_distance2ground(distance2ground_geometry::MatrixX::Zero());

  // fill the metric for the distance policy
  A_distance(0, 0) = a_d_;
  distance_policy.setA(A_distance);

  // fill the metric for the distance2ground policy
  A_distance2ground(0, 0) = a_d2g_;
  distance2ground_policy.setA(A_distance2ground);

  // add the policies into the container
  camera_container.addPolicy(&camera_policy);
  distance_container.addPolicy(&distance_policy);
  distance2ground_container.addPolicy(&distance2ground_policy);

  // create a trapezoidal integrator with the containers
  rmpcpp::TrapezoidalIntegrator<camera_geometry, distance_geometry, distance2ground_geometry> integrator(camera_container, distance_container, distance2ground_container);

  // reset integrator with current pos and vel for x, y, z, yaw
  const int config_space_dimension = camera_geometry::D;
  Eigen::Matrix<double, config_space_dimension, 1> pos, vel, acc;
  pos[0] = planning_uav_state_.pose.translation()[0];
  pos[1] = planning_uav_state_.pose.translation()[1];
  pos[2] = planning_uav_state_.pose.translation()[2];
  pos[3] = planning_uav_state_.yaw;
  vel[0] = planning_uav_state_.velocity[0];
  vel[1] = planning_uav_state_.velocity[1];
  vel[2] = planning_uav_state_.velocity[2];
  vel[3] = planning_uav_state_.yaw_vel;
  acc[0] = planning_uav_state_.acceleration[0];
  acc[1] = planning_uav_state_.acceleration[1];
  acc[2] = planning_uav_state_.acceleration[2];
  acc[3] = planning_uav_state_.yaw_acc;
  integrator.resetTo(pos, vel, acc);

  // set K matrix elements for jacobian calculation
  Eigen::Matrix<double, 4, 1> elems;
  elems[0] = pinhole_constants_.f_x;
  elems[1] = pinhole_constants_.f_y;
  elems[2] = pinhole_constants_.u_0;
  elems[3] = pinhole_constants_.v_0;
  task_space_camera_geometry.SetK(elems);

  // set target position in world frame for jacobian calculation of both geometries
  task_space_camera_geometry.SetTargetPos(cur_target_pos_);
  task_space_distance_geometry.SetTargetPos(cur_target_pos_);

  // set current position in Q for jacobian calculation of both geometries
  task_space_camera_geometry.setQ(pos);
  task_space_distance_geometry.setQ(pos);

  // create msg and define sampling interval & MPC_horizon
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  Eigen::Matrix<double, config_space_dimension, 1> traj_pos, traj_vel, traj_acc;
  Eigen::Matrix<double, task_space_camera_dimension, 1> u_v, u_v_dot;
  Eigen::Matrix<double, task_space_distance_dimension, 1> d, d_dot;
  Eigen::Matrix<double, task_space_distance2ground_dimension, 1> d2g, d2g_dot;

  // integrate 2s into the future
  for(double t = 0.0; t <= MPC_horizon_ + sampling_interval_; t += sampling_interval_){
    // update transformer with roll/pitch 0 and the current position
    if(t > 0.0){
      // get yaw from acceleration and set roll/pitch zero
      double roll, pitch, yaw;
      yaw = traj_pos[3];
      roll = 0;
      pitch = 0;

      // calculate current pose and set transformation matrices new
      Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

      planning_uav_state_.pose.linear() = (rollAngle * yawAngle * pitchAngle).toRotationMatrix();
      planning_uav_state_.pose.translation()[0] = traj_pos[0];
      planning_uav_state_.pose.translation()[1] = traj_pos[1];
      planning_uav_state_.pose.translation()[2] = traj_pos[2];

      transformer_.setMatrices(planning_uav_state_.pose);

      // update current velocity
      planning_uav_state_.velocity[0] = traj_vel[0];
      planning_uav_state_.velocity[1] = traj_vel[1];
      planning_uav_state_.velocity[2] = traj_vel[2];
    }

    // calculate distance 2 ground
    d2g[0] = planning_uav_state_.pose.translation()[2] - 0;
    d2g_dot[0] = planning_uav_state_.velocity[2];

    // get u, v, d and normalization constant for velocity from transformer which was updated above with the pose currently looked at
    std::pair<Eigen::Matrix<double, 3, 1>, double> image_pair =
             transformer_.PosWorld2Image(cur_target_pos_);
    u_v[0] = image_pair.first[0];
    u_v[1] = image_pair.first[1];
    d[0] = image_pair.first[2];
    double vel_normalization = image_pair.second;

    // get the velocities
    Eigen::Matrix<double, 3, 1> image_vel =
             transformer_.VelWorld2Image(cur_target_pos_, planning_uav_state_.velocity, vel_normalization);
    u_v_dot[0] = image_vel[0];
    u_v_dot[1] = image_vel[1];
    d_dot[0] = image_vel[2];

    // calculate max acc in (u, v) depending on a_max_W, use the function for calc vel because it's calculated the same way
    Eigen::Vector3d cur_max_acc;
    cur_max_acc << a_max_W_, 0.0, 0.0;
    Eigen::Matrix<double, 3, 1> image_max_acc =
             transformer_.VelWorld2Image(cur_target_pos_, cur_max_acc, vel_normalization);
    double a_max_C = sqrt(image_max_acc[0] * image_max_acc[0] + image_max_acc[1] * image_max_acc[1]);
    camera_policy.setMaxAcc(a_max_C);

    // fill the metric for the camera policy
    double divider_u = pinhole_constants_.f_x * pinhole_constants_.f_x / (planning_uav_state_.pose.translation()[2] - cur_target_pos_[2]) / (planning_uav_state_.pose.translation()[2] - cur_target_pos_[2]);
    double divider_v = pinhole_constants_.f_y * pinhole_constants_.f_y / (planning_uav_state_.pose.translation()[2] - cur_target_pos_[2]) / (planning_uav_state_.pose.translation()[2] - cur_target_pos_[2]);
    A_camera(0, 0) = a_u_ / divider_u;
    A_camera(1, 1) = a_v_ / divider_v;
    camera_policy.setA(A_camera);

    // set maximum acceleration in distance and distance2ground policy
    distance_policy.setMaxAcc(a_max_W_);
    distance2ground_policy.setMaxAcc(a_max_W_);

    // set whether or not the target is passed
    distance_policy.setTargetPassed(target_passed_);

    // calculate acceleration
    integrator.setX(u_v, u_v_dot, d, d_dot, d2g, d2g_dot);
    integrator.setPercentages(a_u_, a_v_, a_d_, a_d2g_);
    integrator.forwardIntegrate(sampling_interval_);
    integrator.getState(&traj_pos, &traj_vel, &traj_acc);

    // create trajectory point from current state and append it to the trajectory msg
    this->create_traj_point(t, traj_pos, traj_vel, traj_acc, &trajectory_msg);

    // if the target is passed change the distance policy
    Eigen::Vector3d uav_pos;
    uav_pos << traj_pos[0], traj_pos[1], traj_pos[2];
    if((cur_target_pos_ - uav_pos).norm() < 0.1 && !target_passed_){
      target_passed_ = true;
    }

    // publish acc field and state for analyzation purposes
    double t_for_msg = time_of_last_detection_.toSec() + t;

    this->publish_analyzation_msg(camera_policy.getAccField(), u_v, u_v_dot,
                                  distance_policy.getAccField(), d, d_dot, t_for_msg);
  }

  // publish trajectory
  trajectory_msg.header.stamp = ros::Time::now();
  pub_traj_.publish(trajectory_msg);

  chrono_t2_ = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(chrono_t2_ - chrono_t1_).count();
  // std::cout << "Duration: " << duration << " microseconds" << std::endl;
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

    if(this->SubDetection()){
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
    position_now = physical_uav_state_.pose.translation();

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
