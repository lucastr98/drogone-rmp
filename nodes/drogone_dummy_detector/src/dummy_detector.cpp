#include <drogone_dummy_detector/dummy_detector.h>

namespace drogone_dummy_detector {

DummyDetector::DummyDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  old_stamp_(0.0) {

    // publisher for (u, v)
    pub_detection_ = nh_.advertise<drogone_msgs_rmp::target_detection>("target_detection", 0);

    // subscriber for Odometry from drone
    sub_odom_ = nh_.subscribe("uav_pose", 1, &DummyDetector::uavOdomCallback, this);

    // subscriber to world position of victim
    sub_victim_traj_ = nh_.subscribe("victim_trajectory", 10, &DummyDetector::victim_callback, this);

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

    if(!nh_private_.getParam("w", w_)){
      ROS_ERROR("failed to load w");
    }
    if(!nh_private_.getParam("h", h_)){
      ROS_ERROR("failed to load h");
    }

    std::vector<double> cam_trans;
    if (!nh_private_.getParam("translation", cam_trans)) {
      ROS_ERROR("failed to load translation.");
    }
    camera_mounting_.translation << cam_trans[0], cam_trans[1], cam_trans[2];

    if(!nh_private_.getParam("noise", noise_)){
      ROS_ERROR("failed to load noise");
    }
    if(!nh_private_.getParam("d_drone", d_drone_)){
      ROS_ERROR("failed to load d_drone");
    }
    if(!nh_private_.getParam("tol_u", tol_u_)){
      ROS_ERROR("failed to load tol_u");
    }
    if(!nh_private_.getParam("tol_v", tol_v_)){
      ROS_ERROR("failed to load tol_v");
    }
    if(!nh_private_.getParam("tol_d", tol_d_)){
      ROS_ERROR("failed to load tol_d");
    }

    uav_state_.velocity << 0.0, 0.0, 0.0;

    transformer_.setCameraConfig(pinhole_constants_, camera_mounting_, nh);
}

// Callback to get current Pose of UAV
void DummyDetector::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  // calculate the time between the 2 measurements based on stamps
  double duration = odom->header.stamp.sec + odom->header.stamp.nsec/1e9 - old_stamp_;
  uav_state_.timestamp = odom->header.stamp;

  // store pose
  Eigen::Affine3d current_pose;
  tf::poseMsgToEigen(odom->pose.pose, current_pose);

  uav_state_.position = current_pose.translation();
  uav_state_.orientation = current_pose.linear();

  // store current_velocity_ from before, before changing it
  Eigen::Vector3d v0, v1;
  v0 = uav_state_.velocity;

  // store current velocity (after transforming it from msg to an Eigen vector)
  tf::vectorMsgToEigen(odom->twist.twist.linear, uav_state_.velocity);
  uav_state_.velocity = uav_state_.orientation.toRotationMatrix() * uav_state_.velocity;

  double t = duration;
  v1 = uav_state_.velocity;

  uav_state_.acceleration = (v1 - v0) / t;

  // store stamp of current msg
  old_stamp_ = odom->header.stamp.sec + odom->header.stamp.nsec/1e9;
}

void DummyDetector::victim_callback(const trajectory_msgs::MultiDOFJointTrajectory& victim_traj){
  // get target position in world from victim trajectory
  Eigen::Vector3d target_pos_W;
  tf::vectorMsgToEigen(victim_traj.points[0].transforms[0].translation, target_pos_W);

  // check if target position has changed more than 1m since last detection and reset noise param then
  bool reset_noise;
  double distance = (target_pos_W - last_target_pos_).norm();
  if(distance > 1.0){
    reset_noise = true;
  }
  else{
    reset_noise = false;
  }
  last_target_pos_ = target_pos_W;

  // check if point is in fov before transforming
  double fov_x = 2 * atan2(w_, 2 * pinhole_constants_.f_x);
  double fov_y = 2 * atan2(h_, 2 * pinhole_constants_.f_y);
  double angle_x = atan2(target_pos_W[0] - uav_state_.position[0], target_pos_W[2] - uav_state_.position[2]);
  double angle_y = atan2(target_pos_W[1] - uav_state_.position[1], target_pos_W[2] - uav_state_.position[2]);
  if(abs(angle_x) > fov_x / 2 || abs(angle_y) > fov_y / 2){
    ROS_WARN_STREAM("DUMMY DETECTOR ----- TARGET NOT IN FOV");
    drogone_msgs_rmp::target_detection msg;
    msg.detected.data = false;
    pub_detection_.publish(msg);
    return;
  }

  // calculate (u, v)
  Eigen::Affine3d uav_pose;
  uav_pose.translation() = uav_state_.position;
  uav_pose.linear() = uav_state_.orientation.toRotationMatrix();
  transformer_.setMatrices(uav_pose);
  transformer_.setNoiseParams(d_drone_, tol_u_, tol_v_, tol_d_, reset_noise);
  Eigen::Matrix<double, 3, 1> detection = transformer_.PosWorld2Image(target_pos_W, noise_).first;

  // publish (u, v, d)
  this->publish_detection(detection, victim_traj.header.stamp.sec + victim_traj.header.stamp.nsec/1e9);
}

void DummyDetector::publish_detection(Eigen::Matrix<double, 3, 1> detection, double stamp){
  drogone_msgs_rmp::target_detection msg;
  msg.header.stamp = ros::Time(stamp);
  msg.u = detection[0];
  msg.v = detection[1];
  msg.d = detection[2];
  msg.detected.data = true;
  pub_detection_.publish(msg);
}

} // namespace drogone_dummy_detector
