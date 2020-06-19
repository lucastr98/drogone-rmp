#include <drogone_dummy_detector/dummy_detector.h>

namespace drogone_dummy_detector {

DummyDetector::DummyDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  old_stamp_(0.0) {

    // publisher for (u, v)
    pub_u_v_ = nh_.advertise<drogone_msgs_rmp::CameraUV>("u_v", 0);

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

    std::vector<double> cam_trans;
    if (!nh_private_.getParam("translation", cam_trans)) {
      ROS_ERROR("failed to load translation.");
    }
    camera_mounting_.translation << cam_trans[0], cam_trans[1], cam_trans[2];

    uav_state_.velocity << 0.0, 0.0, 0.0;

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

  // calculate (u, v)
  drogone_transformation_lib::WorldToCamera transformer(pinhole_constants_, camera_mounting_);
  Eigen::Affine3d uav_pose;
  uav_pose.translation() = uav_state_.position;
  uav_pose.linear() = uav_state_.orientation.toRotationMatrix();
  Eigen::Matrix<double, 2, 1> u_v = transformer.get_u_v(target_pos_W, uav_pose).first;

  // publish (u, v)
  this->publish_u_v(u_v, victim_traj.header.stamp.sec + victim_traj.header.stamp.nsec/1e9);
}

void DummyDetector::publish_u_v(Eigen::Matrix<double, 2, 1> u_v, double stamp){
  drogone_msgs_rmp::CameraUV msg;
  msg.header.stamp = ros::Time(stamp);
  msg.u = u_v[0];
  msg.v = u_v[1];
  pub_u_v_.publish(msg);
}

} // namespace drogone_dummy_detector
