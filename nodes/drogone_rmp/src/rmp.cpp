#include <drogone_rmp/rmp.h>

namespace drogone_rmp{

RMP::RMP(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  first_odom_cb_(true){
    // load take off position
    std::vector<double> take_off_position;
    if (!nh_.getParam("rmp/take_off_position", take_off_position)) {
      ROS_ERROR("failed to load take_off_position.");
    }
    take_off_position_[0] = take_off_position[0];
    take_off_position_[1] = take_off_position[1];
    take_off_position_[2] = take_off_position[2];

    // load goal position
    std::vector<double> goal_position;
    if (!nh_.getParam("rmp/goal_position", goal_position)) {
      ROS_ERROR("failed to load goal_position.");
    }
    goal_position_[0] = goal_position[0];
    goal_position_[1] = goal_position[1];
    goal_position_[2] = goal_position[2];

    if (!nh_.getParam("rmp/alpha", alpha_)) {
      ROS_ERROR("failed to load alpha.");
    }
    if (!nh_.getParam("rmp/beta", beta_)) {
      ROS_ERROR("failed to load beta.");
    }
    if (!nh_.getParam("rmp/c", c_)) {
      ROS_ERROR("failed to load c.");
    }

    // load A matrix
    XmlRpc::XmlRpcValue A;
    A_ = Eigen::MatrixXd::Zero(3, 3);
    if (!nh_.getParam("rmp/A", A)) {
      ROS_ERROR("failed to load A.");
    }
    ROS_ASSERT(A.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < 3; ++i){
      for(int j = 0; j < 3; ++j){
        A_(i, j) = A[i * 3 + j];
      }
    }

    // subscriber for Odometry from drone
    sub_odom_ =
        nh_.subscribe("uav_pose", 1, &RMP::uavOdomCallback, this);

    // publisher for trajectory to drone
    command_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        mav_msgs::default_topics::COMMAND_POSE, 1);

    // publisher for trajectory to drone
    command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
}

// Callback to get current Pose of UAV
void RMP::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
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

void RMP::start(){
  geometry_msgs::PoseStamped msg;
  msg.pose.position.x = take_off_position_[0];
  msg.pose.position.y = take_off_position_[1];
  msg.pose.position.z = take_off_position_[2];
  msg.header.stamp = ros::Time::now();

  command_pose_pub_.publish(msg);
  Eigen::Vector3d current_pos = uav_state_.position;
  while((take_off_position_ - uav_state_.position).norm() > 0.1){
    ros::Duration(0.01).sleep();
    ros::spinOnce();
    current_pos = uav_state_.position;
  }
  ros::Duration(2).sleep();

  // define f and A matrices
  f_ = Eigen::MatrixXd::Zero(3, 6);
  for(int i = 0; i < 3; ++i){
    for(int j = 0; j < 6; ++j){
      if(i == j){
        f_(i, j) = alpha_;
      }
      else if(j - 3 == i){
        f_(i, j) = -beta_;
      }
    }
  }

  ros::spinOnce();
  Eigen::VectorXd base(6);
  base[0] = s(goal_position_[0] - uav_state_.position[0]);
  base[1] = s(goal_position_[1] - uav_state_.position[1]);
  base[2] = s(goal_position_[2] - uav_state_.position[2]);
  base[3] = uav_state_.velocity[0];
  base[4] = uav_state_.velocity[1];
  base[5] = uav_state_.velocity[2];

  // now the vector field is generated

  Eigen::Vector3d acceleration, acc_metric_consid, vel_metric_consid, pos_metric_consid;
  acceleration = f_ * base;
  acc_metric_consid = A_ * acceleration;
  vel_metric_consid = A_ * uav_state_.velocity;
  pos_metric_consid = A_ * uav_state_.position;

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
  geometry_msgs::Vector3 pos_msg, vel_msg, acc_msg;
  tf::vectorEigenToMsg(pos_metric_consid, pos_msg);
  tf::vectorEigenToMsg(vel_metric_consid, vel_msg);
  tf::vectorEigenToMsg(acc_metric_consid, acc_msg);
  geometry_msgs::Transform transform_pos_msg;
  geometry_msgs::Twist twist_vel_msg, twist_acc_msg;
  transform_pos_msg.translation = pos_msg;
  twist_vel_msg.linear = vel_msg;
  twist_acc_msg.linear = acc_msg;
  trajectory_point_msg.transforms.push_back(transform_pos_msg);
  trajectory_point_msg.velocities.push_back(twist_vel_msg);
  trajectory_point_msg.accelerations.push_back(twist_acc_msg);
  time_from_start_ = 0.0;
  trajectory_point_msg.time_from_start = ros::Duration(time_from_start_);
  trajectory_msg.points.push_back(trajectory_point_msg);

  position_reached_ = false;
  while(!position_reached_){
    get_new_traj_point(&trajectory_msg);
  }

  trajectory_msg.header.stamp = ros::Time::now();
  command_pub_.publish(trajectory_msg);

  get_new_traj_point(&trajectory_msg);
}

// a = a0
// v = a0 * t + v0
// r = a0 / 2 * t * t + v0 * t + r0
void RMP::get_new_traj_point(trajectory_msgs::MultiDOFJointTrajectory *trajectory_msg){
  double dt = 0.01;
  time_from_start_ += dt;
  Eigen::Vector3d a0, v0, r0, a1, v1, r1;
  tf::vectorMsgToEigen(trajectory_msg->points.back().accelerations[0].linear, a0);
  tf::vectorMsgToEigen(trajectory_msg->points.back().velocities[0].linear, v0);
  tf::vectorMsgToEigen(trajectory_msg->points.back().transforms[0].translation, r0);

  if((A_ * goal_position_ - r0).norm() < 0.01){
    position_reached_ = true;
  }

  if(A_.determinant() == 0){
    a0 = A_.completeOrthogonalDecomposition().pseudoInverse() * a0;
    v0 = A_.completeOrthogonalDecomposition().pseudoInverse() * v0;
    r0 = A_.completeOrthogonalDecomposition().pseudoInverse() * r0;
  }
  else{
    a0 = A_.inverse() * a0;
    v0 = A_.inverse() * v0;
    r0 = A_.inverse() * r0;
  }

  v1 = a0 * dt + v0;
  r1 = a0 / 2 * dt * dt + v0 * dt + r0;

  Eigen::VectorXd base(6);
  base[0] = s(goal_position_[0] - r1[0]);
  base[1] = s(goal_position_[1] - r1[1]);
  base[2] = s(goal_position_[2] - r1[2]);
  base[3] = v1[0];
  base[4] = v1[1];
  base[5] = v1[2];

  a1 = f_ * base;
  a1 = A_ * a1;
  v1 = A_ * v1;
  r1 = A_ * r1;

  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
  geometry_msgs::Vector3 pos_msg, vel_msg, acc_msg;
  tf::vectorEigenToMsg(r1, pos_msg);
  tf::vectorEigenToMsg(v1, vel_msg);
  tf::vectorEigenToMsg(a1, acc_msg);
  geometry_msgs::Transform transform_pos_msg;
  geometry_msgs::Twist twist_vel_msg, twist_acc_msg;
  transform_pos_msg.translation = pos_msg;
  twist_vel_msg.linear = vel_msg;
  twist_acc_msg.linear = acc_msg;
  trajectory_point_msg.transforms.push_back(transform_pos_msg);
  trajectory_point_msg.velocities.push_back(twist_vel_msg);
  trajectory_point_msg.accelerations.push_back(twist_acc_msg);
  trajectory_point_msg.time_from_start = ros::Duration(time_from_start_);
  trajectory_msg->points.push_back(trajectory_point_msg);
}

double RMP::s(double v){
  double h = v + c_ * std::log(1 + std::exp(-2 * c_ * v));
  return v / h;
}

}
