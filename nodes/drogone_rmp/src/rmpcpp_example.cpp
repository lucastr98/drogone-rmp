#include <drogone_rmp/rmpcpp_example.h>

namespace drogone_rmp_example{

RMP_example::RMP_example(ros::NodeHandle nh, ros::NodeHandle nh_private):
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
    if (!nh_private_.getParam("goal_position", goal_position)) {
      ROS_ERROR("failed to load goal_position.");
    }
    goal_position_[0] = goal_position[0];
    goal_position_[1] = goal_position[1];
    goal_position_[2] = goal_position[2];

    // load replanning bool
    if (!nh_private_.getParam("replanning", replanning_)) {
      ROS_ERROR("failed to load replanning.");
    }

    // alpha, beta and c
    if (!nh_private_.getParam("alpha", alpha_)) {
      ROS_ERROR("failed to load alpha.");
    }
    if (!nh_private_.getParam("beta", beta_)) {
      ROS_ERROR("failed to load beta.");
    }
    if (!nh_private_.getParam("c", c_)) {
      ROS_ERROR("failed to load c.");
    }

    // subscriber for Odometry from drone
    sub_odom_ =
        nh_.subscribe("uav_pose", 1, &RMP_example::uavOdomCallback, this);

    // publisher for trajectory to drone
    command_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        mav_msgs::default_topics::COMMAND_POSE, 1);

    // publisher for trajectory to drone
    command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
}

// Callback to get current Pose of UAV
void RMP_example::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
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

void RMP_example::start(){
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
  ros::Duration(2.0).sleep();

  using geom = rmpcpp::CylindricalGeometry;
  // using geom = rmpcpp::LinearGeometry<3>;

  // create a variable for the task space geometry in cylindrical coordinates
  geom task_space_geometry;

  // create a container for all policies in the geometry of the task space
  rmpcpp::PolicyContainer<geom> container(task_space_geometry);

  // create simple target policy in task space with a target defined with theta, rho, z
  const int task_space_dimension = geom::K;
  Eigen::Matrix<double, task_space_dimension, 1> target;
  target = goal_position_;
  rmpcpp::SimpleTargetPolicy<task_space_dimension> target_policy(target, alpha_, beta_, c_);

  // create a metric a for the target policy
  geom::Matrix_x A_target(geom::Matrix_x::Zero());

  // fill in the metric A and set it in the policy
  A_target(0, 0) = 1.0;
  A_target(1, 1) = 1.0;
  A_target(2, 2) = 1.0;
  target_policy.setA(A_target);

  // add the policy into the container
  container.addPolicy(&target_policy);

  /* up until now the task policy was defined in the cylindrical task space */

  // create a trapezoidal integrator with the container
  rmpcpp::TrapezoidalIntegrator<geom> integrator(container);

  if(replanning_){
    // creating a publishing rate
    ros::Rate r(10);

    // create msg and define sampling interval & MPC_horizon
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    double dt = 0.01;
    double MPC_horizon = 2.0;

    // reset the integrator with the current position
    ros::spinOnce();
    integrator.resetTo(uav_state_.position, uav_state_.velocity);

    // calculate the distance to the goal
    double distance = (goal_position_ - uav_state_.position).norm();

    while(distance > 0.05){
      integrator.resetTo(uav_state_.position, uav_state_.velocity);

      // integrate over MPC_horizon and store pos, vel & acc in msg
      for(double t = 0.0; t <= MPC_horizon + dt; t += dt){
        integrator.forwardIntegrate(dt);
        integrator.getState(&traj_state_.position, &traj_state_.velocity, &traj_state_.acceleration);
        this->create_traj_point(t, &trajectory_msg);
      }

      // publish trajectory
      trajectory_msg.header.stamp = ros::Time::now();
      command_pub_.publish(trajectory_msg);
      ROS_WARN_STREAM("PUBLISHED");
      trajectory_msg.points.clear();

      // sleep until it's time to publish new traj
      r.sleep();

      // update distance
      ros::spinOnce();
      distance = (goal_position_ - uav_state_.position).norm();
    }
  }
  else{
    // reset the integrator with the current position
    ros::spinOnce();
    integrator.resetTo(uav_state_.position, uav_state_.velocity);

    // create msg and define sampling interval & MPC_horizon
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    double dt = 0.01;
    double t = 0;

    // integrate until drone is at rest and store pos, vel & acc in msg
    while(!integrator.isDone()){
      integrator.forwardIntegrate(dt);
      integrator.getState(&traj_state_.position, &traj_state_.velocity, &traj_state_.acceleration);
      this->create_traj_point(t, &trajectory_msg);
      t += dt;
    }

    // publish trajectory
    trajectory_msg.header.stamp = ros::Time::now();
    command_pub_.publish(trajectory_msg);
  }


}

void RMP_example::create_traj_point(double t, trajectory_msgs::MultiDOFJointTrajectory *msg){
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
  geometry_msgs::Vector3 pos_msg, vel_msg, acc_msg;
  tf::vectorEigenToMsg(traj_state_.position, pos_msg);
  tf::vectorEigenToMsg(traj_state_.velocity, vel_msg);
  tf::vectorEigenToMsg(traj_state_.acceleration, acc_msg);
  geometry_msgs::Transform transform_pos_msg;
  geometry_msgs::Twist twist_vel_msg, twist_acc_msg;
  transform_pos_msg.translation = pos_msg;
  twist_vel_msg.linear = vel_msg;
  twist_acc_msg.linear = acc_msg;
  trajectory_point_msg.transforms.push_back(transform_pos_msg);
  trajectory_point_msg.velocities.push_back(twist_vel_msg);
  trajectory_point_msg.accelerations.push_back(twist_acc_msg);
  trajectory_point_msg.time_from_start = ros::Duration(t);
  msg->points.push_back(trajectory_point_msg);
}


}
