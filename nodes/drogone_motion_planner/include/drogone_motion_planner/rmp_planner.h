#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include <drogone_action_rmp/FSMAction.h>
#include <drogone_motion_planner/cart_cam_geom.h>
#include <drogone_motion_planner/distance_geom.h>
#include <drogone_motion_planner/simple_camera_target_policy.h>
#include <drogone_motion_planner/simple_distance_target_policy.h>
#include <drogone_motion_planner/trapezoidal_integrator.h>
#include <rmpcpp/core/policy_container.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <drogone_msgs_rmp/target_detection.h>
#include <drogone_transformation_lib/transformations.h>
#include <drogone_msgs_rmp/AccFieldWithState.h>

namespace drogone_rmp_planner {

struct UAVState {
  ros::Time timestamp;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Quaterniond orientation;
  double yaw;
  double yaw_vel;
};

class RMPPlanner{
  public:
    RMPPlanner(std::string name, ros::NodeHandle nh, ros::NodeHandle nh_private);

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);
    void server_callback(const drogone_action_rmp::FSMGoalConstPtr& goal);

    bool TakeOff();
    bool Follow();
    void follow_callback(const drogone_msgs_rmp::target_detection& victim_pos);
    bool Land();

    bool accuracy_reached(const Eigen::Vector3d& goal_pos, double waiting_time);
    void create_traj_point(double t, Eigen::Matrix<double, 4, 1> traj_pos,
                           Eigen::Matrix<double, 4, 1> traj_vel,
                           Eigen::Matrix<double, 4, 1> traj_acc,
                           trajectory_msgs::MultiDOFJointTrajectory *msg);
    void publish_analyzation_msg(Eigen::Matrix<double, 2, 1> f_u_v,
                                 Eigen::Matrix<double, 2, 1> u_v,
                                 Eigen::Matrix<double, 2, 1> u_v_dot,
                                 Eigen::Matrix<double, 1, 1> f_d,
                                 Eigen::Matrix<double, 1, 1> d,
                                 Eigen::Matrix<double, 1, 1> d_dot,
                                 double t);

  protected:
    actionlib::SimpleActionServer<drogone_action_rmp::FSMAction> as_;
    std::string action_name_; //variable which has name of actionserve

  private:
    // Publishers & Subscribers
    ros::Publisher pub_traj_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_analyzation_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_follow_;

    // NodeHandles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // flight state of the drone
    UAVState uav_state_;
    bool first_odom_cb_;
    double old_stamp_;

    // camera constraints
    drogone_transformation_lib::PinholeConstants pinhole_constants_;
    drogone_transformation_lib::CameraMounting camera_mounting_;

    int follow_counter_;

    double accuracy_ = 0.3;
    bool stop_sub_;

    double normalize_u_v_;
};

} // namespace drogone_motion_planning
