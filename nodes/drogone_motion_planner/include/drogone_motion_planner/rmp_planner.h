#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include <drogone_action_rmp/FSMAction.h>
#include <drogone_motion_planner/cart_cam_geom.h>
#include <rmpcpp/core/policy_container.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <rmpcpp/eval/trapezoidal_integrator.h>
#include <drogone_msgs_rmp/CameraUV.h>

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
    Eigen::Matrix<double, 2, 1> get_u_v(Eigen::Vector3d target, bool use_odom); // Eigen::Quaterniond q);
    void server_callback(const drogone_action_rmp::FSMGoalConstPtr& goal);

    bool TakeOff();
    bool Follow();
    void follow_callback(const trajectory_msgs::MultiDOFJointTrajectory& victim_traj);
    bool Land();

    bool accuracy_reached(const Eigen::Vector3d& goal_pos, double waiting_time);
    void create_traj_point(double t, Eigen::Matrix<double, 4, 1> traj_pos, Eigen::Matrix<double, 4, 1> traj_vel,
                                     Eigen::Matrix<double, 4, 1> traj_acc, trajectory_msgs::MultiDOFJointTrajectory *msg);
    void calc_future_state(Eigen::Matrix<double, 4, 1> pos, Eigen::Matrix<double, 4, 1> acc);

  protected:
    actionlib::SimpleActionServer<drogone_action_rmp::FSMAction> as_;
    std::string action_name_; //variable which has name of actionserve

  private:
    // Publishers & Subscribers
    ros::Publisher trajectory_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher u_v_pub_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_follow_;

    // NodeHandles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // flight state of the drone
    UAVState uav_state_;
    bool first_odom_cb_;
    double old_stamp_;

    // camera matrix elements
    double f_x_;
    double f_y_;
    double u_0_;
    double v_0_;

    // roll pitch yaw
    double future_roll_;
    double future_pitch_;
    double future_yaw_;
    Eigen::Vector3d future_pos_;

    double accuracy_ = 0.3;
    bool stop_sub_;
};

} // namespace drogone_motion_planning
