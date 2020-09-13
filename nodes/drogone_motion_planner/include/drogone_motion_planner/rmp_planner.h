#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <chrono>
#include <mutex>
#include <condition_variable>

#include <drogone_action_rmp/FSMAction.h>
#include <drogone_motion_planner/cart_cam_geom.h>
#include <drogone_motion_planner/distance_geom.h>
#include <drogone_motion_planner/distance2ground_geom.h>
#include <drogone_motion_planner/camera_target_policy.h>
#include <drogone_motion_planner/distance_target_policy.h>
#include <drogone_motion_planner/distance2ground_policy.h>
#include <drogone_motion_planner/trapezoidal_integrator.h>
#include <rmpcpp/core/policy_container.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <drogone_msgs_rmp/target_detection.h>
#include <drogone_transformation_lib/transformations.h>
#include <drogone_msgs_rmp/AccFieldWithState.h>
#include <geometry_msgs/PoseArray.h>

namespace drogone_rmp_planner {

struct UAVState {
  ros::Time timestamp;
  Eigen::Affine3d pose;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  double yaw;
  double yaw_vel;
  double yaw_acc;
};

struct DetectorInfo {
  double u;
  double v;
  double d;
};

class RMPPlanner{
  public:
    RMPPlanner(std::string name, ros::NodeHandle nh, ros::NodeHandle nh_private);

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);
    void uavTrajCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& traj);
    void server_callback(const drogone_action_rmp::FSMGoalConstPtr& goal);

    bool TakeOff();
    void SubDetection();
    void detection_callback(const drogone_msgs_rmp::target_detection& victim_pos);
    void planTrajectory();
    void updateWeights(double u, double v, double d);
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
    ros::Publisher pub_analyzation_policy_;
    ros::Publisher new_temporary_pub_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_traj_;
    ros::Subscriber sub_follow_;

    // NodeHandles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // flight state of the drone
    UAVState physical_uav_state_;
    UAVState trajectory_uav_state_;
    UAVState planning_uav_state_;
    bool first_odom_cb_;
    double old_stamp_;
    ros::Time time_of_last_detection_;

    Eigen::Vector3d cur_target_pos_;
    ros::Time follow_starting_time_;

    std::string mode_;

    // parameters that are changed depending on which state the uav is in
    double u_target_;
    double v_target_;
    double uv_alpha_;
    double uv_beta_;
    double uv_c_;
    double d_target_;
    double d_alpha_;
    double d_beta_;
    double d_c_;
    double d2g_target_;
    double d2g_alpha_;
    double d2g_beta_;
    double d2g_c_;

    // camera constraints
    drogone_transformation_lib::PinholeConstants pinhole_constants_;
    double image_width_px_;
    double image_height_px_;
    drogone_transformation_lib::CameraMounting camera_mounting_;
    drogone_transformation_lib::Transformations transformer_;

    DetectorInfo detection_;

    // metrics
    double a_d_;
    double a_d2g_;
    double a_u_;
    double a_v_;

    bool first_detection_;

    bool target_passed_;

    double accuracy_ = 0.3;
    double a_max_W_;
    double frequency_;
    double MPC_horizon_;
    double sampling_interval_;
    bool stop_sub_;

    double normalize_u_v_;

    // condition variable
    std::condition_variable cond_var_;
    std::mutex mutex_;

    // analyze computational time
    std::chrono::time_point<std::chrono::high_resolution_clock> chrono_t1_;
    std::chrono::time_point<std::chrono::high_resolution_clock> chrono_t2_;
};

} // namespace drogone_motion_planning
