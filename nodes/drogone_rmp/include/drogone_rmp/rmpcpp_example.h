#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include "geometry_msgs/PoseStamped.h"
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <Eigen/QR>

#include <rmpcpp/core/policy_container.h>
#include <rmpcpp/geometry/linear_geometry.h>
#include <rmpcpp/geometry/cylindrical_geometry.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <rmpcpp/util/vector_range.h>
#include <rmpcpp/eval/trapezoidal_integrator.h>

namespace drogone_rmp_example{

struct UAVState {
  ros::Time timestamp;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Quaterniond orientation;
};

class RMP_example{
  public:
    RMP_example(ros::NodeHandle nh, ros::NodeHandle nh_private);
    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);
    void create_traj_point(double t, trajectory_msgs::MultiDOFJointTrajectory *msg);
    void start();

  private:
    ros::Subscriber sub_odom_;
    ros::Publisher command_pose_pub_;
    ros::Publisher command_pub_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    UAVState uav_state_;
    double old_stamp_;
    bool first_odom_cb_;

    UAVState traj_state_;
    bool replanning_;

    Eigen::MatrixXd A_;
    Eigen::MatrixXd f_;
    Eigen::Vector3d take_off_position_;
    Eigen::Vector3d goal_position_;
    double time_from_start_;
    bool position_reached_;

    double alpha_;
    double beta_;
    double c_;

};

} // namespace drogone_rmp
