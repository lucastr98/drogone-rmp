#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include "geometry_msgs/PoseStamped.h"
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <Eigen/QR>

namespace drogone_rmp{

struct UAVState {
  ros::Time timestamp;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Quaterniond orientation;
};

class RMP{
  public:
    RMP(ros::NodeHandle nh, ros::NodeHandle nh_private);
    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);
    void start();
    void get_new_traj_point(trajectory_msgs::MultiDOFJointTrajectory *trajectory_msg);
    double s(double v);

  private:
    ros::Subscriber sub_odom_;
    ros::Publisher command_pose_pub_;
    ros::Publisher command_pub_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    UAVState uav_state_;
    double old_stamp_;
    bool first_odom_cb_;

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
