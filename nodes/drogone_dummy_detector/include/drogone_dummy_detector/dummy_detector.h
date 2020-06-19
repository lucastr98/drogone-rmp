#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>
#include <drogone_msgs_rmp/CameraUV.h>
#include <drogone_transformation_lib/world_to_camera.h>

namespace drogone_dummy_detector {

struct UAVState {
  ros::Time timestamp;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Quaterniond orientation;
};

class DummyDetector{
  public:
    DummyDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void victim_callback(const trajectory_msgs::MultiDOFJointTrajectory& victim_traj);
    void publish_u_v(Eigen::Matrix<double, 2, 1> u_v, double stamp);

  private:
    // Publishers & Subscribers
    ros::Publisher pub_u_v_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_victim_traj_;

    // NodeHandles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // flight state of the drone
    UAVState uav_state_;
    double old_stamp_;

    // camera constraints
    drogone_transformation_lib::PinholeConstants pinhole_constants_;
    drogone_transformation_lib::CameraMounting camera_mounting_;
};

} // namespace drogone_motion_planning
