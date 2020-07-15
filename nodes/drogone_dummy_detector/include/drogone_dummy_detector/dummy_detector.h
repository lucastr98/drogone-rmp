#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>
#include <drogone_msgs_rmp/target_detection.h>
#include <drogone_transformation_lib/transformations.h>

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
    void publish_detection(Eigen::Matrix<double, 3, 1> u_v, double stamp);

  private:
    // Publishers & Subscribers
    ros::Publisher pub_detection_;
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
    double w_;
    double h_;

    drogone_transformation_lib::Transformations transformer_;
};

} // namespace drogone_motion_planning
