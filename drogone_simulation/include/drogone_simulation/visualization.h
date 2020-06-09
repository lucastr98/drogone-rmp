#include <ros/ros.h>
#include <time.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_msgs/conversions.h>
#include <mav_planning_msgs/PolynomialSegment4D.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <mav_visualization/helpers.h>
#include <drogone_msgs/DijkstraViz.h>



using namespace std;

class Visualize
{
  protected:

    ros::NodeHandle nh_; //startup and shutdown of internal node inside cpp

    ros::Subscriber victim_odom_; //in simulation
    ros::Subscriber tracker_; // position of victim drone from tracker
    ros::Subscriber ground_truth_tracker_; // position of victim drone from ground truth tracker
    ros::Subscriber drogone_odom_;
    ros::Subscriber sub_GUI_;
    ros::Subscriber trajectory_sub_;
    ros::Subscriber sent_trajectory_sub_;
    ros::Subscriber dijkstra_sub_;

    ros::Publisher pub_drogone_;
    ros::Publisher pub_victim_path_;
    ros::Publisher pub_victim_pos_;
    ros::Publisher pub_intersection_point_;
    ros::Publisher pub_tracker_points_;
    ros::Publisher pub_tracker_path_;
    ros::Publisher pub_ground_truth_tracker_points_;
    ros::Publisher pub_ground_truth_tracker_path_;
    ros::Publisher pub_Arena_;
    ros::Publisher pub_markers_;


  public:

    nav_msgs::Odometry odometry_msg_;
    geometry_msgs::PoseArray tracker_poses_ ;
    nav_msgs::Path path_msg_drogone_;
    nav_msgs::Path path_arena_;
    nav_msgs::Path path_msg_victim_;

    bool simulation_ = 0; //simulation or outdoor testing

    Visualize(ros::NodeHandle& nh);

    void victim_odom_callback(const nav_msgs::Odometry& points); //in simulation

    void tracker_callback(const trajectory_msgs::MultiDOFJointTrajectory& traj);
    void ground_truth_tracker_callback(const trajectory_msgs::MultiDOFJointTrajectory& traj);

    void drogone_odom_callback(const nav_msgs::Odometry& msg);

    void publishPath_drogone( nav_msgs::Path& path_msg_,
                              nav_msgs::Odometry& odometry_msg_,
                              const ros::Time& stamp);

    void publishPath_Victim( nav_msgs::Path& path_msg_,
                             nav_msgs::Odometry& odometry_msg_,
                             const ros::Time& stamp);

    void publishArena_Frame(nav_msgs::Path& path_msg);

    void GUI_callback(const std_msgs::Float32MultiArray& params);

    void dijkstra_callback(const drogone_msgs::DijkstraViz& msg);

    void trajectory_callback(const mav_planning_msgs::PolynomialTrajectory4D& traj_msg);

    void sent_trajectory_callback(const trajectory_msgs::MultiDOFJointTrajectory& traj_msg);

};
