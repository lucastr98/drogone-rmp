#include <drogone_motion_planner/rmp_planner.h>

int main(int argc, char **argv){

  std::string node_name = "rmp_planner_node";

  ros::init(argc, argv, node_name);

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  drogone_rmp_planner::RMPPlanner planner("Motionplanner", nh, nh_private);

  ros::Duration(3.0).sleep();
  ROS_WARN_STREAM("MOTIONPLANNER IS READY");

  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}
