#include <drogone_rmp/rmpcpp_example.h>

int main(int argc, char **argv){

  std::string node_name = "drogone_rmp_example";

  ros::init(argc, argv, node_name);

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  drogone_rmp_example::RMP_example rmp_planner(nh, nh_private);

  ros::Duration(3.0).sleep();
  ROS_WARN_STREAM("PLANNER IS READY");

  std::cin.get();

  rmp_planner.start();

  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}
