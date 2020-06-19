#include <drogone_dummy_detector/dummy_detector.h>

int main(int argc, char **argv){

  std::string node_name = "dummy_detector_node";

  ros::init(argc, argv, node_name);

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  drogone_dummy_detector::DummyDetector dummy_detector(nh, nh_private);

  ros::Duration(3.0).sleep();
  ROS_WARN_STREAM("DUMMY DETECTOR IS READY");

  while (ros::ok()) {
    ros::spin();
  }
  
  return 0;
}
