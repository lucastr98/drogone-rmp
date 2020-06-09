//dummy node for the back_to_position_hold service, if we are using the gayebo simulation instead of testing on voliro
// Autor: Felix Stadler, Date: 11.11.19
#include <ros/ros.h>
#include <std_srvs/Empty.h>

//define dummy service_callback function
bool service_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ROS_WARN_STREAM("BACK_TO_POSITION_HOLD: service was called");
  return 1;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "back_to_position_hold_server");
  ros::NodeHandle nh;

  ROS_WARN_STREAM("BACK_TO_POSITION_HOLD: SERVER IS RUNNING");

  //create Service server
  ros::ServiceServer back_to_position_hold_server = nh.advertiseService("drogone/back_to_position_hold", &service_callback);

  ros::spin();

  return 0;
}
