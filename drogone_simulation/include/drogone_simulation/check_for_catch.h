// Author: Felix Stadler, Date 30.3.20
#include <ros/ros.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include <drogone_action/FSMAction.h> //includes the action mesage generated from FSM.action
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>


using namespace std;

class CheckServer
{
protected:

  ros::NodeHandle nh_; //startup and shutdown of internal node inside cpp
  actionlib::SimpleActionServer<drogone_action::FSMAction> as_;
  string action_name_; //variable which has name of actionserver

  drogone_action::FSMResult result_;

  //define Subscriber
  ros::Subscriber victim_odom_;
  ros::Subscriber drogone_odom_;

  ros::Publisher offset_;

  //position of the two drones
  geometry_msgs::Point victim_position_;
  geometry_msgs::Point drogone_position_;

  bool caught_; //zero = not caught, one = caught, default is zero
  std::vector<double> distance_; //distance between victim drone and Drogone drone such that victim drone is caught, can be changed in yaml file


public:

  //Action Constructor (creater of action server)
  CheckServer(string name, ros::NodeHandle& nh);

  //Subscriber callback functions
  void victim_odom_callback(const nav_msgs::Odometry& odom);
  void drogone_odom_callback(const nav_msgs::Odometry& odom);

  //define execute function, goal contains mode
  void server_callback(const drogone_action::FSMGoalConstPtr& goal);

  //check for catch function
  void check_for_catch(const geometry_msgs::Point& victim_pos, const geometry_msgs::Point& drogone_pos);

};
