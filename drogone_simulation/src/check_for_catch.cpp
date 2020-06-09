// Author: Felix Stadler, Date 30.3.20
// node checks if Drogone drone caught the victim drone in the simulation by checking if both positions are close enough
#include <drogone_simulation/check_for_catch.h>


//Action Constructor (creater of action server)
CheckServer::CheckServer(string name, ros::NodeHandle& nh):
  nh_(nh),
  as_(nh_, name, boost::bind(&CheckServer::server_callback, this, _1 ), false), //creates actionserver as_
  action_name_(name), //name of actionserver
  caught_(0)
{
  //get parameters from yaml file
  if (!nh_.getParam("check_for_catch/distances", distance_)) {
      ROS_ERROR("CHECK FOR CATCH: failed to load yaml parameters");
  }
  as_.start(); //start action
}


//define execute/service function, goal contains mode
void CheckServer::server_callback(const drogone_action::FSMGoalConstPtr& goal){

  if (goal->new_mode == "CheckforCatch"){

    victim_odom_ = nh_.subscribe("/victim_drone/odometry", 1, &CheckServer::victim_odom_callback, this);
    drogone_odom_ = nh_.subscribe("odometry_sensor1/odometry", 1, &CheckServer::drogone_odom_callback, this);
    offset_ = nh_.advertise<geometry_msgs::Point>("/visualization/victim_offset", 1);

    while( !as_.isPreemptRequested() && ros::ok() && !caught_){}

    victim_odom_.shutdown();
    drogone_odom_.shutdown();
    if(as_.isPreemptRequested() || !ros::ok()){
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
    }
    else if (caught_){
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded();
    }
    caught_ = 0;
  }
}


//callback for victim drone odometry
void CheckServer::victim_odom_callback(const nav_msgs::Odometry& odom){
  victim_position_ = odom.pose.pose.position;
}


//callback for drogone odometry
void CheckServer::drogone_odom_callback(const nav_msgs::Odometry& odom){
  drogone_position_ = odom.pose.pose.position;
  CheckServer::check_for_catch(drogone_position_, victim_position_);
}


//check if Drogone Drone is close enough to victim drone and "caught" it
void CheckServer::check_for_catch(const geometry_msgs::Point& victim_pos, const geometry_msgs::Point& drogone_pos){
  float dx, dy, dz, horizontal_distance, vertical_distance;
  dx = drogone_position_.x - victim_position_.x;
  dy = drogone_position_.y - victim_position_.y;
  horizontal_distance = sqrt(dx*dx + dy*dy);
  dz = drogone_position_.z - victim_position_.z;
  //make sure vertical distance value is positive
  if (dz > 0){
    vertical_distance = dz;}
  else{
    vertical_distance = dz * (-1);}

  if (horizontal_distance < distance_[0] && vertical_distance < distance_[1] && !caught_ ){
    ROS_WARN("CHECK FOR CATCH: Horizontal Distance %f", horizontal_distance);
    ROS_WARN("CHECK FOR CATCH: Vertical Distance %f", vertical_distance);
    caught_ = 1;
    geometry_msgs::Point offset;
    offset.x = dx;
    offset.y = dy;
    offset.z = dz;
    offset_.publish(offset);
  }
}



int main(int argc, char **argv){
  ros::init(argc, argv, "check_for_catch");
  ros::NodeHandle nh;
  ros::Rate loopRate(10); //how often the subscriber listens to topic

  ROS_WARN_STREAM("CHECK FOR CATCH: SERVER IS RUNNING");

  CheckServer Check ("Check", nh);

  ros::spin();

  return 0;
}
