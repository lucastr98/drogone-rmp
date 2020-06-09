#include <drogone_simulation/visualization.h>


Visualize::Visualize(ros::NodeHandle& nh):
  nh_(nh)
{
  //----------------------------------publsiher----------------------------------------
  pub_drogone_ = nh_.advertise<nav_msgs::Path>("visualization/path_drogone", 1000);
  pub_victim_path_ = nh_.advertise<nav_msgs::Path>("visualization/path_victim", 1000);
  pub_victim_pos_ = nh_.advertise<geometry_msgs::PointStamped>("visualization/victim_position", 1000);
  pub_Arena_ = nh_.advertise<nav_msgs::Path>("visualization/arena", 1000);
  pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization/trajectory_markers", 0);
  //VISUALIZATION either by pose arrays (poses of the ground truth tracker message) or a path through these poses
  pub_tracker_path_ = nh_.advertise<nav_msgs::Path>("visualization/tracker_path", 100); //path
  pub_tracker_points_ = nh_.advertise<geometry_msgs::PoseArray>("visualization/tracker_poses", 100); //poses
  pub_ground_truth_tracker_path_ = nh_.advertise<nav_msgs::Path>("visualization/ground_truth_tracker_path", 100); //path
  pub_ground_truth_tracker_points_ = nh_.advertise<geometry_msgs::PoseArray>("visualization/ground_truth_tracker_poses", 100); //poses
  pub_intersection_point_ = nh_.advertise<geometry_msgs::PointStamped>("visualization/intersection_point", 100);

  //----------------------------------subscriber----------------------------------------
  victim_odom_ = nh_.subscribe("victim_drone/odometry", 1000, &Visualize::victim_odom_callback, this); //in simulation
  tracker_ = nh_.subscribe("/peregrine/target_estimator/tracker_traj", 1000, &Visualize::tracker_callback, this);
  ground_truth_tracker_ = nh_.subscribe("drogone/ground_truth_tracker", 1000, &Visualize::ground_truth_tracker_callback, this);
  drogone_odom_ = nh_.subscribe("drogone/odometry_sensor1/odometry",1000, &Visualize::drogone_odom_callback, this);
  sub_GUI_ = nh_.subscribe("GUI/victim_drone_param", 1000, &Visualize::GUI_callback, this);
  trajectory_sub_ = nh_.subscribe("drogone/trajectory", 10, &Visualize::trajectory_callback, this);
  // sent_trajectory_sub_ = nh_.subscribe("/peregrine/command/trajectory", 10, &Visualize::sent_trajectory_callback, this);
  dijkstra_sub_ = nh_.subscribe("/visualization/Dijkstra", 10, &Visualize::dijkstra_callback, this);

  ros::Duration(2.0).sleep();
  ros::Rate r(10); //10Hz

  publishArena_Frame(path_arena_);
}


void Visualize::drogone_odom_callback(const nav_msgs::Odometry& msg ){
  const ros::Time& stamp = msg.header.stamp;
  nav_msgs::Odometry odometry = msg;

  publishPath_drogone(path_msg_drogone_,
    odometry,
    stamp);
}

//publish function
void Visualize::publishPath_drogone( nav_msgs::Path& path_msg,
                                       nav_msgs::Odometry& odometry_msg,
                                       const ros::Time& stamp){

  // Update path msg header
  path_msg.header.frame_id = "world";
  path_msg.header.stamp = stamp;
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.pose.position = odometry_msg.pose.pose.position;

  path_msg.poses.push_back(pose_msg);

  if (path_msg.poses.size() > 1500)
    path_msg.poses.erase(path_msg.poses.begin());

  Visualize::pub_drogone_.publish(path_msg);
}


void Visualize::victim_odom_callback(const nav_msgs::Odometry& points){
  simulation_ = 1; //simulation is running when this topic is being published
  const ros::Time& stamp = points.header.stamp;
  nav_msgs::Odometry odometry = points;

  publishPath_Victim(path_msg_victim_,
                     odometry,
                     stamp);
}

void Visualize::tracker_callback(const trajectory_msgs::MultiDOFJointTrajectory& traj){
  nav_msgs::Odometry odometry; // for outdoor testing
  geometry_msgs::PoseArray tracker_poses;
  nav_msgs::Path tracker_path;

  tracker_path.header = traj.header;
  tracker_poses.header= traj.header;

  for (int i=0; i<traj.points.size(); i++){
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = traj.points[i].transforms[0].translation.x;
    pose_stamped.pose.position.y = traj.points[i].transforms[0].translation.y;
    pose_stamped.pose.position.z = traj.points[i].transforms[0].translation.z;
    if (i == 0){
      odometry.pose.pose = pose_stamped.pose;
    }
    else{
      tracker_path.poses.push_back(pose_stamped);
      tracker_poses.poses.push_back(pose_stamped.pose);
    }
  }
  tracker_poses_ = tracker_poses; //safe tracker poses for intersection point

  pub_tracker_path_.publish(tracker_path);
  pub_tracker_points_.publish(tracker_poses);
  // ROS_WARN_STREAM("tracker pub");

  //when outdoor testing:
  if (simulation_ == 0){
    const ros::Time& stamp = traj.header.stamp;
    publishPath_Victim(path_msg_victim_,
                       odometry,
                       stamp);
  }
}

void Visualize::ground_truth_tracker_callback(const trajectory_msgs::MultiDOFJointTrajectory& traj){
  nav_msgs::Odometry odometry; // for outdoor testing
  geometry_msgs::PoseArray ground_truth_tracker_poses;
  nav_msgs::Path ground_truth_tracker_path;

  ground_truth_tracker_path.header = traj.header;
  ground_truth_tracker_poses.header= traj.header;

  for (int i=0; i<traj.points.size(); i++){
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = traj.points[i].transforms[0].translation.x;
    pose_stamped.pose.position.y = traj.points[i].transforms[0].translation.y;
    pose_stamped.pose.position.z = traj.points[i].transforms[0].translation.z;
    ground_truth_tracker_path.poses.push_back(pose_stamped);
    ground_truth_tracker_poses.poses.push_back(pose_stamped.pose);
  }
  pub_ground_truth_tracker_path_.publish(ground_truth_tracker_path);
  pub_ground_truth_tracker_points_.publish(ground_truth_tracker_poses);
}


//publish function
void Visualize::publishPath_Victim( nav_msgs::Path& path_msg,
                             nav_msgs::Odometry& odometry_msg,
                             const ros::Time& stamp){

  // Update path msg header
  path_msg.header.frame_id = "world";
  path_msg.header.stamp = stamp;

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.pose.position = odometry_msg.pose.pose.position;

  path_msg.poses.push_back(pose_msg);
  Visualize::pub_victim_path_.publish(path_msg);

  if (path_msg.poses.size() > 1000){
    path_msg.poses.erase(path_msg.poses.begin());}

  //also publish current position
  geometry_msgs::PointStamped curr_position;
  curr_position.header = path_msg.header;
  curr_position.point = odometry_msg.pose.pose.position;
  Visualize::pub_victim_pos_.publish(curr_position);
}

void Visualize::dijkstra_callback(const drogone_msgs::DijkstraViz& msg){
  geometry_msgs::PointStamped point;
  point.header.frame_id = tracker_poses_.header.frame_id;
  point.header.stamp = msg.header.stamp;

  int i = msg.selected_keys[1] - 2; //-1 because selected keys array starts with drogone position and another -1  because tracker poses does not contain the first element of the tracker msg (the victim drone position)
  if (tracker_poses_.poses.size() > 0){
    point.point = tracker_poses_.poses[i].position;
    pub_intersection_point_.publish(point);
  }
  else{
    ROS_WARN_STREAM("VISUALIZATION: Can't plot Intersection point as Tracker MultiDOFJointTrajectory not published");}
}


//publish arena frame
void Visualize::publishArena_Frame(nav_msgs::Path& path_msg){

  path_msg.header.frame_id = "world";

  geometry_msgs::PoseStamped pose_msg;

  pose_msg.pose.position.z = 0;
  pose_msg.pose.position.x = 35;
  pose_msg.pose.position.y = 20;
  path_msg.poses.push_back(pose_msg);
  int z1 = 0;
  int z2 = 40;
  for (int i= 0; i<2; i++){
    pose_msg.pose.position.z = z2;
    path_msg.poses.push_back(pose_msg);
    pose_msg.pose.position.x = -35;
    path_msg.poses.push_back(pose_msg);
    pose_msg.pose.position.z = z1;
    path_msg.poses.push_back(pose_msg);
    pose_msg.pose.position.y = -20;
    path_msg.poses.push_back(pose_msg);
    pose_msg.pose.position.z = z2;
    path_msg.poses.push_back(pose_msg);
    pose_msg.pose.position.x = 35;
    path_msg.poses.push_back(pose_msg);
    pose_msg.pose.position.z = z1;
    path_msg.poses.push_back(pose_msg);
    pose_msg.pose.position.y = 20;
    path_msg.poses.push_back(pose_msg);
    z1 = 40;
    z2 = 0;
  }

  pub_Arena_.publish(path_msg);
}

//erase all entries path_msg_victim if victim drone is new initialized due to a restart or new starting position
void Visualize::GUI_callback(const std_msgs::Float32MultiArray& params){
  if ((params.data[10] == 1.0) || (params.data[2] == 1.0)){
    path_msg_victim_.poses.clear();
  }
}


Eigen::Vector3d GetPosition(double t, const mav_planning_msgs::PolynomialTrajectory4D& traj_msg, int segment){
  Eigen::Vector3d position;
  position[0] = traj_msg.segments[segment].x[0] + traj_msg.segments[segment].x[1] * t +
                traj_msg.segments[segment].x[2] * t * t + traj_msg.segments[segment].x[3] * t * t * t +
                traj_msg.segments[segment].x[4] * t * t * t * t + traj_msg.segments[segment].x[5] * t * t * t * t * t;
  position[1] = traj_msg.segments[segment].y[0] + traj_msg.segments[segment].y[1] * t +
                traj_msg.segments[segment].y[2] * t * t + traj_msg.segments[segment].y[3] * t * t * t +
                traj_msg.segments[segment].y[4] * t * t * t * t + traj_msg.segments[segment].y[5] * t * t * t * t * t;
  position[2] = traj_msg.segments[segment].z[0] + traj_msg.segments[segment].z[1] * t +
                traj_msg.segments[segment].z[2] * t * t + traj_msg.segments[segment].z[3] * t * t * t +
                traj_msg.segments[segment].z[4] * t * t * t * t + traj_msg.segments[segment].z[5] * t * t * t * t * t;
  return position;
}
Eigen::Vector3d GetVelocity(double t, const mav_planning_msgs::PolynomialTrajectory4D& traj_msg, int segment){
  Eigen::Vector3d velocity;
  velocity[0] = traj_msg.segments[segment].x[1] + 2 * traj_msg.segments[segment].x[2] * t +
                3 * traj_msg.segments[segment].x[3] * t * t + 4 * traj_msg.segments[segment].x[4] * t * t * t +
                5 * traj_msg.segments[segment].x[5] * t * t * t * t;
  velocity[1] = traj_msg.segments[segment].y[1] + 2 * traj_msg.segments[segment].y[2] * t +
                3 * traj_msg.segments[segment].y[3] * t * t + 4 * traj_msg.segments[segment].y[4] * t * t * t +
                5 * traj_msg.segments[segment].y[5] * t * t * t * t;
  velocity[2] = traj_msg.segments[segment].z[1] + 2 * traj_msg.segments[segment].z[2] * t +
                3 * traj_msg.segments[segment].z[3] * t * t + 4 * traj_msg.segments[segment].z[4] * t * t * t +
                5 * traj_msg.segments[segment].z[5] * t * t * t * t;
  return velocity;
}
Eigen::Vector3d GetAcceleration(double t, const mav_planning_msgs::PolynomialTrajectory4D& traj_msg, int segment){
  Eigen::Vector3d acceleration;
  acceleration[0] = 2 * traj_msg.segments[segment].x[2] + 3 * 2 * traj_msg.segments[segment].x[3] * t +
                    4 * 3 * traj_msg.segments[segment].x[4] * t * t + 5 * 4 * traj_msg.segments[segment].x[5] * t * t * t;
  acceleration[1] = 2 * traj_msg.segments[segment].y[2] + 3 * 2 * traj_msg.segments[segment].y[3] * t +
                    4 * 3 * traj_msg.segments[segment].y[4] * t * t + 5 * 4 * traj_msg.segments[segment].y[5] * t * t * t;
  acceleration[2] = 2 * traj_msg.segments[segment].z[2] + 3 * 2 * traj_msg.segments[segment].z[3] * t +
                    4 * 3 * traj_msg.segments[segment].z[4] * t * t + 5 * 4 * traj_msg.segments[segment].z[5] * t * t * t;
  return acceleration;
}
Eigen::Vector3d GetJerk(double t, const mav_planning_msgs::PolynomialTrajectory4D& traj_msg, int segment){
  Eigen::Vector3d jerk;
  jerk[0] = 3 * 2 * traj_msg.segments[segment].x[3] + 4 * 3 * 2 * traj_msg.segments[segment].x[4] * t +
            5 * 4 * 3 * traj_msg.segments[segment].x[5] * t * t;
  jerk[1] = 3 * 2 * traj_msg.segments[segment].y[3] + 4 * 3 * 2 * traj_msg.segments[segment].y[4] * t +
            5 * 4 * 3 * traj_msg.segments[segment].y[5] * t * t;
  jerk[2] = 3 * 2 * traj_msg.segments[segment].z[3] + 4 * 3 * 2 * traj_msg.segments[segment].z[4] * t +
            5 * 4 * 3 * traj_msg.segments[segment].z[5] * t * t;
  return jerk;
}


void Visualize::trajectory_callback(const mav_planning_msgs::PolynomialTrajectory4D& traj_msg){
  // visualize in rviz (based on drawMavTrajectory)
  // This is just an empty extra marker that doesn't draw anything.
  mav_visualization::MarkerGroup additional_marker;
  // define variables
  mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
  visualization_msgs::MarkerArray markers;
  visualization_msgs::MarkerArray* marker_array = &markers;

  double num_segments = traj_msg.segments.size();
  std::vector<double> total_segment_times;
  double total_time = 0;
  for(int segment = 0; segment < num_segments; ++segment){
    double segment_time = traj_msg.segments[segment].segment_time.sec + traj_msg.segments[segment].segment_time.nsec/1e9;
    total_time += segment_time;
    total_segment_times.push_back(total_time);
  }

  double distance = 0.1;              // distance in [m] between 2 points
  double sampling_interval = 0.1;
  double min_time = 0.0;
  // double max_time = traj_msg.segments[0].segment_time.sec + traj_msg.segments[0].segment_time.nsec/1e9;
  double max_time = total_time;
  const size_t n_samples = (max_time - min_time) / sampling_interval + 1;       // size_t is like unsigned int can just store more

  // sample trajectory
  trajectory_points.resize(n_samples);
  int segment = 0;
  for(size_t i = 0; i < n_samples; ++i){
    double t;
    double t_complete = min_time + sampling_interval * i;

    if(t > total_segment_times[segment]){
      segment += 1;
    }
    if(segment != 0){
      t = t_complete - total_segment_times[segment - 1];
    }
    else{
      t = t_complete;
    }

    trajectory_points[i].degrees_of_freedom = mav_msgs::MavActuation::DOF4;
    trajectory_points[i].position_W[0] = GetPosition(t, traj_msg, segment)[0];
    trajectory_points[i].position_W[1] = GetPosition(t, traj_msg, segment)[1];
    trajectory_points[i].position_W[2] = GetPosition(t, traj_msg, segment)[2];
    trajectory_points[i].velocity_W[0] = GetVelocity(t, traj_msg, segment)[0];
    trajectory_points[i].velocity_W[1] = GetVelocity(t, traj_msg, segment)[1];
    trajectory_points[i].velocity_W[2] = GetVelocity(t, traj_msg, segment)[2];
    trajectory_points[i].acceleration_W[0] = GetAcceleration(t, traj_msg, segment)[0];
    trajectory_points[i].acceleration_W[1] = GetAcceleration(t, traj_msg, segment)[1];
    trajectory_points[i].acceleration_W[2] = GetAcceleration(t, traj_msg, segment)[2];
    trajectory_points[i].jerk_W[0] = GetJerk(t, traj_msg, segment)[0];
    trajectory_points[i].jerk_W[1] = GetJerk(t, traj_msg, segment)[1];
    trajectory_points[i].jerk_W[2] = GetJerk(t, traj_msg, segment)[2];
    trajectory_points[i].time_from_start_ns = static_cast<int64_t>(t * 1.e9);
  }

  marker_array->markers.clear();

  // set up line_strip
  visualization_msgs::Marker line_strip;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.color = mav_visualization::Color::Orange();
  line_strip.scale.x = 0.01;
  line_strip.ns = "path";

  // set up trajectory viz
  double accumulated_distance = 0.0;
  Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < trajectory_points.size(); ++i) {
    const mav_msgs::EigenTrajectoryPoint& trajectory_point = trajectory_points[i];
    accumulated_distance += (last_position - trajectory_point.position_W).norm();   // calculate accumulated distance passed by multiple points
    if (accumulated_distance > distance) {    // when accumulated distance bigger than some distance plot a point otherwise do nothing
      accumulated_distance = 0.0;   // set accumulated distance back to zero
      mav_msgs::EigenMavState mav_state;
      mav_msgs::EigenMavStateFromEigenTrajectoryPoint(trajectory_point, &mav_state);
      mav_state.orientation_W_B = trajectory_point.orientation_W_B;   // store orientation (roll, pitch obtained from trajectory point)

      // calculate the direction of the arrows based on position and orientation
      visualization_msgs::MarkerArray axes_arrows;
      mav_visualization::drawAxesArrows(mav_state.position_W,
                                        mav_state.orientation_W_B, 0.3, 0.3,
                                        &axes_arrows);
      // push_backMarkers
      marker_array->markers.reserve(marker_array->markers.size() +
                                    axes_arrows.markers.size());
      int i = 0;
      for (const visualization_msgs::Marker& marker : axes_arrows.markers) {
        marker_array->markers.push_back(marker);
        marker_array->markers.back().ns = "pose";
        i++;
      }

      // draw acceleration arrow
      visualization_msgs::Marker arrow;
      mav_visualization::drawArrowPoints(
          trajectory_point.position_W,
          trajectory_point.position_W + trajectory_point.acceleration_W,
          mav_visualization::Color((190.0 / 255.0), (81.0 / 255.0),
                                   (80.0 / 255.0)),
          0.3, &arrow);
      arrow.ns = mav_trajectory_generation::positionDerivativeToString(mav_trajectory_generation::derivative_order::ACCELERATION);
      marker_array->markers.push_back(arrow);

      // draw velocity arrow
      mav_visualization::drawArrowPoints(
          trajectory_point.position_W, trajectory_point.position_W + trajectory_point.velocity_W,
          mav_visualization::Color((80.0 / 255.0), (172.0 / 255.0),
                                   (196.0 / 255.0)),
          0.3, &arrow);
      arrow.ns = mav_trajectory_generation::positionDerivativeToString(mav_trajectory_generation::derivative_order::VELOCITY);
      marker_array->markers.push_back(arrow);

      mav_visualization::MarkerGroup tmp_marker(additional_marker);
      tmp_marker.transform(mav_state.position_W, mav_state.orientation_W_B);
      tmp_marker.getMarkers(marker_array->markers, 1.0, true);
    }

    // store last position
    last_position = trajectory_point.position_W;
    geometry_msgs::Point last_position_msg;
    tf::pointEigenToMsg(last_position, last_position_msg);
    line_strip.points.push_back(last_position_msg);
  }

  marker_array->markers.push_back(line_strip);
  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time::now();
  // setMarkerProperties
  int count = 0;
  for (visualization_msgs::Marker& marker : marker_array->markers) {
    marker.header = header;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = count;
    marker.lifetime = ros::Duration(0.0);
    ++count;
  }

  pub_markers_.publish(markers);
}

void Visualize::sent_trajectory_callback(const trajectory_msgs::MultiDOFJointTrajectory& traj_msg){
  for (int i =0; i<traj_msg.points.size(); i++){
    ROS_WARN("Position traj: %f", traj_msg.points[i].transforms[0].translation.x);}
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "Visualize");
  ros::NodeHandle n;
  ros::Rate loopRate(10); //how often the subscriber listens to topic

  ROS_WARN_STREAM("VISUALIZATION: VISUALIZE IS RUNNING");

  Visualize Visualize (n);

  ros::spin();

  return 0;
}
