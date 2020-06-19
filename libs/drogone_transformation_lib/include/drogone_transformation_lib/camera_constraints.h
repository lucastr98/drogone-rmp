#include <ros/ros.h>
#include <Eigen/Dense>

namespace drogone_transformation_lib{

struct PinholeConstants {
  double f_x;
  double f_y;
  double u_0;
  double v_0;
};

struct CameraMounting {
  double roll;    // in [deg]
  double pitch;   // in [deg]
  double yaw;     // in [deg]
  Eigen::Vector3d translation;
};

} // namespace drogone_transformation_lib
