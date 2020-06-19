#include <drogone_transformation_lib/transformations.h>

namespace drogone_transformation_lib{

Transformations::Transformations(PinholeConstants pinhole_constants, CameraMounting camera_mounting, Eigen::Affine3d uav_pose):
  pinhole_constants_(pinhole_constants),
  camera_mounting_(camera_mounting){
    this->setMatrices(uav_pose);
};

void Transformations::setMatrices(Eigen::Affine3d uav_pose){
  // define camera pose w.r.t. body in correct variables
  double roll_C_B, pitch_C_B, yaw_C_B;
  roll_C_B = camera_mounting_.roll * M_PI / 180;
  pitch_C_B = camera_mounting_.pitch * M_PI / 180;
  yaw_C_B = camera_mounting_.yaw * M_PI / 180;
  Eigen::Vector3d t_C_B = camera_mounting_.translation;

  // define body pose w.r.t. world in correct variables
  double roll_B_W, pitch_B_W, yaw_B_W;
  Eigen::Quaterniond q(uav_pose.linear());
  roll_B_W = atan2(2.0 * (q.x() * q.y() + q.w() * q.z()),
                   q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  pitch_B_W = asin(-2.0 * (q.x() * q.z() - q.w() * q.y()));
  yaw_B_W = atan2(2.0 * (q.y() * q.z() + q.w() * q.x()),
                  q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
  Eigen::Vector3d t_B_W = uav_pose.translation();

  R_C_B_ << cos(pitch_C_B) * cos(yaw_C_B),
             cos(pitch_C_B) * sin(yaw_C_B),
             -sin(pitch_C_B),
            sin(roll_C_B) * sin(pitch_C_B) * cos(yaw_C_B) - cos(roll_C_B) * sin(yaw_C_B),
             sin(roll_C_B) * sin(pitch_C_B) * sin(yaw_C_B) + cos(roll_C_B) * cos(yaw_C_B),
             sin(roll_C_B) * cos(pitch_C_B),
            cos(roll_C_B) * sin(pitch_C_B) * cos(yaw_C_B) + sin(roll_C_B) * sin(yaw_C_B),
             cos(roll_C_B) * sin(pitch_C_B) * sin(yaw_C_B) - sin(roll_C_B) * cos(yaw_C_B),
             cos(roll_C_B) * cos(pitch_C_B);
  R_B_W_ << cos(pitch_B_W) * cos(yaw_B_W),
             cos(pitch_B_W) * sin(yaw_B_W),
             -sin(pitch_B_W),
            sin(roll_B_W) * sin(pitch_B_W) * cos(yaw_B_W) - cos(roll_B_W) * sin(yaw_B_W),
             sin(roll_B_W) * sin(pitch_B_W) * sin(yaw_B_W) + cos(roll_B_W) * cos(yaw_B_W),
             sin(roll_B_W) * cos(pitch_B_W),
            cos(roll_B_W) * sin(pitch_B_W) * cos(yaw_B_W) + sin(roll_B_W) * sin(yaw_B_W),
             cos(roll_B_W) * sin(pitch_B_W) * sin(yaw_B_W) - sin(roll_B_W) * cos(yaw_B_W),
             cos(roll_B_W) * cos(pitch_B_W);

  // calculate translational parts of the Transformation matrices
  Eigen::Vector3d trans_C_B = -R_C_B_ * t_C_B;
  Eigen::Vector3d trans_B_W = -R_B_W_ * t_B_W;

  // set Transformation matrices
  T_C_B_.linear() = R_C_B_;
  T_C_B_.translation() = trans_C_B;
  T_B_W_.linear() = R_B_W_;
  T_B_W_.translation() = trans_B_W;

  // set up camera matrix
  K_(0, 0) = pinhole_constants_.f_x;
  K_(1, 1) = pinhole_constants_.f_y;
  K_(2, 2) = 1;
  K_(0, 2) = pinhole_constants_.u_0;
  K_(1, 2) = pinhole_constants_.v_0;

  // set up inverse of camera Matrix
  K_inv_(0, 0) = 1 / pinhole_constants_.f_x;
  K_inv_(1, 1) = 1 / pinhole_constants_.f_y;
  K_inv_(2, 2) = 1;
  K_inv_(0, 2) = -pinhole_constants_.u_0 / pinhole_constants_.f_x;
  K_inv_(1, 2) = -pinhole_constants_.v_0 / pinhole_constants_.f_y;

  // set uav_position
  uav_pos_ = uav_pose.translation();
}

// pair: (normalized u and v, normalization constant (z-distance))
std::pair<Eigen::Matrix<double, 3, 1>, double> Transformations::PosWorld2Image(Eigen::Vector3d target_W){
  // calculate target pos in camera frame
  Eigen::Affine3d target_pose_W, target_pose_C;
  target_pose_W.translation() = target_W;
  target_pose_C = T_C_B_ * T_B_W_ * target_pose_W;
  Eigen::Vector3d target_C = target_pose_C.translation();

  // get u, v from camera matrix and target position in camera frame
  Eigen::Vector3d u_v;
  u_v = K_ * target_C;

  // normalize u and v
  Eigen::Matrix<double, 2, 1> u_v_normalized;
  u_v_normalized[0] = u_v[0] / u_v[2];
  u_v_normalized[1] = u_v[1] / u_v[2];

  // calculate distance to have full detection
  Eigen::Matrix<double, 3, 1> detection;
  double distance = (target_W - uav_pos_).norm();
  detection[0] = u_v_normalized[0];
  detection[1] = u_v_normalized[1];
  detection[2] = distance;

  // create the pair (normalized u and v, normalization constant) to return
  std::pair<Eigen::Matrix<double, 3, 1>, double> return_pair;
  return_pair.first = detection;
  return_pair.second = u_v[2];

  return return_pair;
}

Eigen::Matrix<double, 3, 1> Transformations::VelWorld2Image(Eigen::Vector3d target_pos_W, Eigen::Vector3d uav_vel, double normalization){
  // calculate relative target velocity (without tracker information that's just 0 - uav_vel)
  Eigen::Vector3d target_rel_vel_W = -uav_vel;

  // calculate target velocity in image frame
  Eigen::Vector3d target_vel_C;
  target_vel_C = R_C_B_ * R_B_W_ * target_rel_vel_W;

  // get u, v from camera matrix and target position in camera frame
  Eigen::Vector3d u_v_dot;
  u_v_dot = K_ * target_vel_C;

  // normalize u and v
  Eigen::Matrix<double, 2, 1> u_v_dot_normalized;
  u_v_dot_normalized[0] = u_v_dot[0] / normalization;
  u_v_dot_normalized[1] = u_v_dot[1] / normalization;

  // calculate velocity in distance
  Eigen::Vector3d connection_vec;
  connection_vec = target_pos_W - uav_pos_;
  double d_dot = -connection_vec.dot(uav_vel);
  Eigen::Matrix<double, 3, 1> image_vel;
  image_vel[0] = u_v_dot_normalized[0];
  image_vel[1] = u_v_dot_normalized[1];
  image_vel[2] = d_dot;

  return image_vel;
}

Eigen::Vector3d Transformations::PosImage2World(Eigen::Matrix<double, 3, 1> detection){
  // calculate normed camera x and y
  Eigen::Matrix<double, 3, 1> xy_C_norm;
  Eigen::Matrix<double, 3, 1> only_uv;
  only_uv = detection;
  only_uv[2] = 1;
  xy_C_norm = K_inv_ * only_uv;

  // calculate target position in camera frame
  Eigen::Vector3d target_C;
  target_C = xy_C_norm * detection[2];

  // transform target_C to target_W
  Eigen::Affine3d target_pose_W, target_pose_C;
  target_pose_C.translation() = target_C;
  target_pose_W = T_B_W_.inverse() * T_C_B_.inverse() * target_pose_C;
  Eigen::Vector3d target_W = target_pose_W.translation();

  return target_W;
}

} // namespace drogone_transformation_lib
