#include <drogone_transformation_lib/world_to_camera.h>

namespace drogone_transformation_lib{

WorldToCamera::WorldToCamera(PinholeConstants pinhole_constants, CameraMounting camera_mounting):
  pinhole_constants_(pinhole_constants),
  camera_mounting_(camera_mounting){
};

// pair: (normalized u and v, normalization constant (z-distance))
std::pair<Eigen::Matrix<double, 2, 1>, double> WorldToCamera::get_u_v(Eigen::Vector3d target_W, Eigen::Affine3d uav_pose){
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

  // define Rotation matrices
  Eigen::Matrix3d R_C_B, R_B_W;
  R_C_B << cos(pitch_C_B) * cos(yaw_C_B),
            cos(pitch_C_B) * sin(yaw_C_B),
            -sin(pitch_C_B),
           sin(roll_C_B) * sin(pitch_C_B) * cos(yaw_C_B) - cos(roll_C_B) * sin(yaw_C_B),
            sin(roll_C_B) * sin(pitch_C_B) * sin(yaw_C_B) + cos(roll_C_B) * cos(yaw_C_B),
            sin(roll_C_B) * cos(pitch_C_B),
           cos(roll_C_B) * sin(pitch_C_B) * cos(yaw_C_B) + sin(roll_C_B) * sin(yaw_C_B),
            cos(roll_C_B) * sin(pitch_C_B) * sin(yaw_C_B) - sin(roll_C_B) * cos(yaw_C_B),
            cos(roll_C_B) * cos(pitch_C_B);
  R_B_W << cos(pitch_B_W) * cos(yaw_B_W),
            cos(pitch_B_W) * sin(yaw_B_W),
            -sin(pitch_B_W),
           sin(roll_B_W) * sin(pitch_B_W) * cos(yaw_B_W) - cos(roll_B_W) * sin(yaw_B_W),
            sin(roll_B_W) * sin(pitch_B_W) * sin(yaw_B_W) + cos(roll_B_W) * cos(yaw_B_W),
            sin(roll_B_W) * cos(pitch_B_W),
           cos(roll_B_W) * sin(pitch_B_W) * cos(yaw_B_W) + sin(roll_B_W) * sin(yaw_B_W),
            cos(roll_B_W) * sin(pitch_B_W) * sin(yaw_B_W) - sin(roll_B_W) * cos(yaw_B_W),
            cos(roll_B_W) * cos(pitch_B_W);

  // calculate translational parts of the Transformation matrices
  Eigen::Vector3d trans_C_B = -R_C_B * t_C_B;
  Eigen::Vector3d trans_B_W = -R_B_W * t_B_W;

  // define Transformation matrices
  Eigen::Affine3d T_C_B, T_B_W;
  T_C_B.linear() = R_C_B;
  T_C_B.translation() = trans_C_B;
  T_B_W.linear() = R_B_W;
  T_B_W.translation() = trans_B_W;

  // calculate target pos in camera frame
  Eigen::Affine3d target_pose_W, target_pose_C;
  target_pose_W.translation() = target_W;
  target_pose_C = T_C_B * T_B_W * target_pose_W;
  Eigen::Vector3d target_C = target_pose_C.translation();

  // set up camera matrix
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(3, 3);
  K(0, 0) = pinhole_constants_.f_x;
  K(1, 1) = pinhole_constants_.f_y;
  K(2, 2) = 1;
  K(0, 2) = pinhole_constants_.u_0;
  K(1, 2) = pinhole_constants_.v_0;

  // get u, v from camera matrix and target position in camera frame
  Eigen::Vector3d u_v;
  u_v = K * target_C;

  // normalize u and v
  Eigen::Matrix<double, 2, 1> u_v_normalized;
  u_v_normalized[0] = u_v[0] / u_v[2];
  u_v_normalized[1] = u_v[1] / u_v[2];

  // create the pair (normalized u and v, normalization constant) to return
  std::pair<Eigen::Matrix<double, 2, 1>, double> return_pair;
  return_pair.first = u_v_normalized;
  return_pair.second = u_v[2];

  return return_pair;
}

Eigen::Matrix<double, 2, 1> WorldToCamera::get_u_v_dot(Eigen::Vector3d target_rel_vel_W, Eigen::Affine3d uav_pose, double normalization){
  // define camera orientation w.r.t. body in correct variables
  double roll_C_B, pitch_C_B, yaw_C_B;
  roll_C_B = camera_mounting_.roll * M_PI / 180;
  pitch_C_B = camera_mounting_.pitch * M_PI / 180;
  yaw_C_B = camera_mounting_.yaw * M_PI / 180;

  // define body orientation w.r.t. world in correct variables
  double roll_B_W, pitch_B_W, yaw_B_W;
  Eigen::Quaterniond q(uav_pose.linear());
  roll_B_W = atan2(2.0 * (q.x() * q.y() + q.w() * q.z()),
                   q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  pitch_B_W = asin(-2.0 * (q.x() * q.z() - q.w() * q.y()));
  yaw_B_W = atan2(2.0 * (q.y() * q.z() + q.w() * q.x()),
                  q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());

  // define Rotation matrices
  Eigen::Matrix3d R_C_B, R_B_W;
  R_C_B << cos(pitch_C_B) * cos(yaw_C_B),
            cos(pitch_C_B) * sin(yaw_C_B),
            -sin(pitch_C_B),
           sin(roll_C_B) * sin(pitch_C_B) * cos(yaw_C_B) - cos(roll_C_B) * sin(yaw_C_B),
            sin(roll_C_B) * sin(pitch_C_B) * sin(yaw_C_B) + cos(roll_C_B) * cos(yaw_C_B),
            sin(roll_C_B) * cos(pitch_C_B),
           cos(roll_C_B) * sin(pitch_C_B) * cos(yaw_C_B) + sin(roll_C_B) * sin(yaw_C_B),
            cos(roll_C_B) * sin(pitch_C_B) * sin(yaw_C_B) - sin(roll_C_B) * cos(yaw_C_B),
            cos(roll_C_B) * cos(pitch_C_B);
  R_B_W << cos(pitch_B_W) * cos(yaw_B_W),
            cos(pitch_B_W) * sin(yaw_B_W),
            -sin(pitch_B_W),
           sin(roll_B_W) * sin(pitch_B_W) * cos(yaw_B_W) - cos(roll_B_W) * sin(yaw_B_W),
            sin(roll_B_W) * sin(pitch_B_W) * sin(yaw_B_W) + cos(roll_B_W) * cos(yaw_B_W),
            sin(roll_B_W) * cos(pitch_B_W),
           cos(roll_B_W) * sin(pitch_B_W) * cos(yaw_B_W) + sin(roll_B_W) * sin(yaw_B_W),
            cos(roll_B_W) * sin(pitch_B_W) * sin(yaw_B_W) - sin(roll_B_W) * cos(yaw_B_W),
            cos(roll_B_W) * cos(pitch_B_W);

  Eigen::Vector3d target_vel_C;
  target_vel_C = R_C_B * R_B_W * target_rel_vel_W;

  // set up camera matrix
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(3, 3);
  K(0, 0) = pinhole_constants_.f_x;
  K(1, 1) = pinhole_constants_.f_y;
  K(2, 2) = 1;
  K(0, 2) = pinhole_constants_.u_0;
  K(1, 2) = pinhole_constants_.v_0;

  // get u, v from camera matrix and target position in camera frame
  Eigen::Vector3d u_v_dot;
  u_v_dot = K * target_vel_C;

  // normalize u and v
  Eigen::Matrix<double, 2, 1> u_v_dot_normalized;
  u_v_dot_normalized[0] = u_v_dot[0] / normalization;
  u_v_dot_normalized[1] = u_v_dot[1] / normalization;

  return u_v_dot_normalized;
}


} // namespace drogone_transformation_lib
