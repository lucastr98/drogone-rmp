#include <drogone_transformation_lib/camera_constraints.h>
#include <utility>

namespace drogone_transformation_lib{

class Transformations{
  public:
    Transformations();
    void setCameraConfig(PinholeConstants pinhole_constants, CameraMounting camera_mounting);
    void setMatrices(Eigen::Affine3d uav_pose);
    std::pair<Eigen::Matrix<double, 3, 1>, double> PosWorld2Image(Eigen::Vector3d target_W);
    Eigen::Matrix<double, 3, 1> VelWorld2Image(Eigen::Vector3d target_pos_W, Eigen::Vector3d uav_vel, double normalization);
    Eigen::Vector3d PosImage2World(Eigen::Matrix<double, 3, 1> detection);
  private:
    PinholeConstants pinhole_constants_;
    CameraMounting camera_mounting_;

    Eigen::Matrix3d R_C_B_;
    Eigen::Matrix3d R_B_W_;
    Eigen::Affine3d T_C_B_;
    Eigen::Affine3d T_B_W_;
    Eigen::Matrix3d K_;
    Eigen::Matrix3d K_inv_;
    Eigen::Vector3d uav_pos_;
};

} // namespace drogone_transformation_lib
