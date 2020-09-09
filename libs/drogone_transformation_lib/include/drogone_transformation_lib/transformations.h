#include <drogone_transformation_lib/camera_constraints.h>
#include <utility>
#include <random>

namespace drogone_transformation_lib{

class Transformations{
  public:
    Transformations();
    void setCameraConfig(PinholeConstants pinhole_constants, CameraMounting camera_mounting);
    void setMatrices(Eigen::Affine3d uav_pose);
    std::pair<Eigen::Matrix<double, 3, 1>, double> PosWorld2Image(Eigen::Vector3d target_W, bool noise);
    Eigen::Matrix<double, 3, 1> VelWorld2Image(Eigen::Vector3d target_pos_W, Eigen::Vector3d uav_vel, double normalization);
    Eigen::Vector3d PosImage2World(Eigen::Matrix<double, 3, 1> detection);
    void setNoiseParams(double d_drone, double tol_u, double tol_v, double tol_d);
  private:
    PinholeConstants pinhole_constants_;
    CameraMounting camera_mounting_;

    Eigen::Matrix3d R_C_B_;
    Eigen::Matrix3d R_B_W_;
    Eigen::Affine3d T_C_B_;
    Eigen::Affine3d T_B_W_;
    Eigen::Matrix3d K_;
    Eigen::Matrix3d K_inv_;
    Eigen::Affine3d uav_pose_;

    // noise parameters
    double d_drone_;
    double tol_u_;
    double tol_v_;
    double tol_d_;
    std::default_random_engine rand_gen_;
    std::normal_distribution<double> rand_noise_x_C_;
    std::normal_distribution<double> rand_noise_y_C_;
    std::normal_distribution<double> rand_noise_z_C_;
};

} // namespace drogone_transformation_lib
