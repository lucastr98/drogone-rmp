#include <drogone_transformation_lib/camera_constraints.h>
#include <utility>

namespace drogone_transformation_lib{

class WorldToCamera{
  public:
    WorldToCamera(PinholeConstants pinhole_constants, CameraMounting camera_mounting);
    std::pair<Eigen::Matrix<double, 2, 1>, double> get_u_v(Eigen::Vector3d target_W, Eigen::Affine3d uav_pose);
    Eigen::Matrix<double, 2, 1> get_u_v_dot(Eigen::Vector3d target_W, Eigen::Affine3d uav_pose, double normalization);
  private:
    PinholeConstants pinhole_constants_;
    CameraMounting camera_mounting_;

};

} // namespace drogone_transformation_lib
