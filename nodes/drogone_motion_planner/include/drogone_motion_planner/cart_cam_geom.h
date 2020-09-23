#include <rmpcpp/core/geometry_base.h>

// Macro to mark unused variables s.t. no unused warning appears.
#define ACK_UNUSED(expr) \
  do {                   \
    (void)(expr);        \
  } while (0)

namespace rmpcpp {

/**
 * Example of a cylindrical geometry that maps to a plane.
 * Here, the task space is the unit sphere, and the configuration space is R^3
 *
 * X Task space coordinates are: theta, rho, z
 * Q Configuration space coordinates are: x,y,z
 */
class CartesianCameraGeometry : public GeometryBase<2, 4> {
  // type alias for readability.
  using base = GeometryBase<2, 4>;

 protected:
  /**
   * Return jacobian.
   */
  virtual typename base::J_phi J(const typename base::VectorX &x,
                                 const typename base::VectorX &x_dot) const {
    ACK_UNUSED(x);
    ACK_UNUSED(x_dot);

    base::J_phi mtx_j(base::J_phi::Identity());

    mtx_j(0, 0) = -f_x_ * cos(q_[3]) / (target_pos_[2] - q_[2]);
    mtx_j(0, 1) = -f_x_ * sin(q_[3]) / (target_pos_[2] - q_[2]);
    mtx_j(0, 2) = -u_0_ / (target_pos_[2] - q_[2]) +
                  1 / (target_pos_[2] - q_[2]) / (target_pos_[2] - q_[2]) *
                  (f_x_ * (cos(q_[3]) * (target_pos_[0] - q_[0]) + sin(q_[3]) *
                  (target_pos_[1] - q_[1])) + u_0_ * (target_pos_[2] - q_[2])); // --> returned nan
    mtx_j(0, 3) = f_x_ / (target_pos_[2] - q_[2]) * (sin(q_[3]) * (q_[0] - target_pos_[0]) -
                                                     cos(q_[3]) * (q_[1] - target_pos_[1]));
    mtx_j(1, 0) = f_y_ * sin(q_[3]) / (target_pos_[2] - q_[2]);
    mtx_j(1, 1) = -f_y_ * cos(q_[3]) / (target_pos_[2] - q_[2]);
    mtx_j(1, 2) = -v_0_ / (target_pos_[2] - q_[2]) +
                  1 / (target_pos_[2] - q_[2]) / (target_pos_[2] - q_[2]) *
                  (f_y_ * (-sin(q_[3]) * (target_pos_[0] - q_[0]) + cos(q_[3]) *
                  (target_pos_[1] - q_[1])) + v_0_ * (target_pos_[2] - q_[2])); // --> returned nan
    mtx_j(1, 3) = f_y_ / (target_pos_[2] - q_[2]) * (cos(q_[3]) * (q_[0] - target_pos_[0]) +
                                                     sin(q_[3]) * (q_[1] - target_pos_[1]));


    // std::cout << "Jacobian (camera): " << mtx_j(0, 0) << ", " << mtx_j(0, 1) << ", " << mtx_j(0, 2) << ", " << mtx_j(0, 3) << ", "
    // << mtx_j(1, 0) << ", " << mtx_j(1, 1) << ", " << mtx_j(1, 2) << ", " << mtx_j(1, 3) << std::endl;

    mtx_j(0, 3) = 0;
    mtx_j(1, 3) = 0;

    if(mode_ == "catch"){
      mtx_j(0, 2) = 0;
      mtx_j(1, 2) = 0;
    }
    else if(mode_ == "follow"){
      mtx_j(0, 2) /= 2;
      mtx_j(1, 2) /= 2;
    }
    else if(mode_ == "recover"){
      mtx_j(0, 2) *= 2;
      mtx_j(1, 2) *= 2;
    }
    else if(mode_ == "recover_distance"){
      mtx_j(0, 2) = 0;
      mtx_j(1, 2) = 0;
    }

    return mtx_j;
  }

  Eigen::Vector3d target_pos_;
  double f_x_, f_y_, u_0_, v_0_, q_x_;
  typename base::VectorQ q_;
  std::string mode_;

 public:
  // position of the target in world frame
  void SetTargetPos(Eigen::Vector3d p){
    target_pos_ = p;
  }

  // elems: (f_x, f_y, u_0, v_0)
  void SetK(Eigen::Matrix<double, 4, 1> elems){
    f_x_ = elems[0];
    f_y_ = elems[1];
    u_0_ = elems[2];
    v_0_ = elems[3];
  }

  void setQ(Eigen::Matrix<double, 4, 1> q){
    q_ = q;
  }

  void setMode(std::string mode){
    mode_ = mode;
  }

  virtual void convertToX(const typename base::VectorQ &q, typename base::VectorX *x,
                          const VectorQ &q_dot = VectorQ::Zero(),
                          VectorX *x_dot = nullptr) const {

    ACK_UNUSED(x);
    ACK_UNUSED(x_dot);
    ACK_UNUSED(q);
    ACK_UNUSED(q_dot);

    std::cout << "SET CONVERT TO X" << std::endl;
  }

  virtual void convertToQ(const typename base::VectorX &x, typename base::VectorQ *q) const {

    ACK_UNUSED(x);
    ACK_UNUSED(q);

    std::cout << "SET CONVERT TO Q" << std::endl;

  }
};
}  // namespace rmpcpp
