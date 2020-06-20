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
class DistanceGeometry : public GeometryBase<1, 4> {
  // type alias for readability.
  using base = GeometryBase<1, 4>;

 protected:
  /**
   * Return jacobian.
   */
  virtual typename base::J_phi J(const typename base::VectorX &x,
                                 const typename base::VectorX &x_dot) const {
    ACK_UNUSED(x);
    ACK_UNUSED(x_dot);

    base::J_phi mtx_j(base::J_phi::Identity());

    mtx_j(0, 0) = (q_[0] - target_pos_[0]) / sqrt((q_[0] - target_pos_[0]) * (q_[0] - target_pos_[0]) +
                                                  (q_[1] - target_pos_[1]) * (q_[1] - target_pos_[1]) +
                                                  (q_[2] - target_pos_[2]) * (q_[2] - target_pos_[2]));
    mtx_j(0, 1) = (q_[1] - target_pos_[1]) / sqrt((q_[0] - target_pos_[0]) * (q_[0] - target_pos_[0]) +
                                                  (q_[1] - target_pos_[1]) * (q_[1] - target_pos_[1]) +
                                                  (q_[2] - target_pos_[2]) * (q_[2] - target_pos_[2]));
    mtx_j(0, 2) = (q_[2] - target_pos_[2]) / sqrt((q_[0] - target_pos_[0]) * (q_[0] - target_pos_[0]) +
                                                  (q_[1] - target_pos_[1]) * (q_[1] - target_pos_[1]) +
                                                  (q_[2] - target_pos_[2]) * (q_[2] - target_pos_[2]));
    mtx_j(0, 3) = 0;

    return mtx_j;
  }

  Eigen::Vector3d target_pos_;
  typename base::VectorQ q_;

 public:
  // position of the target in world frame
  void SetTargetPos(Eigen::Vector3d p){
    target_pos_ = p;
  }

  void setQ(Eigen::Matrix<double, 4, 1> q){
    q_ = q;
  }

  virtual void convertToX(const typename base::VectorQ &q, typename base::VectorX *x,
                          const VectorQ &q_dot = VectorQ::Zero(),
                          VectorX *x_dot = nullptr) const {

    ACK_UNUSED(x);
    ACK_UNUSED(x_dot);
    ACK_UNUSED(q);
    ACK_UNUSED(q_dot);

    std::cout << "DISTANCE GEOM: SET CONVERT TO X" << std::endl;
  }

  virtual void convertToQ(const typename base::VectorX &x, typename base::VectorQ *q) const {

    ACK_UNUSED(x);
    ACK_UNUSED(q);

    std::cout << "DISTANCE GEOM: SET CONVERT TO Q" << std::endl;

  }
};
}  // namespace rmpcpp
