/**
 * This file is part of RMPCPP
 *
 * Copyright (C) 2020 Michael Pantic <mpantic at ethz dot ch>
 *
 * RMPCPP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RMPCPP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RMPCPP. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RMPCPP_POLICIES_CAMERA_TARGET_POLICY_H_
#define RMPCPP_POLICIES_CAMERA_TARGET_POLICY_H_
#include <rmpcpp/core/policy_base.h>
#include <ros/ros.h>
namespace rmpcpp {

/**
 * Defines a simple n dimensional target policy, as described in [1].
 * \tparam n Dimensionality of geometry
 */
template <int n>
class CameraTargetPolicy : public PolicyBase<n> {
  using Vector = typename PolicyBase<n>::Vector;
  using Matrix = typename PolicyBase<n>::Matrix;

 public:
  /**
   * Sets up the policy.
   * A is the metric to be used.
   * beta and c are tuning parameters.
   */
  CameraTargetPolicy(Vector target, Matrix A, double beta, double c)
      : target_(target), beta_(beta), c_(c) {
    this->A_ = A;
  }

  CameraTargetPolicy(Vector target, double beta, double c)
      : target_(target), beta_(beta), c_(c) {}

  CameraTargetPolicy(Vector target) : target_(target) {}

  virtual void setState(const Vector &x, const Vector &x_dot) override {
    if(!target_passed_){
      this->f_ = s(this->space_->minus(target_, x)) * max_acc_ - beta_ * x_dot;
    }
    else{
      this->f_ = Vector::Zero();
    }
  }

  void setMaxAcc(double max_acc){
    max_acc_ = max_acc;
  }

  Vector getAccField(){
    return this->f_;
  }

  void setTargetPassed(bool target_passed){
    target_passed_ = target_passed;
  }

 protected:
  /**
   *  Normalization helper function.
   */
  inline Vector s(Vector x) { return x / h(this->space_->norm(x)); }

  /**
   * Softmax helper function
   */
  inline double h(const double z) { return (z + sigma_ * c_ * log(1 + exp(-2 * c_ * z / sigma_))); }

  Vector target_;
  double beta_{8.0}, c_{0.005};
  double sigma_{sqrt((1024 / 2) * (1024 / 2) + (768 / 2) * (768 / 2))};
  double max_acc_ = 200;
  bool target_passed_;
};

}  // namespace rmpcpp

#endif  // RMPCPP_POLICIES_TARGET_POLICY_H_
