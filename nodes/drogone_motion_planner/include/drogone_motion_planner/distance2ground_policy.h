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

#ifndef RMPCPP_POLICIES_DISTANCE2GROUND_POLICY_H_
#define RMPCPP_POLICIES_DISTANCE2GROUND_POLICY_H_
#include <rmpcpp/core/policy_base.h>
#include <ros/ros.h>
namespace rmpcpp {

/**
 * Defines a simple n dimensional target policy, as described in [1].
 * \tparam n Dimensionality of geometry
 */
template <int n>
class Distance2GroundPolicy : public PolicyBase<n> {
  using Vector = typename PolicyBase<n>::Vector;
  using Matrix = typename PolicyBase<n>::Matrix;

 public:
  /**
   * Sets up the policy.
   * target is the target to move to.
   * A is the metric to be used.
   * alpha, beta and c are tuning parameters.
   */
  Distance2GroundPolicy(Matrix A, double alpha, double beta)
      : alpha_(alpha), beta_(beta) {
    this->A_ = A;
  }

  Distance2GroundPolicy(double alpha, double beta)
      : alpha_(alpha), beta_(beta) {}


  virtual void setState(const Vector &x, const Vector &x_dot) override {
    this->f_ = (alpha_ / x[0] - beta_ * x_dot[0]) * x / x[0];
    if(this->f_[0] > max_acc_){
      this->f_[0] = max_acc_;
    }
    // std::cout << this->f_ << std::endl;
  }

  void setMaxAcc(double max_acc){
    max_acc_ = max_acc;
  }

  Vector getAccField(){
    return this->f_;
  }

 protected:

  double alpha_{1.0}, beta_{8.0};
  double max_acc_;
};

}  // namespace rmpcpp

#endif  // RMPCPP_POLICIES_POLICY_H_
