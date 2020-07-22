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

#ifndef RMPCPP_EVAL_INTEGRATOR_2_H
#define RMPCPP_EVAL_INTEGRATOR_2_H
#include <rmpcpp/core/policy_container.h>
namespace rmpcpp {

/**
 * Simple Integrator to integrate a trajectory through
 * a policy field.
 *
 * Uses Trapezoidal integration with a user definable timestep.
 * Keeps a distance integral.
 *
 * \tparam TGeometry1 Geometry on which the integrator operates.
 *                   Has to be inherited from GeometryBase.
 */
template <class TGeometry1, class TGeometry2>
class TrapezoidalIntegrator {
 public:
  static_assert(std::is_base_of<GeometryBase<TGeometry1::K, TGeometry1::D>, TGeometry1>::value,
                "Geometry must inherit from GeometryBase with correct dimensionality");

  using Vector_x1 = typename TGeometry1::VectorX;
  using Matrix_x1 = typename TGeometry1::MatrixX;
  using Vector_x2 = typename TGeometry2::VectorX;
  using Matrix_x2 = typename TGeometry2::MatrixX;
  using Vector_q = typename TGeometry1::VectorQ;
  using Matrix_q = typename TGeometry1::MatrixQ;
  using PolicyBaseQ = PolicyBase<TGeometry1::D>;

  /**
   * Constructor to supply a policycontainer.
   */
  TrapezoidalIntegrator(PolicyContainer<TGeometry1> &policy1, PolicyContainer<TGeometry2> &policy2):
    policy1_(policy1),
    policy2_(policy2) {}

  /**
   *  Reset integrator to a specific state.
   */
  void resetTo(const Vector_q position, const Vector_q velocity = Vector_q::Zero(), const Vector_q acceleration = Vector_q::Zero()) {
    current_pos_ = position;
    current_vel_ = velocity;
    last_acc_ = acceleration;
    distance_ = 0.0;
    done_ = false;
    counter_ = 0;
  }

  void setX(const Vector_x1 pos_x1, Vector_x1 x1_dot,
            const Vector_x2 pos_x2, Vector_x2 x2_dot){
    pos_x1_ = pos_x1;
    x1_dot_ = x1_dot;
    pos_x2_ = pos_x2;
    x2_dot_ = x2_dot;
  }

  /**
   * Advance by dt and return position in configuration space.
   */
  Vector_q forwardIntegrate(float dt) {
    if (done_) {
      return current_pos_;
    }

    // relative start and end of integration
    const float a = 0.0;
    const float b = dt;

    // get position in manifold
    PolicyBaseQ acc_b1, acc_b2, acc_b_sum;
    Vector_q acc_b, vel_b, acc_a, vel_a;
    Vector_q dist_increment;
    acc_a = last_acc_;
    vel_a = current_vel_;

    // evaluate policy and get new accelerations
    acc_b1 = policy1_.evaluate(pos_x1_, x1_dot_);
    acc_b2 = policy2_.evaluate(pos_x2_, x2_dot_);
    std::vector<PolicyBaseQ> pol_vec;
    pol_vec.push_back(acc_b1);
    pol_vec.push_back(acc_b2);
    acc_b_sum = PolicyBaseQ::sum(pol_vec);
    acc_b = acc_b_sum.getf();

    // trapezoidal integration of acceleration.
    vel_b = vel_a + ((b - a) * (acc_a + acc_b) / 2.0);

    // trapezoidal integration of velocity
    dist_increment = (b - a) * (vel_a + vel_b) / 2.0;
    current_pos_ += dist_increment;
    distance_ += dist_increment.norm();
    last_acc_ = acc_b;
    current_vel_ = vel_b;

    // if we come to a rest, stop integrating
    if ((acc_b.norm() < 0.01 && current_vel_.norm() < 0.01) ){
      done_ = true;
    }

    counter_ += 1;

    return current_pos_;
  }

  /**
   * Returns true if we came to a rest.
   */
  bool isDone() { return done_; }

  /**
   * Returns the total distance in configuration space.
   */
  double totalDistance() { return distance_; }

  void getState(Vector_q *pos, Vector_q *vel, Vector_q *acc) {
    *pos = current_pos_;
    *vel = current_vel_;
    *acc = last_acc_;
  }

 private:
  PolicyContainer<TGeometry1> &policy1_;
  PolicyContainer<TGeometry2> &policy2_;
  bool done_{false};
  double distance_{0.0};
  Vector_q current_pos_{Vector_q::Zero()};
  Vector_q current_vel_{Vector_q::Zero()};
  Vector_x1 pos_x1_{Vector_x1::Zero()};
  Vector_x1 x1_dot_{Vector_x1::Zero()};
  Vector_x2 pos_x2_{Vector_x2::Zero()};
  Vector_x2 x2_dot_{Vector_x2::Zero()};
  Vector_q last_acc_{Vector_q::Zero()};
  uint counter_ = 0;
};

}  // namespace rmpcpp

#endif  // RMPCPP_EVAL_INTEGRATOR_H
