// Copyright 2018-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/kalman_filter/time_delay_kalman_filter.hpp"

#include <Eigen/Cholesky>

#include <iostream>
#include <utility>

namespace autoware::kalman_filter
{
void TimeDelayKalmanFilter::init(
  const Eigen::MatrixXd & x, const Eigen::MatrixXd & P0, const int max_delay_step)
{
  max_delay_step_ = max_delay_step;
  dim_x_ = x.rows();
  dim_x_ex_ = dim_x_ * max_delay_step;

  x_ = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
  P_ = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);

  for (int i = 0; i < max_delay_step_; ++i) {
    x_.block(i * dim_x_, 0, dim_x_, 1) = x;
    P_.block(i * dim_x_, i * dim_x_, dim_x_, dim_x_) = P0;
  }
}

Eigen::MatrixXd TimeDelayKalmanFilter::getLatestX() const
{
  return x_.block(0, 0, dim_x_, 1);
}

Eigen::MatrixXd TimeDelayKalmanFilter::getLatestP() const
{
  return P_.block(0, 0, dim_x_, dim_x_);
}

bool TimeDelayKalmanFilter::predictWithDelay(
  const Eigen::MatrixXd & x_next, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q)
{
  /*
   * time delay model:
   *
   *     [A   0   0]      [P11   P12   P13]      [Q   0   0]
   * A = [I   0   0], P = [P21   P22   P23], Q = [0   0   0]
   *     [0   I   0]      [P31   P32   P33]      [0   0   0]
   *
   * covariance calculation in prediction : P = A * P * A' + Q
   *
   *     [A*P11*A'*+Q  A*P11  A*P12]
   * P = [     P11*A'    P11    P12]
   *     [     P21*A'    P21    P22]
   */

  const int d_dim_x = dim_x_ex_ - dim_x_;

  /* slide states in the time direction */
  Eigen::MatrixXd x_tmp = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
  x_tmp.block(0, 0, dim_x_, 1) = x_next;
  x_tmp.block(dim_x_, 0, d_dim_x, 1) = x_.block(0, 0, d_dim_x, 1);
  x_ = std::move(x_tmp);

  /* update P with delayed measurement A matrix structure */
  Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);
  P_tmp.block(0, 0, dim_x_, dim_x_) = A * P_.block(0, 0, dim_x_, dim_x_) * A.transpose() + Q;
  P_tmp.block(0, dim_x_, dim_x_, d_dim_x) = A * P_.block(0, 0, dim_x_, d_dim_x);
  P_tmp.block(dim_x_, 0, d_dim_x, dim_x_) = P_.block(0, 0, d_dim_x, dim_x_) * A.transpose();
  P_tmp.block(dim_x_, dim_x_, d_dim_x, d_dim_x) = P_.block(0, 0, d_dim_x, d_dim_x);
  P_ = std::move(P_tmp);

  return true;
}

bool TimeDelayKalmanFilter::updateWithDelay(
  const Eigen::MatrixXd & y, const Eigen::MatrixXd & C, const Eigen::MatrixXd & R,
  const int delay_step)
{
  if (delay_step < 0 || delay_step >= max_delay_step_) {
    std::cerr << "Invalid delay step: " << delay_step << ". Update ignored." << std::endl;
    return false;
  }

  if (C.cols() != dim_x_) {
    std::cerr << "Dimension mismatch in C matrix." << std::endl;
    return false;
  }

  const int dim_x = dim_x_;
  const int start_idx = dim_x * delay_step;

  /*
   * Calculate Innovation: e
   *
   * Theoretical formula:
   * e = y - C_ex * x
   * where C_ex is the observation matrix for the augmented state (extended with zeros).
   *
   * Optimization:
   * Since C_ex is sparse (mostly zeros), full multiplication is inefficient.
   * We extract only the relevant state block x_d corresponding to the delay.
   * e = y - C * x_d
   */
  const auto x_d = x_.block(start_idx, 0, dim_x, 1);
  const Eigen::MatrixXd e = y - C * x_d;

  /*
   * Calculate Innovation Covariance: S
   *
   * Theoretical formula:
   * S = C_ex * P * C_ex^T + R
   *
   * Optimization:
   * Due to the sparsity structure C_ex = [0...C...0], this simplifies to:
   * S = C * P_dd * C^T + R
   * where P_dd is the diagonal block of P corresponding to the delayed state.
   *
   * Note: P_dd can be simplified to a diagonal block instead of a row/column block
   * because the sparse observation matrix is multiplied from both sides.
   */
  const auto P_dd = P_.block(start_idx, start_idx, dim_x, dim_x);

  Eigen::MatrixXd S = R;
  S.noalias() += C * P_dd * C.transpose();

  /*
   * Prepare Kalman Gain Numerator: P_CT
   *
   * Theoretical term: P * C_ex^T
   *
   * Optimization:
   * This simplifies to P_star_d * C^T, where P_star_d is the column block of P.
   * This represents the covariance between ALL states (current & past) and the delayed state.
   */
  const auto P_star_d = P_.middleCols(start_idx, dim_x);
  const Eigen::MatrixXd P_CT = P_star_d * C.transpose();

  /*
   * Calculate Kalman Gain: K
   *
   * Theoretical formula:
   * K = P_CT * S^-1
   *
   * Numerical Stability Optimization:
   * Avoid explicit inversion of S. Instead, solve the linear system using Cholesky decomposition
   * (LLT). Since Eigen's solve() handles `Ax = B`, we transform the equation:
   *
   * K = P_CT * S^-1
   * K^T = (P_CT * S^-1)^T
   * K^T = (S^-1)^T * P_CT^T   (Identity: (AB)^T = B^T * A^T)
   * K^T = S^-1 * P_CT^T       (S is symmetric, so (S^-1)^T = S^-1)
   * S * K^T = P_CT^T
   *
   * We solve for K^T (denoted as K_transposed) and then transpose back.
   */
  Eigen::LLT<Eigen::MatrixXd> lltOfS(S);
  if (lltOfS.info() != Eigen::Success) {
    std::cerr << "LLT decomposition failed. S matrix might not be positive definite." << std::endl;
    return false;
  }

  const Eigen::MatrixXd K_transposed = lltOfS.solve(P_CT.transpose());
  const Eigen::MatrixXd K = K_transposed.transpose();

  // Update state and covariance
  x_.noalias() += K * e;
  P_.noalias() -= P_CT * K.transpose();

  return true;
}
}  // namespace autoware::kalman_filter
