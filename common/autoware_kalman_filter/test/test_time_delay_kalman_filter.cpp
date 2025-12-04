// Copyright 2023 The Autoware Foundation
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
#include <gtest/gtest.h>
#include <Eigen/Dense>

using autoware::kalman_filter::TimeDelayKalmanFilter;

// Helper function to shift the extended state for ground truth verification
void ground_truth_predict(
  Eigen::MatrixXd & x_ex, Eigen::MatrixXd & P_ex,
  const Eigen::MatrixXd & x_next, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q,
  int dim_x, int dim_x_ex)
{
  // 1. Shift existing states down (old t becomes t-1)
  // The bottom-most block is discarded, 0 moves to 1, etc.
  Eigen::MatrixXd x_shifted = Eigen::MatrixXd::Zero(dim_x_ex, 1);
  x_shifted.block(dim_x, 0, dim_x_ex - dim_x, 1) = x_ex.block(0, 0, dim_x_ex - dim_x, 1);

  // 2. Insert new prediction at the top (t)
  // Note: specific to TDKF implementation, usually x_next is provided or calculated as A*x
  // In your test case, you provided x_next explicitly.
  x_shifted.block(0, 0, dim_x, 1) = x_next;
  x_ex = x_shifted;

  // 3. Update Covariance
  // P_new = [ A*P_00*A'+Q   A*P_01... ]
  //         [ ...           P_shifted ]
  Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(dim_x_ex, dim_x_ex);

  // Top-Left: Standard prediction covariance
  P_tmp.block(0, 0, dim_x, dim_x) =
    A * P_ex.block(0, 0, dim_x, dim_x) * A.transpose() + Q;

  // Top-Right: Correlation between new state and past states
  P_tmp.block(0, dim_x, dim_x, dim_x_ex - dim_x) =
    A * P_ex.block(0, 0, dim_x, dim_x_ex - dim_x);

  // Bottom-Left: Transpose of Top-Right
  P_tmp.block(dim_x, 0, dim_x_ex - dim_x, dim_x) =
    P_ex.block(0, 0, dim_x_ex - dim_x, dim_x) * A.transpose();

  // Bottom-Right: Shifted previous covariance
  P_tmp.block(dim_x, dim_x, dim_x_ex - dim_x, dim_x_ex - dim_x) =
    P_ex.block(0, 0, dim_x_ex - dim_x, dim_x_ex - dim_x);

  P_ex = P_tmp;
}

void ground_truth_update(
  Eigen::MatrixXd & x_ex, Eigen::MatrixXd & P_ex,
  const Eigen::MatrixXd & y, const Eigen::MatrixXd & C, const Eigen::MatrixXd & R,
  int delay_step, int dim_x, int dim_y, int dim_x_ex)
{
  // Construct Extended C matrix (all zeros except at the delayed block)
  Eigen::MatrixXd C_ex = Eigen::MatrixXd::Zero(dim_y, dim_x_ex);
  C_ex.block(0, delay_step * dim_x, dim_y, dim_x) = C;

  // Standard Kalman Update on the Extended State
  const Eigen::MatrixXd PCT = P_ex * C_ex.transpose();
  const Eigen::MatrixXd K = PCT * ((R + C_ex * PCT).inverse());
  const Eigen::MatrixXd y_pred = C_ex * x_ex;

  x_ex = x_ex + K * (y - y_pred);
  P_ex = P_ex - K * (C_ex * P_ex);
}

TEST(time_delay_kalman_filter, comprehensive_lifecycle)
{
  TimeDelayKalmanFilter td_kf_;

  // --- Constants ---
  const int dim_x = 3;
  const int max_delay_step = 5;
  const int dim_x_ex = dim_x * max_delay_step;
  const double epsilon = 1e-5;

  // --- Initialization Data ---
  Eigen::MatrixXd x_t(dim_x, 1);
  x_t << 1.0, 2.0, 3.0;
  Eigen::MatrixXd P_t = Eigen::MatrixXd::Identity(dim_x, dim_x) * 0.1;

  // --- Ground Truth State (Extended) ---
  Eigen::MatrixXd x_ex_gt = Eigen::MatrixXd::Zero(dim_x_ex, 1);
  Eigen::MatrixXd P_ex_gt = Eigen::MatrixXd::Zero(dim_x_ex, dim_x_ex);

  // Fill ground truth with initial state repeated (assuming steady state init)
  for (int i = 0; i < max_delay_step; ++i) {
    x_ex_gt.block(i * dim_x, 0, dim_x, 1) = x_t;
    P_ex_gt.block(i * dim_x, i * dim_x, dim_x, dim_x) = P_t;
  }

  // --- 1. Init ---
  td_kf_.init(x_t, P_t, max_delay_step);

  Eigen::MatrixXd x_check = td_kf_.getLatestX();
  Eigen::MatrixXd P_check = td_kf_.getLatestP();

  EXPECT_TRUE(x_check.isApprox(x_t, epsilon));
  EXPECT_TRUE(P_check.isApprox(P_t, epsilon));

  // --- Setup Models ---
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x, dim_x) * 2.0;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(dim_x, dim_x) * 0.01;
  Eigen::MatrixXd C = Eigen::MatrixXd::Identity(dim_x, dim_x) * 0.5;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(dim_x, dim_x) * 0.001;
  Eigen::MatrixXd x_next(dim_x, 1);
  x_next << 2.0, 4.0, 6.0;

  // ==========================================================
  // Test Case 2: Prediction
  // ==========================================================

  // Update Ground Truth
  ground_truth_predict(x_ex_gt, P_ex_gt, x_next, A, Q, dim_x, dim_x_ex);

  // Update TDKF
  EXPECT_TRUE(td_kf_.predictWithDelay(x_next, A, Q));

  // Check
  x_check = td_kf_.getLatestX();
  P_check = td_kf_.getLatestP();

  // Verify top block (current time) matches
  EXPECT_TRUE(x_check.isApprox(x_ex_gt.block(0,0,dim_x,1), epsilon));
  EXPECT_TRUE(P_check.isApprox(P_ex_gt.block(0,0,dim_x,dim_x), epsilon));

  // ==========================================================
  // Test Case 3: Update with Delay (Step = 2)
  // ==========================================================

  Eigen::MatrixXd y_delayed(dim_x, 1);
  y_delayed << 1.05, 2.05, 3.05;
  int delay_step = 2;

  // Update Ground Truth
  ground_truth_update(x_ex_gt, P_ex_gt, y_delayed, C, R, delay_step, dim_x, dim_x, dim_x_ex);

  // Update TDKF
  EXPECT_TRUE(td_kf_.updateWithDelay(y_delayed, C, R, delay_step));

  // Check
  x_check = td_kf_.getLatestX();
  P_check = td_kf_.getLatestP();

  EXPECT_TRUE(x_check.isApprox(x_ex_gt.block(0,0,dim_x,1), epsilon));
  EXPECT_TRUE(P_check.isApprox(P_ex_gt.block(0,0,dim_x,dim_x), epsilon));

  // ==========================================================
  // Test Case 4: Update with Zero Delay (Current Time)
  // ==========================================================
  // Validates that the TDKF behaves like a standard KF when delay is 0

  Eigen::MatrixXd y_current(dim_x, 1);
  y_current << 2.1, 4.1, 6.1; // Close to current state x_next
  delay_step = 0;

  // Update Ground Truth
  ground_truth_update(x_ex_gt, P_ex_gt, y_current, C, R, delay_step, dim_x, dim_x, dim_x_ex);

  // Update TDKF
  EXPECT_TRUE(td_kf_.updateWithDelay(y_current, C, R, delay_step));

  // Check
  x_check = td_kf_.getLatestX();
  P_check = td_kf_.getLatestP();

  EXPECT_TRUE(x_check.isApprox(x_ex_gt.block(0,0,dim_x,1), epsilon));
  EXPECT_TRUE(P_check.isApprox(P_ex_gt.block(0,0,dim_x,dim_x), epsilon));

  // ==========================================================
  // Test Case 5: Update with Max Delay
  // ==========================================================
  // Validates updating the tail of the buffer

  Eigen::MatrixXd y_old(dim_x, 1);
  y_old << 0.9, 1.9, 2.9;
  delay_step = max_delay_step - 1; // Index 4 (0-4)

  // Update Ground Truth
  ground_truth_update(x_ex_gt, P_ex_gt, y_old, C, R, delay_step, dim_x, dim_x, dim_x_ex);

  // Update TDKF
  EXPECT_TRUE(td_kf_.updateWithDelay(y_old, C, R, delay_step));

  // Check
  x_check = td_kf_.getLatestX();
  P_check = td_kf_.getLatestP();

  EXPECT_TRUE(x_check.isApprox(x_ex_gt.block(0,0,dim_x,1), epsilon));
  EXPECT_TRUE(P_check.isApprox(P_ex_gt.block(0,0,dim_x,dim_x), epsilon));
}
