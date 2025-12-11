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

using autoware::kalman_filter::TimeDelayKalmanFilter;

namespace
{
// Test constants
constexpr int kDimX = 3;
constexpr int kMaxDelayStep = 5;
constexpr int kDimXEx = kDimX * kMaxDelayStep;
constexpr double kEpsilon = 1e-5;
constexpr double kInitialCovariance = 0.1;
constexpr double kProcessNoise = 0.01;
constexpr double kMeasurementNoise = 0.001;
constexpr double kStateTransitionScale = 2.0;
constexpr double kObservationScale = 0.5;
}  // namespace

// Helper function to shift the extended state for ground truth verification
void ground_truth_predict(
  Eigen::MatrixXd & x_ex, Eigen::MatrixXd & P_ex, const Eigen::MatrixXd & x_next,
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q, int dim_x, int dim_x_ex)
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
  P_tmp.block(0, 0, dim_x, dim_x) = A * P_ex.block(0, 0, dim_x, dim_x) * A.transpose() + Q;

  // Top-Right: Correlation between new state and past states
  P_tmp.block(0, dim_x, dim_x, dim_x_ex - dim_x) = A * P_ex.block(0, 0, dim_x, dim_x_ex - dim_x);

  // Bottom-Left: Transpose of Top-Right
  P_tmp.block(dim_x, 0, dim_x_ex - dim_x, dim_x) =
    P_ex.block(0, 0, dim_x_ex - dim_x, dim_x) * A.transpose();

  // Bottom-Right: Shifted previous covariance
  P_tmp.block(dim_x, dim_x, dim_x_ex - dim_x, dim_x_ex - dim_x) =
    P_ex.block(0, 0, dim_x_ex - dim_x, dim_x_ex - dim_x);

  P_ex = P_tmp;
}

void ground_truth_update(
  Eigen::MatrixXd & x_ex, Eigen::MatrixXd & P_ex, const Eigen::MatrixXd & y,
  const Eigen::MatrixXd & C, const Eigen::MatrixXd & R, int delay_step, int dim_x, int dim_y,
  int dim_x_ex)
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

class TimeDelayKalmanFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize state vector
    x_t_.resize(kDimX, 1);
    x_t_ << 1.0, 2.0, 3.0;

    // Initialize covariance matrix
    P_t_ = Eigen::MatrixXd::Identity(kDimX, kDimX) * kInitialCovariance;

    // Initialize ground truth extended state
    x_ex_gt_ = Eigen::MatrixXd::Zero(kDimXEx, 1);
    P_ex_gt_ = Eigen::MatrixXd::Zero(kDimXEx, kDimXEx);

    // Fill ground truth with initial state repeated (assuming steady state init)
    for (int i = 0; i < kMaxDelayStep; ++i) {
      x_ex_gt_.block(i * kDimX, 0, kDimX, 1) = x_t_;
      P_ex_gt_.block(i * kDimX, i * kDimX, kDimX, kDimX) = P_t_;
    }

    // Setup model matrices
    A_ = Eigen::MatrixXd::Identity(kDimX, kDimX) * kStateTransitionScale;
    Q_ = Eigen::MatrixXd::Identity(kDimX, kDimX) * kProcessNoise;
    C_ = Eigen::MatrixXd::Identity(kDimX, kDimX) * kObservationScale;
    R_ = Eigen::MatrixXd::Identity(kDimX, kDimX) * kMeasurementNoise;

    // Setup predicted next state
    x_next_.resize(kDimX, 1);
    x_next_ << 2.0, 4.0, 6.0;
  }

  TimeDelayKalmanFilter td_kf_;
  Eigen::MatrixXd x_t_;
  Eigen::MatrixXd P_t_;
  Eigen::MatrixXd x_ex_gt_;
  Eigen::MatrixXd P_ex_gt_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd C_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd x_next_;
};

TEST_F(TimeDelayKalmanFilterTest, Initialization)
{
  td_kf_.init(x_t_, P_t_, kMaxDelayStep);

  const Eigen::MatrixXd x_check = td_kf_.getLatestX();
  const Eigen::MatrixXd P_check = td_kf_.getLatestP();

  EXPECT_TRUE(x_check.isApprox(x_t_, kEpsilon));
  EXPECT_TRUE(P_check.isApprox(P_t_, kEpsilon));
}

TEST_F(TimeDelayKalmanFilterTest, Prediction)
{
  td_kf_.init(x_t_, P_t_, kMaxDelayStep);

  // Update Ground Truth
  ground_truth_predict(x_ex_gt_, P_ex_gt_, x_next_, A_, Q_, kDimX, kDimXEx);

  // Update TDKF
  ASSERT_TRUE(td_kf_.predictWithDelay(x_next_, A_, Q_));

  // Check
  const Eigen::MatrixXd x_check = td_kf_.getLatestX();
  const Eigen::MatrixXd P_check = td_kf_.getLatestP();

  EXPECT_TRUE(x_check.isApprox(x_ex_gt_.block(0, 0, kDimX, 1), kEpsilon));
  EXPECT_TRUE(P_check.isApprox(P_ex_gt_.block(0, 0, kDimX, kDimX), kEpsilon));
}

TEST_F(TimeDelayKalmanFilterTest, UpdateWithDelay)
{
  td_kf_.init(x_t_, P_t_, kMaxDelayStep);

  // First predict
  ground_truth_predict(x_ex_gt_, P_ex_gt_, x_next_, A_, Q_, kDimX, kDimXEx);
  ASSERT_TRUE(td_kf_.predictWithDelay(x_next_, A_, Q_));

  // Update with delay step = 2
  Eigen::MatrixXd y_delayed(kDimX, 1);
  y_delayed << 1.05, 2.05, 3.05;
  constexpr int delay_step = 2;

  // Update Ground Truth
  ground_truth_update(x_ex_gt_, P_ex_gt_, y_delayed, C_, R_, delay_step, kDimX, kDimX, kDimXEx);

  // Update TDKF
  ASSERT_TRUE(td_kf_.updateWithDelay(y_delayed, C_, R_, delay_step));

  // Check
  const Eigen::MatrixXd x_check = td_kf_.getLatestX();
  const Eigen::MatrixXd P_check = td_kf_.getLatestP();

  EXPECT_TRUE(x_check.isApprox(x_ex_gt_.block(0, 0, kDimX, 1), kEpsilon));
  EXPECT_TRUE(P_check.isApprox(P_ex_gt_.block(0, 0, kDimX, kDimX), kEpsilon));
}

TEST_F(TimeDelayKalmanFilterTest, UpdateWithZeroDelay)
{
  // Validates that the TDKF behaves like a standard KF when delay is 0
  td_kf_.init(x_t_, P_t_, kMaxDelayStep);

  // First predict
  ground_truth_predict(x_ex_gt_, P_ex_gt_, x_next_, A_, Q_, kDimX, kDimXEx);
  ASSERT_TRUE(td_kf_.predictWithDelay(x_next_, A_, Q_));

  // Update with zero delay
  Eigen::MatrixXd y_current(kDimX, 1);
  y_current << 2.1, 4.1, 6.1;  // Close to current state x_next
  constexpr int delay_step = 0;

  // Update Ground Truth
  ground_truth_update(x_ex_gt_, P_ex_gt_, y_current, C_, R_, delay_step, kDimX, kDimX, kDimXEx);

  // Update TDKF
  ASSERT_TRUE(td_kf_.updateWithDelay(y_current, C_, R_, delay_step));

  // Check
  const Eigen::MatrixXd x_check = td_kf_.getLatestX();
  const Eigen::MatrixXd P_check = td_kf_.getLatestP();

  EXPECT_TRUE(x_check.isApprox(x_ex_gt_.block(0, 0, kDimX, 1), kEpsilon));
  EXPECT_TRUE(P_check.isApprox(P_ex_gt_.block(0, 0, kDimX, kDimX), kEpsilon));
}

TEST_F(TimeDelayKalmanFilterTest, UpdateWithMaxDelay)
{
  // Validates updating the tail of the buffer
  td_kf_.init(x_t_, P_t_, kMaxDelayStep);

  // First predict
  ground_truth_predict(x_ex_gt_, P_ex_gt_, x_next_, A_, Q_, kDimX, kDimXEx);
  ASSERT_TRUE(td_kf_.predictWithDelay(x_next_, A_, Q_));

  // Update with max delay
  Eigen::MatrixXd y_old(kDimX, 1);
  y_old << 0.9, 1.9, 2.9;
  const int delay_step = kMaxDelayStep - 1;  // Index 4 (0-4)

  // Update Ground Truth
  ground_truth_update(x_ex_gt_, P_ex_gt_, y_old, C_, R_, delay_step, kDimX, kDimX, kDimXEx);

  // Update TDKF
  ASSERT_TRUE(td_kf_.updateWithDelay(y_old, C_, R_, delay_step));

  // Check
  const Eigen::MatrixXd x_check = td_kf_.getLatestX();
  const Eigen::MatrixXd P_check = td_kf_.getLatestP();

  EXPECT_TRUE(x_check.isApprox(x_ex_gt_.block(0, 0, kDimX, 1), kEpsilon));
  EXPECT_TRUE(P_check.isApprox(P_ex_gt_.block(0, 0, kDimX, kDimX), kEpsilon));
}

TEST_F(TimeDelayKalmanFilterTest, UpdateWithInvalidDelayStepExceedsMax)
{
  td_kf_.init(x_t_, P_t_, kMaxDelayStep);

  Eigen::MatrixXd y(kDimX, 1);
  y << 1.0, 2.0, 3.0;
  const int invalid_delay_step = kMaxDelayStep;  // Should be max_delay_step - 1 at most

  EXPECT_FALSE(td_kf_.updateWithDelay(y, C_, R_, invalid_delay_step));
}

TEST_F(TimeDelayKalmanFilterTest, UpdateWithNegativeDelayStep)
{
  td_kf_.init(x_t_, P_t_, kMaxDelayStep);

  Eigen::MatrixXd y(kDimX, 1);
  y << 1.0, 2.0, 3.0;
  constexpr int negative_delay_step = -1;

  EXPECT_FALSE(td_kf_.updateWithDelay(y, C_, R_, negative_delay_step));
}

TEST_F(TimeDelayKalmanFilterTest, MultiplePredictionsBeforeUpdate)
{
  td_kf_.init(x_t_, P_t_, kMaxDelayStep);

  // Perform multiple predictions
  for (int i = 0; i < 3; ++i) {
    Eigen::MatrixXd x_pred(kDimX, 1);
    x_pred << 2.0 * (i + 1), 4.0 * (i + 1), 6.0 * (i + 1);

    ground_truth_predict(x_ex_gt_, P_ex_gt_, x_pred, A_, Q_, kDimX, kDimXEx);
    ASSERT_TRUE(td_kf_.predictWithDelay(x_pred, A_, Q_));
  }

  // Now update
  Eigen::MatrixXd y(kDimX, 1);
  y << 1.0, 2.0, 3.0;
  constexpr int delay_step = 2;

  ground_truth_update(x_ex_gt_, P_ex_gt_, y, C_, R_, delay_step, kDimX, kDimX, kDimXEx);
  ASSERT_TRUE(td_kf_.updateWithDelay(y, C_, R_, delay_step));

  const Eigen::MatrixXd x_check = td_kf_.getLatestX();
  const Eigen::MatrixXd P_check = td_kf_.getLatestP();

  EXPECT_TRUE(x_check.isApprox(x_ex_gt_.block(0, 0, kDimX, 1), kEpsilon));
  EXPECT_TRUE(P_check.isApprox(P_ex_gt_.block(0, 0, kDimX, kDimX), kEpsilon));
}

TEST_F(TimeDelayKalmanFilterTest, ZeroInitialState)
{
  Eigen::MatrixXd x_zero = Eigen::MatrixXd::Zero(kDimX, 1);
  Eigen::MatrixXd P_zero = Eigen::MatrixXd::Identity(kDimX, kDimX) * kInitialCovariance;

  td_kf_.init(x_zero, P_zero, kMaxDelayStep);

  const Eigen::MatrixXd x_check = td_kf_.getLatestX();
  const Eigen::MatrixXd P_check = td_kf_.getLatestP();

  EXPECT_TRUE(x_check.isApprox(x_zero, kEpsilon));
  EXPECT_TRUE(P_check.isApprox(P_zero, kEpsilon));
}
