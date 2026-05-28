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

#include "autoware/kalman_filter/kalman_filter.hpp"

#include <gtest/gtest.h>

using autoware::kalman_filter::KalmanFilter;

namespace
{
// Build a minimal, valid 2-state filter (x_ is 2x1, P_ is 2x2) for exercising guard branches.
KalmanFilter make_initialized_filter()
{
  Eigen::MatrixXd x(2, 1);
  x << 1.0, 2.0;
  Eigen::MatrixXd p(2, 2);
  p << 1.0, 0.0, 0.0, 1.0;
  KalmanFilter kf;
  kf.init(x, p);
  return kf;
}
}  // namespace

TEST(kalman_filter, kf)
{
  KalmanFilter kf_;

  Eigen::MatrixXd x_t(2, 1);
  x_t << 1, 2;

  Eigen::MatrixXd P_t(2, 2);
  P_t << 1, 0, 0, 1;

  Eigen::MatrixXd Q_t(2, 2);
  Q_t << 0.01, 0, 0, 0.01;

  Eigen::MatrixXd R_t(2, 2);
  R_t << 0.09, 0, 0, 0.09;

  Eigen::MatrixXd C_t(2, 2);
  C_t << 1, 0, 0, 1;

  Eigen::MatrixXd A_t(2, 2);
  A_t << 1, 0, 0, 1;

  Eigen::MatrixXd B_t(2, 2);
  B_t << 1, 0, 0, 1;

  // Initialize the filter and check if initialization was successful
  EXPECT_TRUE(kf_.init(x_t, A_t, B_t, C_t, Q_t, R_t, P_t));

  // Perform prediction
  Eigen::MatrixXd u_t(2, 1);
  u_t << 0.1, 0.1;
  EXPECT_TRUE(kf_.predict(u_t));

  // Check the updated state and covariance matrix
  Eigen::MatrixXd x_predict_expected = A_t * x_t + B_t * u_t;
  Eigen::MatrixXd P_predict_expected = A_t * P_t * A_t.transpose() + Q_t;

  Eigen::MatrixXd x_predict;
  kf_.getX(x_predict);
  Eigen::MatrixXd P_predict;
  kf_.getP(P_predict);

  EXPECT_NEAR(x_predict(0, 0), x_predict_expected(0, 0), 1e-5);
  EXPECT_NEAR(x_predict(1, 0), x_predict_expected(1, 0), 1e-5);
  EXPECT_NEAR(P_predict(0, 0), P_predict_expected(0, 0), 1e-5);
  EXPECT_NEAR(P_predict(1, 1), P_predict_expected(1, 1), 1e-5);

  // Perform update
  Eigen::MatrixXd y_t(2, 1);
  y_t << 1.05, 2.05;
  EXPECT_TRUE(kf_.update(y_t));

  // Check the updated state and covariance matrix
  const Eigen::MatrixXd PCT_t = P_predict_expected * C_t.transpose();
  const Eigen::MatrixXd K_t = PCT_t * ((R_t + C_t * PCT_t).inverse());
  const Eigen::MatrixXd y_pred = C_t * x_predict_expected;
  Eigen::MatrixXd x_update_expected = x_predict_expected + K_t * (y_t - y_pred);
  Eigen::MatrixXd P_update_expected = P_predict_expected - K_t * (C_t * P_predict_expected);

  Eigen::MatrixXd x_update;
  kf_.getX(x_update);
  Eigen::MatrixXd P_update;
  kf_.getP(P_update);

  EXPECT_NEAR(x_update(0, 0), x_update_expected(0, 0), 1e-5);
  EXPECT_NEAR(x_update(1, 0), x_update_expected(1, 0), 1e-5);
  EXPECT_NEAR(P_update(0, 0), P_update_expected(0, 0), 1e-5);
  EXPECT_NEAR(P_update(1, 1), P_update_expected(1, 1), 1e-5);

  // Add tests to cover missed lines

  KalmanFilter kf_new(x_t, A_t, B_t, C_t, Q_t, R_t, P_t);
  kf_new.init(x_t, P_t);
  kf_new.setA(A_t);
  kf_new.setB(B_t);
  kf_new.setC(C_t);
  kf_new.setQ(Q_t);
  kf_new.setR(R_t);

  EXPECT_TRUE(kf_new.predict(x_predict_expected, A_t));
  P_predict_expected = A_t * P_t * A_t.transpose() + Q_t;
  kf_new.getP(P_predict);
  EXPECT_NEAR(P_predict(0, 0), P_predict_expected(0, 0), 1e-5);
  EXPECT_NEAR(P_predict(1, 1), P_predict_expected(1, 1), 1e-5);
}

TEST(kalman_filter, init_full_rejects_empty_matrix)
{
  KalmanFilter kf;
  const Eigen::MatrixXd x = Eigen::MatrixXd::Zero(2, 1);
  const Eigen::MatrixXd m = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd empty;  // 0x0

  // Each empty argument independently makes the full init return false.
  EXPECT_FALSE(kf.init(empty, m, m, m, m, m, m));  // x empty
  EXPECT_FALSE(kf.init(x, empty, m, m, m, m, m));  // A empty
  EXPECT_FALSE(kf.init(x, m, empty, m, m, m, m));  // B empty
  EXPECT_FALSE(kf.init(x, m, m, empty, m, m, m));  // C empty
  EXPECT_FALSE(kf.init(x, m, m, m, empty, m, m));  // Q empty
  EXPECT_FALSE(kf.init(x, m, m, m, m, empty, m));  // R empty
  EXPECT_FALSE(kf.init(x, m, m, m, m, m, empty));  // P empty
  EXPECT_TRUE(kf.init(x, m, m, m, m, m, m));       // all valid
}

TEST(kalman_filter, init_state_cov_rejects_empty_matrix)
{
  KalmanFilter kf;
  const Eigen::MatrixXd x = Eigen::MatrixXd::Zero(2, 1);
  const Eigen::MatrixXd p = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd empty;

  EXPECT_FALSE(kf.init(empty, p));
  EXPECT_FALSE(kf.init(x, empty));
  EXPECT_TRUE(kf.init(x, p));
}

TEST(kalman_filter, init_state_cov_sets_state_and_getters)
{
  KalmanFilter kf;
  Eigen::MatrixXd x(2, 1);
  x << 3.0, -4.0;
  Eigen::MatrixXd p(2, 2);
  p << 2.0, 0.5, 0.5, 3.0;
  ASSERT_TRUE(kf.init(x, p));

  Eigen::MatrixXd x_out;
  kf.getX(x_out);
  Eigen::MatrixXd p_out;
  kf.getP(p_out);
  EXPECT_TRUE(x_out.isApprox(x));
  EXPECT_TRUE(p_out.isApprox(p));
  EXPECT_DOUBLE_EQ(kf.getXelement(0), 3.0);
  EXPECT_DOUBLE_EQ(kf.getXelement(1), -4.0);
}

TEST(kalman_filter, predict_with_x_and_a_rejects_dimension_mismatch)
{
  const Eigen::MatrixXd a2 = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd q2 = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd x_next = Eigen::MatrixXd::Zero(2, 1);

  {  // x_.rows() != x_next.rows()
    auto kf = make_initialized_filter();
    EXPECT_FALSE(kf.predict(Eigen::MatrixXd::Zero(3, 1), a2, q2));
  }
  {  // A.cols() != P_.rows()
    auto kf = make_initialized_filter();
    EXPECT_FALSE(kf.predict(x_next, Eigen::MatrixXd::Zero(2, 3), q2));
  }
  {  // Q.cols() != Q.rows()
    auto kf = make_initialized_filter();
    EXPECT_FALSE(kf.predict(x_next, a2, Eigen::MatrixXd::Zero(2, 3)));
  }
  {  // A.rows() != Q.cols()
    auto kf = make_initialized_filter();
    EXPECT_FALSE(kf.predict(x_next, Eigen::MatrixXd::Zero(3, 2), q2));
  }
}

TEST(kalman_filter, predict_with_input_rejects_dimension_mismatch)
{
  const Eigen::MatrixXd a2 = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd b2 = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd q2 = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd u2 = Eigen::MatrixXd::Zero(2, 1);

  {  // A.cols() != x_.rows()
    auto kf = make_initialized_filter();
    EXPECT_FALSE(kf.predict(u2, Eigen::MatrixXd::Zero(2, 3), b2, q2));
  }
  {  // B.cols() != u.rows()
    auto kf = make_initialized_filter();
    EXPECT_FALSE(kf.predict(u2, a2, Eigen::MatrixXd::Zero(2, 3), q2));
  }
}

TEST(kalman_filter, update_full_rejects_dimension_mismatch)
{
  const Eigen::MatrixXd y2 = Eigen::MatrixXd::Zero(2, 1);
  const Eigen::MatrixXd c2 = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd r2 = Eigen::MatrixXd::Identity(2, 2);

  {  // P_.cols() != C.cols()
    auto kf = make_initialized_filter();
    EXPECT_FALSE(kf.update(y2, y2, Eigen::MatrixXd::Identity(2, 3), r2));
  }
  {  // R not square
    auto kf = make_initialized_filter();
    EXPECT_FALSE(kf.update(y2, y2, c2, Eigen::MatrixXd::Zero(2, 3)));
  }
  {  // R.rows() != C.rows()
    auto kf = make_initialized_filter();
    EXPECT_FALSE(kf.update(y2, y2, c2, Eigen::MatrixXd::Identity(3, 3)));
  }
  {  // y.rows() != y_pred.rows()
    auto kf = make_initialized_filter();
    EXPECT_FALSE(kf.update(y2, Eigen::MatrixXd::Zero(3, 1), c2, r2));
  }
  {  // y.rows() != C.rows()
    auto kf = make_initialized_filter();
    const Eigen::MatrixXd y3 = Eigen::MatrixXd::Zero(3, 1);
    EXPECT_FALSE(kf.update(y3, y3, c2, r2));
  }
}

TEST(kalman_filter, update_rejects_observation_dimension_mismatch)
{
  auto kf = make_initialized_filter();
  const Eigen::MatrixXd y = Eigen::MatrixXd::Zero(2, 1);
  const Eigen::MatrixXd c_wrong = Eigen::MatrixXd::Identity(2, 3);  // C.cols() != x_.rows()
  const Eigen::MatrixXd r = Eigen::MatrixXd::Identity(2, 2);
  EXPECT_FALSE(kf.update(y, c_wrong, r));
}

TEST(kalman_filter, update_matches_closed_form_with_correlated_covariance)
{
  // Exercises the gain computation on a non-diagonal, well-conditioned problem and pins the
  // result to the closed-form (explicit-inverse) solution, guarding any change to the solver.
  KalmanFilter kf;
  Eigen::MatrixXd x(2, 1);
  x << 1.0, -2.0;
  Eigen::MatrixXd p(2, 2);
  p << 2.0, 0.3, 0.3, 1.5;  // symmetric positive-definite with off-diagonal correlation
  ASSERT_TRUE(kf.init(x, p));

  Eigen::MatrixXd c(1, 2);
  c << 1.0, 0.5;
  Eigen::MatrixXd r(1, 1);
  r << 0.2;
  Eigen::MatrixXd y(1, 1);
  y << 0.4;

  const Eigen::MatrixXd y_pred = c * x;
  const Eigen::MatrixXd pct = p * c.transpose();
  const Eigen::MatrixXd k = pct * (r + c * pct).inverse();
  const Eigen::MatrixXd x_expected = x + k * (y - y_pred);
  const Eigen::MatrixXd p_expected = p - k * (c * p);

  ASSERT_TRUE(kf.update(y, c, r));
  Eigen::MatrixXd x_out;
  kf.getX(x_out);
  Eigen::MatrixXd p_out;
  kf.getP(p_out);
  EXPECT_TRUE(x_out.isApprox(x_expected, 1e-10));
  EXPECT_TRUE(p_out.isApprox(p_expected, 1e-10));
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  bool result = RUN_ALL_TESTS();
  return result;
}
