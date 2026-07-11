// Copyright 2023 TIER IV, Inc.
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

#include "autoware/qp_interface/osqp_interface.hpp"
#include "autoware/qp_interface/qp_interface.hpp"

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace autoware::qp_interface
{
TEST(QPInterfaceTest, InitializeProblem_NonSquareP_ThrowsException)
{
  Eigen::MatrixXd P(2, 3);
  Eigen::MatrixXd A(1, 2);
  std::vector<double> q = {1.0, 2.0};
  std::vector<double> l = {1.0};
  std::vector<double> u = {1.0};
  bool enable_warm_start = false;
  OSQPFloat eps_abs = 1e-4;

  EXPECT_THROW(
    { OSQPInterface osqp_instance(P, A, q, l, u, enable_warm_start, eps_abs); },
    std::invalid_argument);
}

TEST(QPInterfaceTest, InitializeProblem_PRowsNotEqualQSize_ThrowsException)
{
  Eigen::MatrixXd P(2, 2);
  Eigen::MatrixXd A(1, 2);
  std::vector<double> q = {1.0};
  std::vector<double> l = {1.0};
  std::vector<double> u = {1.0};
  bool enable_warm_start = false;
  OSQPFloat eps_abs = 1e-4;

  EXPECT_THROW(
    { OSQPInterface osqp_instance(P, A, q, l, u, enable_warm_start, eps_abs); },
    std::invalid_argument);
}

TEST(QPInterfaceTest, InitializeProblem_PRowsNotEqualACols_ThrowsException)
{
  Eigen::MatrixXd P(2, 2);
  Eigen::MatrixXd A(1, 3);
  std::vector<double> q = {1.0, 2.0};
  std::vector<double> l = {1.0};
  std::vector<double> u = {1.0};
  bool enable_warm_start = false;
  OSQPFloat eps_abs = 1e-4;

  EXPECT_THROW(
    { OSQPInterface osqp_instance(P, A, q, l, u, enable_warm_start, eps_abs); },
    std::invalid_argument);
}

TEST(QPInterfaceTest, InitializeProblem_ARowsNotEqualLSize_ThrowsException)
{
  Eigen::MatrixXd P(2, 2);
  Eigen::MatrixXd A(2, 2);
  std::vector<double> q = {1.0, 2.0};
  std::vector<double> l = {1.0};
  std::vector<double> u = {1.0, 2.0};
  bool enable_warm_start = false;
  OSQPFloat eps_abs = 1e-4;

  EXPECT_THROW(
    { OSQPInterface osqp_instance(P, A, q, l, u, enable_warm_start, eps_abs); },
    std::invalid_argument);
}

TEST(QPInterfaceTest, InitializeProblem_ARowsNotEqualUSize_ThrowsException)
{
  Eigen::MatrixXd P(2, 2);
  Eigen::MatrixXd A(2, 2);
  std::vector<double> q = {1.0, 2.0};
  std::vector<double> l = {1.0, 2.0};
  std::vector<double> u = {1.0};
  bool enable_warm_start = false;
  OSQPFloat eps_abs = 1e-4;

  EXPECT_THROW(
    { OSQPInterface osqp_instance(P, A, q, l, u, enable_warm_start, eps_abs); },
    std::invalid_argument);
}

TEST(QPInterfaceTest, InitializeProblem_ValidInputs_Success)
{
  Eigen::MatrixXd P(2, 2);
  P << 1, 0, 0, 1;
  Eigen::MatrixXd A(1, 2);
  A << 1, 1;
  std::vector<double> q = {1.0, 2.0};
  std::vector<double> l = {1.0};
  std::vector<double> u = {2.0};
  bool enable_warm_start = false;
  OSQPFloat eps_abs = 1e-4;

  OSQPInterface osqp_instance(P, A, q, l, u, enable_warm_start, eps_abs);
  EXPECT_NO_THROW({ OSQPInterface osqp_instance(P, A, q, l, u, enable_warm_start, eps_abs); });
}

TEST(QPInterfaceTest, Optimize_ValidInputs_ReturnsResult)
{
  Eigen::MatrixXd P(2, 2);
  P << 1, 0, 0, 1;
  Eigen::MatrixXd A(1, 2);
  A << 1, 1;
  std::vector<double> q = {1.0, 2.0};
  std::vector<double> l = {1.0};
  std::vector<double> u = {1.0};
  bool enable_warm_start = false;
  OSQPFloat eps_abs = 1e-4;

  OSQPInterface osqp(P, A, q, l, u, enable_warm_start, eps_abs);
  std::vector<double> result = osqp.QPInterface::optimize(P, A, q, l, u);
  EXPECT_EQ(result.size(), 2);
}

}  // namespace autoware::qp_interface
