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
#include "gtest/gtest.h"
#include "osqp_interface_status.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace
{
// Unit tests for the pure status-mapping helper. These pin the mapping from the
// raw OSQP status codes to their canonical string names independently of any
// solver run.
TEST(TestOsqpInterfaceStatus, StatusToString)
{
  using autoware::qp_interface::status_to_string;

  EXPECT_EQ(status_to_string(OSQP_SOLVED), "OSQP_SOLVED");
  EXPECT_EQ(status_to_string(OSQP_SOLVED_INACCURATE), "OSQP_SOLVED_INACCURATE");
  EXPECT_EQ(status_to_string(OSQP_MAX_ITER_REACHED), "OSQP_MAX_ITER_REACHED");
  EXPECT_EQ(status_to_string(OSQP_PRIMAL_INFEASIBLE), "OSQP_PRIMAL_INFEASIBLE");
  EXPECT_EQ(
    status_to_string(OSQP_PRIMAL_INFEASIBLE_INACCURATE), "OSQP_PRIMAL_INFEASIBLE_INACCURATE");
  EXPECT_EQ(status_to_string(OSQP_DUAL_INFEASIBLE), "OSQP_DUAL_INFEASIBLE");
  EXPECT_EQ(status_to_string(OSQP_DUAL_INFEASIBLE_INACCURATE), "OSQP_DUAL_INFEASIBLE_INACCURATE");
  EXPECT_EQ(status_to_string(OSQP_SIGINT), "OSQP_SIGINT");
  EXPECT_EQ(status_to_string(OSQP_NON_CVX), "OSQP_NON_CVX");
  EXPECT_EQ(status_to_string(OSQP_UNSOLVED), "OSQP_UNSOLVED");
#ifdef OSQP_TIME_LIMIT_REACHED
  EXPECT_EQ(status_to_string(OSQP_TIME_LIMIT_REACHED), "OSQP_TIME_LIMIT_REACHED");
#endif

  // Unknown / unmapped value falls back to a sentinel rather than mis-reporting
  // a solved status.
  EXPECT_EQ(status_to_string(static_cast<c_int>(12345)), "OSQP_UNKNOWN");
}

// Coverage: a primal-infeasible problem must NOT report solved. With the old
// hardcoded getStatus() this fails because it always returned "OSQP_SOLVED".
TEST(TestOsqpInterfaceStatus, InfeasibleProblemReportsNotSolved)
{
  using autoware::qp_interface::calCSCMatrix;
  using autoware::qp_interface::calCSCMatrixTrapezoidal;
  using autoware::qp_interface::CSC_Matrix;

  // min 1/2 x'Px + q'x  s.t.  l <= A x <= u, with contradictory bounds:
  //   x >= 1  and  x <= 0  ->  primal infeasible.
  const Eigen::MatrixXd P = (Eigen::MatrixXd(1, 1) << 1.0).finished();
  const Eigen::MatrixXd A = (Eigen::MatrixXd(2, 1) << 1.0, 1.0).finished();
  const std::vector<double> q = {0.0};
  const std::vector<double> l = {1.0, -autoware::qp_interface::OSQP_INF};
  const std::vector<double> u = {autoware::qp_interface::OSQP_INF, 0.0};

  autoware::qp_interface::OSQPInterface osqp(false, 4000, 1e-6);
  osqp.optimize(calCSCMatrixTrapezoidal(P), calCSCMatrix(A), q, l, u);

  EXPECT_FALSE(osqp.isSolved());
  const auto status = osqp.getStatus();
  EXPECT_NE(status, "OSQP_SOLVED");
}

// A freshly-constructed interface (no optimize() call yet) must report a
// deterministic "not solved" state. latest_work_info_ is value-initialized and
// its status_val is set to OSQP_UNSOLVED in the constructor, so this does not
// read uninitialized memory.
TEST(TestOsqpInterfaceStatus, FreshlyConstructedIsNotSolved)
{
  // This constructor only sets up settings; it does not run the solver.
  autoware::qp_interface::OSQPInterface osqp(false, 4000, 1e-6);

  EXPECT_FALSE(osqp.isSolved());
  EXPECT_EQ(osqp.getStatus(), "OSQP_UNSOLVED");
}
}  // namespace
