// Copyright 2021 The Autoware Foundation
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

#include "autoware/osqp_interface/osqp_interface.hpp"
#include "gtest/gtest.h"

#include <Eigen/Core>

#include <iostream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace
{
// Problem taken from https://github.com/osqp/osqp/blob/master/tests/basic_qp/generate_problem.py
//
// min  1/2 * x' * P * x  + q' * x
// s.t. lb <= A * x <= ub
//
// P = [4, 1], q = [1], A = [1, 1], lb = [   1], ub = [1.0]
//     [1, 2]      [1]      [1, 0]       [   0]       [0.7]
//                          [0, 1]       [   0]       [0.7]
//                          [0, 1]       [-inf]       [inf]
//
// The optimal solution is
// x = [0.3, 0.7]'
// y = [-2.9, 0.0, 0.2, 0.0]`
// obj = 1.88

TEST(TestOsqpInterface, BasicQp)
{
  using autoware::osqp_interface::calCSCMatrix;
  using autoware::osqp_interface::calCSCMatrixTrapezoidal;
  using autoware::osqp_interface::CSC_Matrix;

  auto check_result = [](const autoware::osqp_interface::OSQPResult & result) {
    EXPECT_EQ(result.polish_status, 1);    // polish succeeded
    EXPECT_EQ(result.solution_status, 1);  // solution succeeded

    static const auto ep = 1.0e-8;

    const auto prime_val = result.primal_solution;
    ASSERT_EQ(prime_val.size(), size_t(2));
    EXPECT_NEAR(prime_val[0], 0.3, ep);
    EXPECT_NEAR(prime_val[1], 0.7, ep);

    const auto dual_val = result.lagrange_multipliers;
    ASSERT_EQ(dual_val.size(), size_t(4));
    EXPECT_NEAR(dual_val[0], -2.9, ep);
    EXPECT_NEAR(dual_val[1], 0.0, ep);
    EXPECT_NEAR(dual_val[2], 0.2, ep);
    EXPECT_NEAR(dual_val[3], 0.0, ep);
  };

  const Eigen::MatrixXd P = (Eigen::MatrixXd(2, 2) << 4, 1, 1, 2).finished();
  const Eigen::MatrixXd A = (Eigen::MatrixXd(4, 2) << 1, 1, 1, 0, 0, 1, 0, 1).finished();
  const std::vector<double> q = {1.0, 1.0};
  const std::vector<double> l = {1.0, 0.0, 0.0, -autoware::osqp_interface::INF};
  const std::vector<double> u = {1.0, 0.7, 0.7, autoware::osqp_interface::INF};

  {
    // Define problem during optimization
    autoware::osqp_interface::OSQPInterface osqp;
    autoware::osqp_interface::OSQPResult result = osqp.optimize(P, A, q, l, u);
    check_result(result);
  }

  {
    // Define problem during initialization
    autoware::osqp_interface::OSQPInterface osqp(P, A, q, l, u, 1e-6);
    autoware::osqp_interface::OSQPResult result = osqp.optimize();
    check_result(result);
  }

  {
    autoware::osqp_interface::OSQPResult result;
    // Dummy initial problem
    Eigen::MatrixXd P_ini = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd A_ini = Eigen::MatrixXd::Zero(4, 2);
    std::vector<double> q_ini(2, 0.0);
    std::vector<double> l_ini(4, 0.0);
    std::vector<double> u_ini(4, 0.0);
    autoware::osqp_interface::OSQPInterface osqp(P_ini, A_ini, q_ini, l_ini, u_ini, 1e-6);
    osqp.optimize();

    // Redefine problem before optimization
    osqp.initializeProblem(P, A, q, l, u);
    result = osqp.optimize();
    check_result(result);
  }

  {
    // Define problem during initialization with csc matrix
    CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
    CSC_Matrix A_csc = calCSCMatrix(A);
    autoware::osqp_interface::OSQPInterface osqp(P_csc, A_csc, q, l, u, 1e-6);
    autoware::osqp_interface::OSQPResult result = osqp.optimize();
    check_result(result);
  }

  {
    autoware::osqp_interface::OSQPResult result;
    // Dummy initial problem with csc matrix
    CSC_Matrix P_ini_csc = calCSCMatrixTrapezoidal(Eigen::MatrixXd::Zero(2, 2));
    CSC_Matrix A_ini_csc = calCSCMatrix(Eigen::MatrixXd::Zero(4, 2));
    std::vector<double> q_ini(2, 0.0);
    std::vector<double> l_ini(4, 0.0);
    std::vector<double> u_ini(4, 0.0);
    autoware::osqp_interface::OSQPInterface osqp(P_ini_csc, A_ini_csc, q_ini, l_ini, u_ini, 1e-6);
    osqp.optimize();

    // Redefine problem before optimization
    CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
    CSC_Matrix A_csc = calCSCMatrix(A);
    osqp.initializeProblem(P_csc, A_csc, q, l, u);
    result = osqp.optimize();
    check_result(result);
  }

  // add warm startup
  {
    autoware::osqp_interface::OSQPResult result;
    // Dummy initial problem with csc matrix
    CSC_Matrix P_ini_csc = calCSCMatrixTrapezoidal(Eigen::MatrixXd::Zero(2, 2));
    CSC_Matrix A_ini_csc = calCSCMatrix(Eigen::MatrixXd::Zero(4, 2));
    std::vector<double> q_ini(2, 0.0);
    std::vector<double> l_ini(4, 0.0);
    std::vector<double> u_ini(4, 0.0);
    autoware::osqp_interface::OSQPInterface osqp(P_ini_csc, A_ini_csc, q_ini, l_ini, u_ini, 1e-6);
    osqp.optimize();

    // Redefine problem before optimization
    CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
    CSC_Matrix A_csc = calCSCMatrix(A);
    osqp.initializeProblem(P_csc, A_csc, q, l, u);
    result = osqp.optimize();
    check_result(result);

    osqp.updateCheckTermination(1);
    const auto primal_val = result.primal_solution;
    const auto dual_val = result.lagrange_multipliers;
    for (size_t i = 0; i < primal_val.size(); ++i) {
      std::cerr << primal_val.at(i) << std::endl;
    }
    osqp.setWarmStart(primal_val, dual_val);
    result = osqp.optimize();
    check_result(result);
    EXPECT_EQ(osqp.getTakenIter(), 1);
  }

  // update settings
  {
    autoware::osqp_interface::OSQPInterface osqp(P, A, q, l, u, 1e-6);

    osqp.updateP(P);
    osqp.updateCscP(calCSCMatrixTrapezoidal(P));
    osqp.updateA(A);
    osqp.updateCscA(calCSCMatrix(A));
    osqp.updateQ(q);
    osqp.updateL(l);
    osqp.updateU(u);
    osqp.updateBounds(l, u);
    osqp.updateEpsAbs(1e-6);
    osqp.updateEpsRel(1e-6);
    osqp.updateMaxIter(1000);
    osqp.updateVerbose(true);
    osqp.updateRhoInterval(10);
    osqp.updateRho(0.1);
    osqp.updateAlpha(1.6);
    osqp.updateScaling(10);
    osqp.updatePolish(true);
    osqp.updatePolishRefinementIteration(10);
    osqp.updateCheckTermination(1);

    EXPECT_NO_THROW(osqp.optimize());
  }

  // get status
  {
    autoware::osqp_interface::OSQPInterface osqp(P, A, q, l, u, 1e-6);
    osqp.optimize();
    osqp.updatePolish(true);
    EXPECT_EQ(osqp.getStatus(), 1);
    EXPECT_EQ(osqp.getStatusMessage(), "solved");
  }
}

// Each of the five dimension-consistency checks in initializeProblem(Eigen, ...) must throw
// std::invalid_argument with a descriptive message. The happy path is already covered by
// TestOsqpInterface.BasicQp; here we pin the individual failure branches.
TEST(TestOsqpInterface, InitializeProblemDimensionMismatch)
{
  using autoware::osqp_interface::OSQPInterface;

  // Consistent baseline: P is 2x2, A is 2x2, q/l/u sized accordingly.
  const Eigen::MatrixXd P = (Eigen::MatrixXd(2, 2) << 4, 1, 1, 2).finished();
  const Eigen::MatrixXd A = (Eigen::MatrixXd(2, 2) << 1, 1, 1, 0).finished();
  const std::vector<double> q = {1.0, 1.0};
  const std::vector<double> l = {0.0, 0.0};
  const std::vector<double> u = {1.0, 1.0};

  OSQPInterface osqp;

  // 1. P not square.
  {
    const Eigen::MatrixXd p_non_square = Eigen::MatrixXd::Zero(2, 3);
    EXPECT_THROW(osqp.initializeProblem(p_non_square, A, q, l, u), std::invalid_argument);
  }
  // 2. P.rows() != q.size().
  {
    const std::vector<double> q_bad = {1.0, 1.0, 1.0};
    EXPECT_THROW(osqp.initializeProblem(P, A, q_bad, l, u), std::invalid_argument);
  }
  // 3. P.rows() != A.cols().
  {
    const Eigen::MatrixXd a_bad = Eigen::MatrixXd::Zero(2, 3);
    EXPECT_THROW(osqp.initializeProblem(P, a_bad, q, l, u), std::invalid_argument);
  }
  // 4. A.rows() != l.size().
  {
    const std::vector<double> l_bad = {0.0, 0.0, 0.0};
    EXPECT_THROW(osqp.initializeProblem(P, A, q, l_bad, u), std::invalid_argument);
  }
  // 5. A.rows() != u.size().
  {
    const std::vector<double> u_bad = {1.0, 1.0, 1.0};
    EXPECT_THROW(osqp.initializeProblem(P, A, q, l, u_bad), std::invalid_argument);
  }

  // Sanity: the baseline (all consistent) does not throw.
  EXPECT_NO_THROW(osqp.initializeProblem(P, A, q, l, u));
}

// Warm-start setters must return false (and not crash) when called before the workspace has been
// initialized, i.e. on a default-constructed interface.
TEST(TestOsqpInterface, WarmStartBeforeInitializationFails)
{
  using autoware::osqp_interface::OSQPInterface;

  OSQPInterface osqp;  // no problem set up yet -> m_work_initialized == false
  const std::vector<double> primal = {0.0, 0.0};
  const std::vector<double> dual = {0.0, 0.0};

  EXPECT_FALSE(osqp.setPrimalVariables(primal));
  EXPECT_FALSE(osqp.setDualVariables(dual));
  EXPECT_FALSE(osqp.setWarmStart(primal, dual));
}

// logUnsolvedStatus after a solved problem must be a
// no-op (status == 1) and must never throw.
TEST(TestOsqpInterface, LogUnsolvedStatusSolvedIsNoThrow)
{
  using autoware::osqp_interface::OSQPInterface;

  const Eigen::MatrixXd P = (Eigen::MatrixXd(2, 2) << 4, 1, 1, 2).finished();
  const Eigen::MatrixXd A = (Eigen::MatrixXd(4, 2) << 1, 1, 1, 0, 0, 1, 0, 1).finished();
  const std::vector<double> q = {1.0, 1.0};
  const std::vector<double> l = {1.0, 0.0, 0.0, -autoware::osqp_interface::INF};
  const std::vector<double> u = {1.0, 0.7, 0.7, autoware::osqp_interface::INF};

  OSQPInterface osqp(P, A, q, l, u, 1e-6);
  osqp.optimize();
  ASSERT_EQ(osqp.getStatus(), 1);  // solved
  EXPECT_NO_THROW(osqp.logUnsolvedStatus());
  EXPECT_NO_THROW(osqp.logUnsolvedStatus("prefix"));
}
}  // namespace
