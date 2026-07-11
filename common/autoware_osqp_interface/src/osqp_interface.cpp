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

#include "autoware/osqp_interface/csc_matrix_conv.hpp"
#include "osqp/osqp.h"

#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace autoware::osqp_interface
{
OSQPInterface::OSQPInterface(const OSQPFloat eps_abs, const bool polish)
: m_solver{nullptr, OSQPSolverDeleter}
{
  m_settings = std::make_unique<OSQPSettings>();

  if (m_settings) {
    osqp_set_default_settings(m_settings.get());
    m_settings->alpha = 1.6;  // Change alpha parameter
    m_settings->eps_rel = 1.0E-4;
    m_settings->eps_abs = eps_abs;
    m_settings->eps_prim_inf = 1.0E-4;
    m_settings->eps_dual_inf = 1.0E-4;
    m_settings->warm_starting = true;
    m_settings->max_iter = 4000;
    m_settings->verbose = false;
    m_settings->polishing = polish;
  }
  m_exitflag = 0;
}

OSQPInterface::OSQPInterface(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u, const OSQPFloat eps_abs)
: OSQPInterface(eps_abs)
{
  initializeProblem(P, A, q, l, u);
}

OSQPInterface::OSQPInterface(
  const CSC_Matrix & P, const CSC_Matrix & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u, const OSQPFloat eps_abs)
: OSQPInterface(eps_abs)
{
  initializeProblem(P, A, q, l, u);
}

OSQPInterface::~OSQPInterface() = default;

void OSQPInterface::OSQPSolverDeleter(OSQPSolver * ptr) noexcept
{
  if (ptr != nullptr) {
    osqp_cleanup(ptr);
  }
}

void OSQPInterface::updateP(const Eigen::MatrixXd & P_new)
{
  /*
  // Transform 'P' into an 'upper trapezoidal matrix'
  Eigen::MatrixXd P_trap = P_new.triangularView<Eigen::Upper>();
  // Transform 'P' into a sparse matrix and extract data as dynamic arrays
  Eigen::SparseMatrix<double> P_sparse = P_trap.sparseView();
  double *P_val_ptr = P_sparse.valuePtr();
  // Convert dynamic 'int' arrays to 'c_int' arrays (OSQP input type)
  c_int P_elem_N = P_sparse.nonZeros();
  */
  CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P_new);
  osqp_update_data_mat(
    m_solver.get(), P_csc.m_vals.data(), OSQP_NULL, static_cast<OSQPInt>(P_csc.m_vals.size()),
    OSQP_NULL, OSQP_NULL, 0);
}

void OSQPInterface::updateCscP(const CSC_Matrix & P_csc)
{
  osqp_update_data_mat(
    m_solver.get(), P_csc.m_vals.data(), OSQP_NULL, static_cast<OSQPInt>(P_csc.m_vals.size()),
    OSQP_NULL, OSQP_NULL, 0);
}

void OSQPInterface::updateA(const Eigen::MatrixXd & A_new)
{
  /*
  // Transform 'A' into a sparse matrix and extract data as dynamic arrays
  Eigen::SparseMatrix<double> A_sparse = A_new.sparseView();
  double *A_val_ptr = A_sparse.valuePtr();
  // Convert dynamic 'int' arrays to 'c_int' arrays (OSQP input type)
  c_int A_elem_N = A_sparse.nonZeros();
  */
  CSC_Matrix A_csc = calCSCMatrix(A_new);
  osqp_update_data_mat(
    m_solver.get(), OSQP_NULL, OSQP_NULL, 0, A_csc.m_vals.data(), OSQP_NULL,
    static_cast<OSQPInt>(A_csc.m_vals.size()));
  return;
}

void OSQPInterface::updateCscA(const CSC_Matrix & A_csc)
{
  osqp_update_data_mat(
    m_solver.get(), OSQP_NULL, OSQP_NULL, 0, A_csc.m_vals.data(), OSQP_NULL,
    static_cast<OSQPInt>(A_csc.m_vals.size()));
}

void OSQPInterface::updateQ(const std::vector<double> & q_new)
{
  std::vector<OSQPFloat> q_tmp(q_new.begin(), q_new.end());
  osqp_update_data_vec(m_solver.get(), q_tmp.data(), OSQP_NULL, OSQP_NULL);
}

void OSQPInterface::updateL(const std::vector<double> & l_new)
{
  std::vector<OSQPFloat> l_tmp(l_new.begin(), l_new.end());
  osqp_update_data_vec(m_solver.get(), OSQP_NULL, l_tmp.data(), OSQP_NULL);
}

void OSQPInterface::updateU(const std::vector<double> & u_new)
{
  std::vector<OSQPFloat> u_tmp(u_new.begin(), u_new.end());
  osqp_update_data_vec(m_solver.get(), OSQP_NULL, OSQP_NULL, u_tmp.data());
}

void OSQPInterface::updateBounds(
  const std::vector<double> & l_new, const std::vector<double> & u_new)
{
  std::vector<OSQPFloat> l_tmp(l_new.begin(), l_new.end());
  std::vector<OSQPFloat> u_tmp(u_new.begin(), u_new.end());
  osqp_update_data_vec(m_solver.get(), OSQP_NULL, l_tmp.data(), u_tmp.data());
}

void OSQPInterface::updateSettings()
{
  // In OSQP 1.0.0 the per-field update functions were removed. Settings are updated by pushing the
  // whole settings object to the solver. Note that osqp_update_settings ignores scaling, sigma and
  // the adaptive_rho_* settings (those can only be set at osqp_setup time), and rho must be updated
  // via osqp_update_rho.
  if (m_work_initialized) {
    osqp_update_settings(m_solver.get(), m_settings.get());  // for current work
  }
}

void OSQPInterface::updateEpsAbs(const double eps_abs)
{
  m_settings->eps_abs = eps_abs;  // for default setting
  updateSettings();               // for current work
}

void OSQPInterface::updateEpsRel(const double eps_rel)
{
  m_settings->eps_rel = eps_rel;  // for default setting
  updateSettings();               // for current work
}

void OSQPInterface::updateMaxIter(const int max_iter)
{
  m_settings->max_iter = max_iter;  // for default setting
  updateSettings();                 // for current work
}

void OSQPInterface::updateVerbose(const bool is_verbose)
{
  m_settings->verbose = is_verbose;  // for default setting
  updateSettings();                  // for current work
}

void OSQPInterface::updateRhoInterval(const int rho_interval)
{
  m_settings->adaptive_rho_interval = rho_interval;  // for default setting
}

void OSQPInterface::updateRho(const double rho)
{
  m_settings->rho = rho;
  if (m_work_initialized) {
    osqp_update_rho(m_solver.get(), rho);
  }
}

void OSQPInterface::updateAlpha(const double alpha)
{
  m_settings->alpha = alpha;  // for default setting
  updateSettings();           // for current work
}

void OSQPInterface::updateScaling(const int scaling)
{
  m_settings->scaling = scaling;
}

void OSQPInterface::updatePolish(const bool polish)
{
  m_settings->polishing = polish;  // for default setting
  updateSettings();                // for current work
}

void OSQPInterface::updatePolishRefinementIteration(const int polish_refine_iter)
{
  if (polish_refine_iter < 0) {
    std::cerr << "Polish refinement iterations must be positive" << std::endl;
    return;
  }

  m_settings->polish_refine_iter = polish_refine_iter;  // for default setting
  updateSettings();                                     // for current work
}

void OSQPInterface::updateCheckTermination(const int check_termination)
{
  if (check_termination < 0) {
    std::cerr << "Check termination must be positive" << std::endl;
    return;
  }

  m_settings->check_termination = check_termination;  // for default setting
  updateSettings();                                   // for current work
}

bool OSQPInterface::setWarmStart(
  const std::vector<double> & primal_variables, const std::vector<double> & dual_variables)
{
  return setPrimalVariables(primal_variables) && setDualVariables(dual_variables);
}

bool OSQPInterface::setPrimalVariables(const std::vector<double> & primal_variables)
{
  if (!m_work_initialized) {
    return false;
  }

  const auto result = osqp_warm_start(m_solver.get(), primal_variables.data(), OSQP_NULL);
  if (result != 0) {
    std::cerr << "Failed to set primal variables for warm start" << std::endl;
    return false;
  }

  return true;
}

bool OSQPInterface::setDualVariables(const std::vector<double> & dual_variables)
{
  if (!m_work_initialized) {
    return false;
  }

  const auto result = osqp_warm_start(m_solver.get(), OSQP_NULL, dual_variables.data());
  if (result != 0) {
    std::cerr << "Failed to set dual variables for warm start" << std::endl;
    return false;
  }

  return true;
}

int64_t OSQPInterface::initializeProblem(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u)
{
  // check if arguments are valid
  std::stringstream ss;
  if (P.rows() != P.cols()) {
    ss << "P.rows() and P.cols() are not the same. P.rows() = " << P.rows()
       << ", P.cols() = " << P.cols();
    throw std::invalid_argument(ss.str());
  }
  if (P.rows() != static_cast<int>(q.size())) {
    ss << "P.rows() and q.size() are not the same. P.rows() = " << P.rows()
       << ", q.size() = " << q.size();
    throw std::invalid_argument(ss.str());
  }
  if (P.rows() != A.cols()) {
    ss << "P.rows() and A.cols() are not the same. P.rows() = " << P.rows()
       << ", A.cols() = " << A.cols();
    throw std::invalid_argument(ss.str());
  }
  if (A.rows() != static_cast<int>(l.size())) {
    ss << "A.rows() and l.size() are not the same. A.rows() = " << A.rows()
       << ", l.size() = " << l.size();
    throw std::invalid_argument(ss.str());
  }
  if (A.rows() != static_cast<int>(u.size())) {
    ss << "A.rows() and u.size() are not the same. A.rows() = " << A.rows()
       << ", u.size() = " << u.size();
    throw std::invalid_argument(ss.str());
  }

  CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
  CSC_Matrix A_csc = calCSCMatrix(A);
  return initializeProblem(P_csc, A_csc, q, l, u);
}

int64_t OSQPInterface::initializeProblem(
  CSC_Matrix P_csc, CSC_Matrix A_csc, const std::vector<double> & q, const std::vector<double> & l,
  const std::vector<double> & u)
{
  // Dynamic float arrays
  std::vector<OSQPFloat> q_tmp(q.begin(), q.end());
  std::vector<OSQPFloat> l_tmp(l.begin(), l.end());
  std::vector<OSQPFloat> u_tmp(u.begin(), u.end());

  /**********************
   * OBJECTIVE FUNCTION
   **********************/
  m_param_n = static_cast<int>(q.size());
  m_param_m = static_cast<OSQPInt>(l.size());
  const OSQPInt n = static_cast<OSQPInt>(m_param_n);

  /*****************
   * POPULATE DATA
   *****************/
  // The CSC wrappers only reference the data arrays (owned == 0); osqp_setup copies the problem
  // data internally, so the wrappers can be freed immediately afterwards.
  OSQPCscMatrix * P = OSQPCscMatrix_new(
    n, n, static_cast<OSQPInt>(P_csc.m_vals.size()), P_csc.m_vals.data(), P_csc.m_row_idxs.data(),
    P_csc.m_col_idxs.data());
  OSQPCscMatrix * A = OSQPCscMatrix_new(
    m_param_m, n, static_cast<OSQPInt>(A_csc.m_vals.size()), A_csc.m_vals.data(),
    A_csc.m_row_idxs.data(), A_csc.m_col_idxs.data());

  // Setup solver
  OSQPSolver * solver = nullptr;
  m_exitflag = osqp_setup(
    &solver, P, q_tmp.data(), A, l_tmp.data(), u_tmp.data(), m_param_m, n, m_settings.get());
  m_solver.reset(solver);
  m_work_initialized = true;

  OSQPCscMatrix_free(P);
  OSQPCscMatrix_free(A);

  return m_exitflag;
}

OSQPResult OSQPInterface::solve()
{
  // Solve Problem
  int32_t exit_flag = osqp_solve(m_solver.get());

  /********************
   * EXTRACT SOLUTION
   ********************/
  OSQPFloat * sol_x = m_solver->solution->x;
  OSQPFloat * sol_y = m_solver->solution->y;
  std::vector<double> sol_primal(sol_x, sol_x + m_param_n);
  std::vector<double> sol_lagrange_multiplier(sol_y, sol_y + m_param_m);

  int64_t status_polish = m_solver->info->status_polish;
  int64_t status_solution = m_solver->info->status_val;
  int64_t status_iteration = m_solver->info->iter;

  // Result tuple
  OSQPResult result;

  result.primal_solution = sol_primal;
  result.lagrange_multipliers = sol_lagrange_multiplier;
  result.polish_status = status_polish;
  result.solution_status = status_solution;
  result.iteration_status = status_iteration;
  result.exit_flag = exit_flag;

  m_latest_work_info = *(m_solver->info);

  return result;
}

OSQPResult OSQPInterface::optimize()
{
  // Run the solver on the stored problem representation.
  OSQPResult result = solve();
  return result;
}

OSQPResult OSQPInterface::optimize(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u)
{
  // Allocate memory for problem
  initializeProblem(P, A, q, l, u);

  // Run the solver on the stored problem representation.
  OSQPResult result = solve();

  m_solver.reset();
  m_work_initialized = false;

  return result;
}

void OSQPInterface::logUnsolvedStatus(const std::string & prefix_message) const
{
  const int status = getStatus();
  if (status == 1) {
    // No need to log since optimization was solved.
    return;
  }

  // create message
  std::string output_message = "";
  if (prefix_message != "") {
    output_message = prefix_message + " ";
  }

  const auto status_message = getStatusMessage();
  output_message += "Optimization failed due to " + status_message;

  // log with warning
  RCLCPP_WARN(rclcpp::get_logger("osqp_interface"), "%s", output_message.c_str());
}
}  // namespace autoware::osqp_interface
