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

#include "autoware/qp_interface/osqp_csc_matrix_conv.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace autoware::qp_interface
{
OSQPInterface::OSQPInterface(
  const bool enable_warm_start, const int max_iteration, const OSQPFloat eps_abs,
  const OSQPFloat eps_rel, const bool polish, const bool verbose)
: QPInterface(enable_warm_start), solver_{nullptr, OSQPSolverDeleter}
{
  settings_ = std::make_unique<OSQPSettings>();

  if (settings_) {
    osqp_set_default_settings(settings_.get());
    settings_->alpha = 1.6;  // Change alpha parameter
    settings_->eps_rel = eps_rel;
    settings_->eps_abs = eps_abs;
    settings_->eps_prim_inf = 1.0E-4;
    settings_->eps_dual_inf = 1.0E-4;
    settings_->warm_starting = enable_warm_start;
    settings_->max_iter = max_iteration;
    settings_->verbose = verbose;
    settings_->polishing = polish;
  }
  exitflag_ = 0;
}

OSQPInterface::OSQPInterface(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u, const bool enable_warm_start,
  const OSQPFloat eps_abs)
: OSQPInterface(enable_warm_start, OSQP_MAX_ITERATION, eps_abs)
{
  initializeProblem(P, A, q, l, u);
}

OSQPInterface::OSQPInterface(
  const CSC_Matrix & P, const CSC_Matrix & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u, const bool enable_warm_start,
  const OSQPFloat eps_abs)
: OSQPInterface(enable_warm_start, OSQP_MAX_ITERATION, eps_abs)
{
  initializeCSCProblemImpl(P, A, q, l, u);
}

OSQPInterface::~OSQPInterface() = default;

void OSQPInterface::initializeProblemImpl(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
  const std::vector<double> & l, const std::vector<double> & u)
{
  CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
  CSC_Matrix A_csc = calCSCMatrix(A);
  initializeCSCProblemImpl(P_csc, A_csc, q, l, u);
}

void OSQPInterface::initializeCSCProblemImpl(
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
  param_n_ = static_cast<int>(q.size());
  param_m_ = static_cast<OSQPInt>(l.size());
  const OSQPInt n = static_cast<OSQPInt>(param_n_);

  /*****************
   * POPULATE DATA
   *****************/
  // The CSC wrappers only reference the data arrays (owned == 0); osqp_setup copies the problem
  // data internally, so the wrappers can be freed immediately afterwards.
  OSQPCscMatrix * P = OSQPCscMatrix_new(
    n, n, static_cast<OSQPInt>(P_csc.vals_.size()), P_csc.vals_.data(), P_csc.row_idxs_.data(),
    P_csc.col_idxs_.data());
  OSQPCscMatrix * A = OSQPCscMatrix_new(
    param_m_, n, static_cast<OSQPInt>(A_csc.vals_.size()), A_csc.vals_.data(),
    A_csc.row_idxs_.data(), A_csc.col_idxs_.data());

  // Setup solver
  OSQPSolver * solver = nullptr;
  exitflag_ = osqp_setup(
    &solver, P, q_tmp.data(), A, l_tmp.data(), u_tmp.data(), param_m_, n, settings_.get());
  solver_.reset(solver);
  work__initialized = true;

  OSQPCscMatrix_free(P);
  OSQPCscMatrix_free(A);
}

void OSQPInterface::OSQPSolverDeleter(OSQPSolver * ptr) noexcept
{
  if (ptr != nullptr) {
    osqp_cleanup(ptr);
  }
}

void OSQPInterface::updateSettings()
{
  // In OSQP 1.0.0 the per-field update functions were removed. Settings are updated by pushing the
  // whole settings object to the solver. Note that osqp_update_settings ignores scaling, sigma and
  // the adaptive_rho_* settings (those can only be set at osqp_setup time), and rho must be updated
  // via osqp_update_rho.
  if (work__initialized) {
    osqp_update_settings(solver_.get(), settings_.get());  // for current work
  }
}

void OSQPInterface::updateEpsAbs(const double eps_abs)
{
  settings_->eps_abs = eps_abs;  // for default setting
  updateSettings();              // for current work
}

void OSQPInterface::updateEpsRel(const double eps_rel)
{
  settings_->eps_rel = eps_rel;  // for default setting
  updateSettings();              // for current work
}

void OSQPInterface::updateMaxIter(const int max_iter)
{
  settings_->max_iter = max_iter;  // for default setting
  updateSettings();                // for current work
}

void OSQPInterface::updateVerbose(const bool is_verbose)
{
  settings_->verbose = is_verbose;  // for default setting
  updateSettings();                 // for current work
}

void OSQPInterface::updateRhoInterval(const int rho_interval)
{
  settings_->adaptive_rho_interval = rho_interval;  // for default setting
}

void OSQPInterface::updateRho(const double rho)
{
  settings_->rho = rho;
  if (work__initialized) {
    osqp_update_rho(solver_.get(), rho);
  }
}

void OSQPInterface::updateAlpha(const double alpha)
{
  settings_->alpha = alpha;  // for default setting
  updateSettings();          // for current work
}

void OSQPInterface::updateScaling(const int scaling)
{
  settings_->scaling = scaling;
}

void OSQPInterface::updatePolish(const bool polish)
{
  settings_->polishing = polish;  // for default setting
  updateSettings();               // for current work
}

void OSQPInterface::updatePolishRefinementIteration(const int polish_refine_iter)
{
  if (polish_refine_iter < 0) {
    std::cerr << "Polish refinement iterations must be positive" << std::endl;
    return;
  }

  settings_->polish_refine_iter = polish_refine_iter;  // for default setting
  updateSettings();                                    // for current work
}

void OSQPInterface::updateCheckTermination(const int check_termination)
{
  if (check_termination < 0) {
    std::cerr << "Check termination must be positive" << std::endl;
    return;
  }

  settings_->check_termination = check_termination;  // for default setting
  updateSettings();                                  // for current work
}

bool OSQPInterface::setWarmStart(
  const std::vector<double> & primal_variables, const std::vector<double> & dual_variables)
{
  return setPrimalVariables(primal_variables) && setDualVariables(dual_variables);
}

bool OSQPInterface::setPrimalVariables(const std::vector<double> & primal_variables)
{
  if (!work__initialized) {
    return false;
  }

  const auto result = osqp_warm_start(solver_.get(), primal_variables.data(), OSQP_NULL);
  if (result != 0) {
    std::cerr << "Failed to set primal variables for warm start" << std::endl;
    return false;
  }

  return true;
}

bool OSQPInterface::setDualVariables(const std::vector<double> & dual_variables)
{
  if (!work__initialized) {
    return false;
  }

  const auto result = osqp_warm_start(solver_.get(), OSQP_NULL, dual_variables.data());
  if (result != 0) {
    std::cerr << "Failed to set dual variables for warm start" << std::endl;
    return false;
  }

  return true;
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
    solver_.get(), P_csc.vals_.data(), OSQP_NULL, static_cast<OSQPInt>(P_csc.vals_.size()),
    OSQP_NULL, OSQP_NULL, 0);
}

void OSQPInterface::updateCscP(const CSC_Matrix & P_csc)
{
  osqp_update_data_mat(
    solver_.get(), P_csc.vals_.data(), OSQP_NULL, static_cast<OSQPInt>(P_csc.vals_.size()),
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
    solver_.get(), OSQP_NULL, OSQP_NULL, 0, A_csc.vals_.data(), OSQP_NULL,
    static_cast<OSQPInt>(A_csc.vals_.size()));
  return;
}

void OSQPInterface::updateCscA(const CSC_Matrix & A_csc)
{
  osqp_update_data_mat(
    solver_.get(), OSQP_NULL, OSQP_NULL, 0, A_csc.vals_.data(), OSQP_NULL,
    static_cast<OSQPInt>(A_csc.vals_.size()));
}

void OSQPInterface::updateQ(const std::vector<double> & q_new)
{
  std::vector<OSQPFloat> q_tmp(q_new.begin(), q_new.end());
  osqp_update_data_vec(solver_.get(), q_tmp.data(), OSQP_NULL, OSQP_NULL);
}

void OSQPInterface::updateL(const std::vector<double> & l_new)
{
  std::vector<OSQPFloat> l_tmp(l_new.begin(), l_new.end());
  osqp_update_data_vec(solver_.get(), OSQP_NULL, l_tmp.data(), OSQP_NULL);
}

void OSQPInterface::updateU(const std::vector<double> & u_new)
{
  std::vector<OSQPFloat> u_tmp(u_new.begin(), u_new.end());
  osqp_update_data_vec(solver_.get(), OSQP_NULL, OSQP_NULL, u_tmp.data());
}

void OSQPInterface::updateBounds(
  const std::vector<double> & l_new, const std::vector<double> & u_new)
{
  std::vector<OSQPFloat> l_tmp(l_new.begin(), l_new.end());
  std::vector<OSQPFloat> u_tmp(u_new.begin(), u_new.end());
  osqp_update_data_vec(solver_.get(), OSQP_NULL, l_tmp.data(), u_tmp.data());
}

int OSQPInterface::getIterationNumber() const
{
  return solver_->info->iter;
}

std::string OSQPInterface::getStatus() const
{
  return "OSQP_SOLVED";
}

bool OSQPInterface::isSolved() const
{
  return latest_work_info_.status_val == 1;
}

int OSQPInterface::getPolishStatus() const
{
  return static_cast<int>(latest_work_info_.status_polish);
}

std::vector<double> OSQPInterface::getDualSolution() const
{
  OSQPFloat * sol_y = solver_->solution->y;
  std::vector<double> dual_solution(sol_y, sol_y + param_m_);
  return dual_solution;
}

std::vector<double> OSQPInterface::optimizeImpl()
{
  osqp_solve(solver_.get());

  OSQPFloat * sol_x = solver_->solution->x;
  std::vector<double> sol_primal(sol_x, sol_x + param_n_);

  latest_work_info_ = *(solver_->info);

  if (!enable_warm_start_) {
    solver_.reset();
    work__initialized = false;
  }

  return sol_primal;
}

std::vector<double> OSQPInterface::optimize(
  CSC_Matrix P, CSC_Matrix A, const std::vector<double> & q, const std::vector<double> & l,
  const std::vector<double> & u)
{
  initializeCSCProblemImpl(P, A, q, l, u);
  const auto result = optimizeImpl();

  // show polish status if not successful
  const int status_polish = static_cast<int>(latest_work_info_.status_polish);
  if (status_polish != 1) {
    const auto msg = status_polish == 0    ? "unperformed"
                     : status_polish == -1 ? "unsuccessful"
                                           : "unknown";
    std::cerr << "osqp polish process failed : " << msg << ". The result may be inaccurate"
              << std::endl;
  }

  return result;
}

}  // namespace autoware::qp_interface
