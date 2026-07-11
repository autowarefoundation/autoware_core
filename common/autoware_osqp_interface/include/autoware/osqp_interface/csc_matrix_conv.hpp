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

#ifndef AUTOWARE__OSQP_INTERFACE__CSC_MATRIX_CONV_HPP_
#define AUTOWARE__OSQP_INTERFACE__CSC_MATRIX_CONV_HPP_

#include "autoware/osqp_interface/visibility_control.hpp"
#include "osqp/osqp.h"  // for 'OSQPInt' and 'OSQPFloat' types

#include <Eigen/Core>

#include <vector>

namespace autoware::osqp_interface
{
/// \brief Compressed-Column-Sparse Matrix
struct OSQP_INTERFACE_PUBLIC CSC_Matrix
{
  /// Vector of non-zero values. Ex: [4,1,1,2]
  std::vector<OSQPFloat> m_vals;
  /// Vector of row index corresponding to values. Ex: [0, 1, 0, 1] (Eigen: 'inner')
  std::vector<OSQPInt> m_row_idxs;
  /// Vector of 'val' indices where each column starts. Ex: [0, 2, 4] (Eigen: 'outer')
  std::vector<OSQPInt> m_col_idxs;

  friend std::ostream & operator<<(std::ostream & os, const CSC_Matrix & matrix)
  {
    os << "CSC_Matrix: {\n";
    os << "\tm_vals: [";

    // Iterator-based loop for m_vals
    for (auto it = std::begin(matrix.m_vals); it != std::end(matrix.m_vals); ++it) {
      os << *it;  // Print the current element (dereference iterator)
      if (std::next(it) != std::end(matrix.m_vals)) {  // Check if not the last element
        os << ", ";
      }
    }
    os << "],\n";

    os << "\tm_row_idxs: [";
    // Iterator-based loop for m_row_idxs
    for (auto it = std::begin(matrix.m_row_idxs); it != std::end(matrix.m_row_idxs); ++it) {
      os << *it;
      if (std::next(it) != std::end(matrix.m_row_idxs)) {
        os << ", ";
      }
    }
    os << "],\n";

    os << "\tm_col_idxs: [";
    // Iterator-based loop for m_col_idxs
    for (auto it = std::begin(matrix.m_col_idxs); it != std::end(matrix.m_col_idxs); ++it) {
      os << *it;
      if (std::next(it) != std::end(matrix.m_col_idxs)) {
        os << ", ";
      }
    }
    os << "]\n";

    os << "}\n";
    return os;
  }
};

/// \brief Calculate CSC matrix from Eigen matrix
OSQP_INTERFACE_PUBLIC CSC_Matrix calCSCMatrix(const Eigen::MatrixXd & mat);
/// \brief Calculate upper trapezoidal CSC matrix from square Eigen matrix
OSQP_INTERFACE_PUBLIC CSC_Matrix calCSCMatrixTrapezoidal(const Eigen::MatrixXd & mat);
/// \brief Print the given CSC matrix to the standard output
OSQP_INTERFACE_PUBLIC void printCSCMatrix(const CSC_Matrix & csc_mat);

}  // namespace autoware::osqp_interface

#endif  // AUTOWARE__OSQP_INTERFACE__CSC_MATRIX_CONV_HPP_
