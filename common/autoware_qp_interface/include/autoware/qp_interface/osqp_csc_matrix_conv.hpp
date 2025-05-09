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

#ifndef AUTOWARE__QP_INTERFACE__OSQP_CSC_MATRIX_CONV_HPP_
#define AUTOWARE__QP_INTERFACE__OSQP_CSC_MATRIX_CONV_HPP_

#include <Eigen/Core>

#include <osqp/glob_opts.h>

#include <vector>

namespace autoware::qp_interface
{
/// \brief Compressed-Column-Sparse Matrix
struct CSC_Matrix
{
  /// Vector of non-zero values. Ex: [4,1,1,2]
  std::vector<c_float> vals_;
  /// Vector of row index corresponding to values. Ex: [0, 1, 0, 1] (Eigen: 'inner')
  std::vector<c_int> row_idxs_;
  /// Vector of 'val' indices where each column starts. Ex: [0, 2, 4] (Eigen: 'outer')
  std::vector<c_int> col_idxs_;
};

/// \brief Calculate CSC matrix from Eigen matrix
CSC_Matrix calCSCMatrix(const Eigen::MatrixXd & mat);
/// \brief Calculate upper trapezoidal CSC matrix from square Eigen matrix
CSC_Matrix calCSCMatrixTrapezoidal(const Eigen::MatrixXd & mat);
/// \brief Print the given CSC matrix to the standard output
void printCSCMatrix(const CSC_Matrix & csc_mat);

}  // namespace autoware::qp_interface

#endif  // AUTOWARE__QP_INTERFACE__OSQP_CSC_MATRIX_CONV_HPP_
