// Copyright 2026 Autoware Foundation
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

#ifndef AUTOWARE__NDT_SCAN_MATCHER_COMMON__NDT_TYPES_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER_COMMON__NDT_TYPES_HPP_

#include <Eigen/Core>

#include <vector>

// The types of the NDT backend interface are in an inline namespace whose name holds three things:
// the version of the interface, the static alignment of Eigen, and the heap allocator of Eigen.
// The alignment depends on the SIMD options of the compiler (for example, AVX makes it 32). The
// allocator is std::malloc(), or an allocator of Eigen with another pointer layout (for example,
// with AVX, or with the address sanitizer of GCC). The interface passes fixed-size Eigen types by
// value, and one side frees vectors that the other side allocated, so both must agree.
//
// A backend that was built against another version, or with another alignment or allocator than
// the node, has other C++ symbols. Then the node loads its library but finds no factory for the
// backend. Without this, the node and the backend corrupt the memory of each other.
//
// The name does not hold the version of Eigen. The node and its backends must use the same Eigen.
//
// Change "v1" below when a declaration in ndt_types.hpp or ndt_backend.hpp changes.
#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
#error "The NDT backend interface needs the default (column-major) storage order of Eigen."
#endif
#if EIGEN_MAX_STATIC_ALIGN_BYTES == 0
#define AUTOWARE_NDT_BACKEND_ABI_ALIGN 0
#elif EIGEN_MAX_STATIC_ALIGN_BYTES == 16
#define AUTOWARE_NDT_BACKEND_ABI_ALIGN 16
#elif EIGEN_MAX_STATIC_ALIGN_BYTES == 32
#define AUTOWARE_NDT_BACKEND_ABI_ALIGN 32
#elif EIGEN_MAX_STATIC_ALIGN_BYTES == 64
#define AUTOWARE_NDT_BACKEND_ABI_ALIGN 64
#else
#error "Unknown value of EIGEN_MAX_STATIC_ALIGN_BYTES"
#endif
// The condition of Eigen::internal::aligned_malloc() for std::malloc().
#if (EIGEN_DEFAULT_ALIGN_BYTES == 0) || EIGEN_MALLOC_ALREADY_ALIGNED
#define AUTOWARE_NDT_BACKEND_ABI_ALLOCATOR std
#else
#define AUTOWARE_NDT_BACKEND_ABI_ALLOCATOR eigen
#endif
#define AUTOWARE_NDT_BACKEND_ABI_JOIN_IMPL(version, align, allocator) \
  version##_eigen_align_##align##_malloc_##allocator
#define AUTOWARE_NDT_BACKEND_ABI_JOIN(version, align, allocator) \
  AUTOWARE_NDT_BACKEND_ABI_JOIN_IMPL(version, align, allocator)
// For example, v1_eigen_align_16_malloc_std.
#define AUTOWARE_NDT_BACKEND_ABI \
  AUTOWARE_NDT_BACKEND_ABI_JOIN( \
    v1, AUTOWARE_NDT_BACKEND_ABI_ALIGN, AUTOWARE_NDT_BACKEND_ABI_ALLOCATOR)

namespace autoware::ndt_scan_matcher
{
inline namespace AUTOWARE_NDT_BACKEND_ABI
{
/// The parameters of an NDT alignment. The node sets every field from its parameters.
///
/// A step of the alignment is a 6-D vector: x, y and z in meters, and roll, pitch and yaw in
/// radians.
struct NdtParams
{
  /// The alignment stops when the length of a step is less than this.
  double trans_epsilon{};
  /// The largest length of one step.
  double step_size{};
  /// The size of a voxel of the map [m]. It is also the search radius for the voxels near a point.
  float resolution{};
  /// The largest number of iterations of one alignment.
  int max_iterations{};
  /// The number of CPU threads that the backend can use. At least 1.
  int num_threads{};
  /// The weight of the regularization. It has an effect only while a regularization pose is set
  /// (NdtBackend::set_regularization_pose()).
  float regularization_scale_factor{};
};

/// The result of an NDT alignment.
///
/// The node compares the scores with thresholds that are tuned for ndt_omp, so a backend must
/// compute them as ndt_omp does. The NDT score of a point for one voxel is the score of Magnusson
/// (2009, eq. 6.9) with an outlier ratio of 0.55. The voxels near a point are the voxels whose
/// mean is within NdtParams::resolution of the point.
///
/// The scores also depend on the voxels. ndt_omp builds them in this way:
/// - Each map cloud (each map_id) has its own grid. So two clouds can each have a voxel for the
///   same cell.
/// - A voxel is a cube with the side NdtParams::resolution. A point p is in the cell
///   floor(p / resolution).
/// - A voxel needs at least 6 points. Cells with fewer points have no voxel.
/// - The covariance of a voxel with n points p_i and the mean m is
///   (I + sum((p_i - m) * (p_i - m)^T)) / (n - 1). I is the 3x3 identity matrix. So it is larger
///   than the sample covariance by I / (n - 1).
/// - After that, an eigenvalue of the covariance that is less than 0.01 times the largest
///   eigenvalue is raised to that value.
///
/// The regularization score is -s * n * d * d. s is NdtParams::regularization_scale_factor. n is
/// the number of pairs of a point and a voxel near it. d is the distance from the pose to the
/// regularization pose along the heading of the pose: d = dx * cos(yaw) + dy * sin(yaw).
///
/// The backend "ndt_omp" of autoware_ndt_scan_matcher is the reference for all of these rules.
struct NdtResult
{
  /// The aligned pose (base_link to map).
  Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};
  /// The sum of the NDT scores of each point for all voxels near it, plus the regularization score
  /// if a regularization pose is set, divided by the number of all points (also the points with no
  /// voxel near them). 0 for an empty source. The node compares it with a threshold.
  float transform_probability{};
  /// The sum of the largest NDT score of each point among the voxels near it, divided by the
  /// number of points that have at least one voxel near them. 0 if no point has a voxel near it.
  /// It does not include the regularization. The node compares it with a threshold (the default).
  float nearest_voxel_transformation_likelihood{};
  /// The number of iterations. If it is not less than NdtParams::max_iterations, the node treats
  /// the alignment as not converged, unless the poses of transformation_array oscillate.
  int iteration_num{};
  /// The guess, then the pose after each iteration (iteration_num + 1 poses). The node finds
  /// oscillations in these poses, so an empty array changes which alignments the node accepts.
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> transformation_array;
  /// The hessian of the sum score (transform_probability times the number of points, with the
  /// regularization) at the aligned pose, in the order (x, y, z, roll, pitch, yaw). The angles are
  /// those of Eigen's eulerAngles(0, 1, 2). It is negative definite at a maximum of the score. For
  /// the covariance estimation LAPLACE, the node uses the inverse of its x-y block, negated, as the
  /// covariance of x and y.
  Eigen::Matrix<double, 6, 6> hessian{Eigen::Matrix<double, 6, 6>::Zero()};
  /// transform_probability at the guess and after each iteration (iteration_num + 1 values):
  /// front() at the guess, back() at the aligned pose. The node uses front() and back(). For other
  /// sizes, it does not use the array and gives a warning in the diagnostics.
  std::vector<float> transform_probability_array;
  /// The same as transform_probability_array, for nearest_voxel_transformation_likelihood.
  std::vector<float> nearest_voxel_transformation_likelihood_array;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace AUTOWARE_NDT_BACKEND_ABI
}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER_COMMON__NDT_TYPES_HPP_
