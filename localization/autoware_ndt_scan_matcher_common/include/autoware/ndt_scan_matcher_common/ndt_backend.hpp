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

#ifndef AUTOWARE__NDT_SCAN_MATCHER_COMMON__NDT_BACKEND_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER_COMMON__NDT_BACKEND_HPP_

#include "autoware/ndt_scan_matcher_common/ndt_types.hpp"

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{
inline namespace AUTOWARE_NDT_BACKEND_ABI
{
/// An implementation of NDT scan matching for autoware_ndt_scan_matcher. The node loads it with
/// pluginlib, by the name in its parameter `ndt.backend`. The README of this package explains how
/// a package adds a backend.
///
/// Instances: the node calls initialize() only on the instance that pluginlib makes. clone() and
/// clone_empty() make instances that are ready without initialize(), so they must share or take
/// again what initialize() took. The node destroys the instance from pluginlib at the first full
/// map rebuild.
///
/// Threads: one thread at a time uses one instance, but different threads use it over its life.
/// So a backend must not need a resource that works only in one thread. Two instances (the active
/// one and the one that the map update module builds) run at the same time in two threads, so they
/// must not share mutable state.
///
/// Order of calls: after add_target() or remove_target(), the node calls commit_target() before it
/// calls any function of the instance other than these two. The node calls align() and the
/// calculate functions only while has_target() is true. The source of align() always has points.
/// The cloud of a calculate function can be empty.
///
/// Exceptions: an exception from initialize() stops the node at startup. The node does not catch
/// exceptions from the other functions either, so such an exception stops the node.
///
/// Library lifetime: the node never unloads a backend library. So clone() and clone_empty() can
/// make instances in the library without pluginlib.
class NdtBackend
{
public:
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using SharedPtr = std::shared_ptr<NdtBackend>;

  virtual ~NdtBackend() = default;

  /// Called once, after pluginlib makes the instance. A backend that needs a resource (for
  /// example, a GPU) gets it here, and throws std::runtime_error if it cannot run on this computer.
  virtual void initialize(const NdtParams & params) = 0;

  /// The name of the backend, for logs.
  virtual std::string name() const = 0;

  /// The parameters that the backend uses. The node compares NdtResult::iteration_num with
  /// get_params().max_iterations.
  virtual NdtParams get_params() const = 0;

  /// add_target() and remove_target() change the map, and commit_target() finishes the change.
  /// These three functions do all the map work, on an instance that no other thread uses at the
  /// same time. align() must not do map work that these functions deferred.
  ///
  /// The backend can keep the cloud. The node takes the ids from the map loader and does not
  /// examine them. Normally, each map_id is unique among the added clouds and remove_target() gets
  /// only ids that were added. remove_target() must ignore an id that was not added.
  virtual void add_target(
    const pcl::PointCloud<PointTarget>::ConstPtr & cloud, const std::string & map_id) = 0;
  virtual void remove_target(const std::string & map_id) = 0;
  virtual void commit_target() = 0;
  /// The ids of the map clouds that were added and not removed.
  virtual std::vector<std::string> get_current_map_ids() const = 0;
  /// True if get_current_map_ids() is not empty. The map can still have no voxel, for example if
  /// no cell has enough points. Then align() must return a result and must not fail.
  virtual bool has_target() const = 0;

  /// An independent instance with the same parameters, map, regularization pose and state. Later
  /// changes to one instance do not change the other. The node calls it while it holds the lock
  /// of the active instance, so a long copy delays the alignments. The instances can share
  /// immutable data, for example a map that is already on the GPU.
  virtual SharedPtr clone() const = 0;
  /// A new instance of the same backend, with the same parameters, without a map and without a
  /// regularization pose.
  virtual SharedPtr clone_empty() const = 0;

  /// Sets the regularization pose (base_link to map) for the next alignments, until
  /// unset_regularization_pose(). The regularization pulls the aligned pose toward the x and y of
  /// this pose, along the heading of the vehicle, with the weight
  /// NdtParams::regularization_scale_factor. The node sets it only if its parameter
  /// `ndt.regularization.enable` is true.
  virtual void set_regularization_pose(const Eigen::Matrix4f & pose) = 0;
  virtual void unset_regularization_pose() = 0;

  /// Aligns source (in the base_link frame) to the map, from guess (base_link to map). The backend
  /// does not keep source after it returns. NdtResult describes what the node needs from each
  /// field.
  virtual NdtResult align(
    const Eigen::Matrix4f & guess, const pcl::PointCloud<PointSource>::ConstPtr & source) = 0;

  /// The scores of NdtResult for a cloud that is already in the map frame, without the
  /// regularization. For an empty cloud, the two scores are 0 and the cloud of the third function
  /// is empty.
  virtual double calculate_transformation_probability(
    const pcl::PointCloud<PointSource> & cloud_in_map) const = 0;
  virtual double calculate_nearest_voxel_transformation_likelihood(
    const pcl::PointCloud<PointSource> & cloud_in_map) const = 0;
  /// The points of the cloud that have at least one voxel near them, each with its largest NDT
  /// score as the intensity, in any order.
  virtual pcl::PointCloud<pcl::PointXYZI> calculate_nearest_voxel_score_each_point(
    const pcl::PointCloud<PointSource> & cloud_in_map) const = 0;

protected:
  NdtBackend() = default;
  // Only a derived class copies the base, for clone(). This prevents slicing.
  NdtBackend(const NdtBackend &) = default;
  NdtBackend & operator=(const NdtBackend &) = delete;
};
}  // namespace AUTOWARE_NDT_BACKEND_ABI
}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER_COMMON__NDT_BACKEND_HPP_
