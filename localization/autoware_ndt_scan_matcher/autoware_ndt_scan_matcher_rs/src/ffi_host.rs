// Copyright 2024 Autoware Foundation
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

//! The ROS **side-effects host vtable** (`AwHost`) — the C-ABI adapter that realizes the portable
//! [`crate::host`] `Host` seam for the rclcpp node. A migrated callback body drives ROS side effects
//! (clock, logging, TF lookup, and — added as later sub-slices land — publishing) through this vtable
//! of C function pointers over an opaque `ctx` (the `NDTScanMatcher *`), instead of touching rclcpp
//! directly. Side-effects only; no node state (that lives Rust-side on the handle). Field order must
//! match the C `AwHost` struct.
//!
//! This is deliberately minimal for Phase 5 sub-slice 1 (the sensor-callback prologue needs only
//! `now_ns` / `log` / `lookup_transform`); publish ops are added with the sub-slices that move the
//! align + publish tail.

use core::ffi::c_void;

/// A borrowed UTF-8 (or raw) byte string crossing the C ABI: `(ptr, len)`, not NUL-terminated.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct AwStr {
    pub ptr: *const u8,
    pub len: usize,
}

impl AwStr {
    /// Borrow `self` as a byte slice for the duration of the call (null/zero-length → empty).
    ///
    /// # Safety
    /// When `len > 0`, `ptr` must point to `len` readable bytes that outlive the borrow.
    #[must_use]
    #[expect(
        unsafe_code,
        reason = "C ABI boundary; pointer validated per rust-c-ffi-safety"
    )]
    pub unsafe fn as_bytes<'a>(self) -> &'a [u8] {
        if self.ptr.is_null() || self.len == 0 {
            return &[];
        }
        // SAFETY: non-null with len > 0 per the check; caller guarantees `len` readable bytes.
        unsafe { core::slice::from_raw_parts(self.ptr, self.len) }
    }
}

/// Log severity byte for [`AwHost::log`] (mirrors the ROS logger levels used by the node).
pub const LOG_WARN: i32 = 1;
pub const LOG_ERROR: i32 = 2;

/// A pose crossing the C ABI: position + `[x, y, z, w]` quaternion (Phase 5 sub-slice 3 publish ops).
/// The C++ trampolines build the concrete ROS message; Rust only passes this POD.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct AwPose {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

/// Which pose publisher [`AwHost::publish_pose`] targets (the C++ side maps it to the concrete
/// publisher + message type; `NdtPose` is a `PoseStamped`, the others `PoseWithCovarianceStamped`).
#[repr(i32)]
#[derive(Clone, Copy)]
pub enum AwPoseTopic {
    NdtPose = 0,
    NdtPoseWithCovariance = 1,
    InitialPoseWithCovariance = 2,
}

/// Which `PoseArray` publisher [`AwHost::publish_pose_array`] targets (covariance debug arrays).
#[repr(i32)]
#[derive(Clone, Copy)]
pub enum AwPoseArrayTopic {
    MultiNdtPose = 0,
    MultiInitialPose = 1,
}

/// Which `Float32Stamped` publisher [`AwHost::publish_float32`] targets.
#[repr(i32)]
#[derive(Clone, Copy)]
pub enum AwFloat32Topic {
    TransformProbability = 0,
    NearestVoxelTransformationLikelihood = 1,
    NoGroundTransformProbability = 2,
    NoGroundNearestVoxelTransformationLikelihood = 3,
}

/// Which `Int32Stamped` publisher [`AwHost::publish_int32`] targets.
#[repr(i32)]
#[derive(Clone, Copy)]
pub enum AwInt32Topic {
    IterationNum = 0,
}

/// Which point-cloud publisher [`AwHost::publish_pointcloud_xyz`] targets.
#[repr(i32)]
#[derive(Clone, Copy)]
pub enum AwPointCloudTopic {
    PointsAligned = 0,
    PointsAlignedNoGround = 1,
    VoxelScorePoints = 2,
}

/// Borrowed `[[f32; 3]]` point slice crossing the C ABI.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct AwPoint3fSlice {
    pub ptr: *const f32,
    pub len: usize,
}

/// The ROS side-effects vtable. Built + owned C++-side (`make_host`); the trampolines cast `ctx` back
/// to `NDTScanMatcher *`. Every fn-pointer + `ctx` must stay valid for the duration of the call.
#[repr(C)]
pub struct AwHost {
    ctx: *mut c_void,
    /// `this->now().nanoseconds()`.
    now_ns: extern "C" fn(*mut c_void) -> i64,
    /// Emit a ROS log line at `level` (`LOG_WARN`/`LOG_ERROR`); `msg` is `(ptr, len)` UTF-8.
    log: extern "C" fn(*mut c_void, level: i32, msg: *const u8, msg_len: usize),
    /// Look up the `target←source` transform and write it as a row-major 4x4 `f32` into
    /// `out_matrix4x4_row_major` (16 floats). Returns `false` on a TF failure (out left untouched).
    lookup_transform: extern "C" fn(
        *mut c_void,
        target: AwStr,
        source: AwStr,
        out_matrix4x4_row_major: *mut f32,
    ) -> bool,

    // --- publish ops (Phase 5 sub-slice 3). POD in; the C++ trampolines build the ROS message +
    // fan out by topic, and know the frame_ids (static node config). Each is catch(...)-guarded C++
    // side so a publish exception never unwinds across the FFI. ---
    /// Publish a pose; `cov6x6_row_major` null ⇒ `PoseStamped`-family, else `PoseWithCovarianceStamped`.
    publish_pose: extern "C" fn(
        *mut c_void,
        topic: AwPoseTopic,
        stamp_ns: i64,
        pose: *const AwPose,
        cov6x6_row_major: *const f64,
    ),
    /// Publish a `PoseArray` of `n` poses (covariance debug).
    publish_pose_array: extern "C" fn(
        *mut c_void,
        topic: AwPoseArrayTopic,
        stamp_ns: i64,
        poses: *const AwPose,
        n: usize,
    ),
    /// Publish the NDT iteration-trajectory `MarkerArray` (C++ builds the ARROW markers, padded to
    /// `max_iterations + 2`).
    publish_marker: extern "C" fn(
        *mut c_void,
        stamp_ns: i64,
        poses: *const AwPose,
        n: usize,
        max_iterations: i32,
    ),
    /// Publish a `Float32Stamped` scalar.
    publish_float32: extern "C" fn(*mut c_void, topic: AwFloat32Topic, stamp_ns: i64, value: f32),
    /// Publish an `Int32Stamped` scalar.
    publish_int32: extern "C" fn(*mut c_void, topic: AwInt32Topic, stamp_ns: i64, value: i32),
    /// Broadcast the `map → ndt_base_frame` TF for the result pose.
    publish_tf: extern "C" fn(*mut c_void, stamp_ns: i64, pose: *const AwPose),
    /// Publish the initial-to-result debug quadruple (relative pose + 3 distances); the C++ side does
    /// the `inverse_transform_pose` + `norm` geometry. `old_pos`/`new_pos` are the bracket positions.
    publish_initial_to_result: extern "C" fn(
        *mut c_void,
        stamp_ns: i64,
        result: *const AwPose,
        initial: *const AwPose,
        old_pos: *const f64,
        new_pos: *const f64,
    ),
    /// Transitional until the align service moves to Rust: store the prepared `base_link` cloud in the
    /// C++ shell for `service_ndt_align`'s legacy `sensor_points_in_baselink_frame_` dependency.
    store_sensor_points_base_link: extern "C" fn(*mut c_void, points: AwPoint3fSlice),
    /// Whether a point-cloud publisher currently has subscribers. Used to avoid expensive optional
    /// score-cloud generation.
    pointcloud_has_subscribers: extern "C" fn(*mut c_void, topic: AwPointCloudTopic) -> bool,
    /// Publish an XYZ point cloud in map frame.
    publish_pointcloud_xyz:
        extern "C" fn(*mut c_void, topic: AwPointCloudTopic, stamp_ns: i64, points: AwPoint3fSlice),
    /// Publish voxel-score points. `points` and `scores` have the same length; C++ applies the
    /// existing score-to-RGB color map and publishes `voxel_score_points`.
    publish_voxel_score_points: extern "C" fn(
        *mut c_void,
        stamp_ns: i64,
        points: AwPoint3fSlice,
        scores: *const f32,
        scores_len: usize,
    ),
}

impl AwHost {
    /// The host clock in nanoseconds.
    #[must_use]
    pub fn now_ns(&self) -> i64 {
        (self.now_ns)(self.ctx)
    }

    /// Emit a ROS log line.
    pub fn log(&self, level: i32, msg: &str) {
        (self.log)(self.ctx, level, msg.as_ptr(), msg.len());
    }

    /// Look up the `target←source` transform as a row-major 4x4. `None` on TF failure.
    #[must_use]
    pub fn lookup_transform(&self, target: &[u8], source: &[u8]) -> Option<[f32; 16]> {
        let mut out = [0.0_f32; 16];
        let ok = (self.lookup_transform)(
            self.ctx,
            AwStr {
                ptr: target.as_ptr(),
                len: target.len(),
            },
            AwStr {
                ptr: source.as_ptr(),
                len: source.len(),
            },
            out.as_mut_ptr(),
        );
        ok.then_some(out)
    }

    /// Publish `pose` (with `cov` ⇒ `PoseWithCovarianceStamped`, else `PoseStamped`) on `topic`.
    pub fn publish_pose(
        &self,
        topic: AwPoseTopic,
        stamp_ns: i64,
        pose: &AwPose,
        cov: Option<&[f64; 36]>,
    ) {
        let cov_ptr = cov.map_or(core::ptr::null(), |c| c.as_ptr());
        (self.publish_pose)(self.ctx, topic, stamp_ns, pose, cov_ptr);
    }

    /// Publish `poses` as a `PoseArray` on `topic`.
    pub fn publish_pose_array(&self, topic: AwPoseArrayTopic, stamp_ns: i64, poses: &[AwPose]) {
        (self.publish_pose_array)(self.ctx, topic, stamp_ns, poses.as_ptr(), poses.len());
    }

    /// Publish the NDT iteration-trajectory markers (`max_iterations` sets the C++ padding length).
    pub fn publish_marker(&self, stamp_ns: i64, poses: &[AwPose], max_iterations: i32) {
        (self.publish_marker)(
            self.ctx,
            stamp_ns,
            poses.as_ptr(),
            poses.len(),
            max_iterations,
        );
    }

    /// Publish a `Float32Stamped` scalar on `topic`.
    pub fn publish_float32(&self, topic: AwFloat32Topic, stamp_ns: i64, value: f32) {
        (self.publish_float32)(self.ctx, topic, stamp_ns, value);
    }

    /// Publish an `Int32Stamped` scalar on `topic`.
    pub fn publish_int32(&self, topic: AwInt32Topic, stamp_ns: i64, value: i32) {
        (self.publish_int32)(self.ctx, topic, stamp_ns, value);
    }

    /// Broadcast the result-pose TF.
    pub fn publish_tf(&self, stamp_ns: i64, pose: &AwPose) {
        (self.publish_tf)(self.ctx, stamp_ns, pose);
    }

    /// Publish the initial-to-result debug quadruple (C++ does the relative-pose + distance geometry).
    pub fn publish_initial_to_result(
        &self,
        stamp_ns: i64,
        result: &AwPose,
        initial: &AwPose,
        old_pos: &[f64; 3],
        new_pos: &[f64; 3],
    ) {
        (self.publish_initial_to_result)(
            self.ctx,
            stamp_ns,
            result,
            initial,
            old_pos.as_ptr(),
            new_pos.as_ptr(),
        );
    }

    /// Store the prepared `base_link` cloud in the C++ shell for the deferred align-service path.
    pub fn store_sensor_points_base_link(&self, points: &[[f32; 3]]) {
        (self.store_sensor_points_base_link)(
            self.ctx,
            AwPoint3fSlice {
                ptr: points.as_ptr().cast::<f32>(),
                len: points.len(),
            },
        );
    }

    /// Whether a point-cloud topic currently has subscribers.
    #[must_use]
    pub fn pointcloud_has_subscribers(&self, topic: AwPointCloudTopic) -> bool {
        (self.pointcloud_has_subscribers)(self.ctx, topic)
    }

    /// Publish an XYZ point cloud in map frame.
    pub fn publish_pointcloud_xyz(
        &self,
        topic: AwPointCloudTopic,
        stamp_ns: i64,
        points: &[[f32; 3]],
    ) {
        (self.publish_pointcloud_xyz)(
            self.ctx,
            topic,
            stamp_ns,
            AwPoint3fSlice {
                ptr: points.as_ptr().cast::<f32>(),
                len: points.len(),
            },
        );
    }

    /// Publish voxel-score points with their nearest-voxel score intensities.
    pub fn publish_voxel_score_points(&self, stamp_ns: i64, points: &[[f32; 3]], scores: &[f32]) {
        (self.publish_voxel_score_points)(
            self.ctx,
            stamp_ns,
            AwPoint3fSlice {
                ptr: points.as_ptr().cast::<f32>(),
                len: points.len(),
            },
            scores.as_ptr(),
            scores.len(),
        );
    }
}
