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

//! The sensor-callback **prologue** in Rust (Phase 5 sub-slice 1): validate the incoming
//! `PointCloud2`, decode its xyz, look up the sensor→`base_link` transform through the [`AwHost`]
//! vtable, transform the points into `base_link`, and gate on size / delay / max-distance — mirroring
//! `callback_sensor_points_main`'s first ~80 lines. The transformed `base_link` points are written
//! into a caller-provided buffer (the C++ tail rebuilds its pcl cloud from them); diagnostics are
//! emitted through the diagnostics vtable with the same keys/levels as the C++ body. std-only.

use nalgebra::{
    Isometry3, Matrix3, Matrix4, Matrix6, Quaternion, Rotation3, Translation3, UnitQuaternion,
};

use crate::engine::{
    ConvergenceParams, CovEstimationParams, NdtEngine, estimate_pose_covariance, run_align,
};
use crate::ffi_host::{
    AwFloat32Topic, AwHost, AwInt32Topic, AwPointCloudTopic, AwPose, AwPoseArrayTopic, AwPoseTopic,
    LOG_ERROR, LOG_WARN,
};
use crate::node::{DIAGNOSTIC_ERROR, DIAGNOSTIC_WARN, Diagnostics};
use crate::node_handle::NdtScanMatcherRs;

/// `sensor_msgs::msg::PointField::FLOAT32` — the only xyz datatype the fast path decodes.
const POINT_FIELD_FLOAT32: u8 = 7;

/// Outcome of [`autoware_ndt_scan_matcher_rs_node_on_sensor_points_prepare`] (mirrors the C++
/// early-returns). `PREPARED` = the `base_link` points were written to the out buffer.
pub const SP_PREPARED: i32 = 0;
/// The cloud was empty (`width == 0`) → the C++ WARN + `return false`.
pub const SP_EMPTY: i32 = 1;
/// The sensor→`base_link` transform was unavailable → the C++ ERROR + `return false`.
pub const SP_TF_FAILED: i32 = 2;
/// The max point distance was below `required_distance` → the C++ WARN + `return false`.
pub const SP_TOO_CLOSE: i32 = 3;
/// A pointer was null or the cloud's xyz datatype is not `FLOAT32` (cannot decode).
pub const SP_INVALID: i32 = 4;

/// C-ABI view of a `sensor_msgs::msg::PointCloud2` (borrowed for the call only). `x_offset` /
/// `y_offset` / `z_offset` are byte offsets within each `point_step`-sized record; `datatype` is the
/// shared xyz `PointField` datatype (must be `FLOAT32`). `frame_id` is `(ptr, len)` UTF-8.
#[repr(C)]
pub struct AwPointCloud2View {
    pub stamp_ns: i64,
    pub frame_id: crate::ffi_host::AwStr,
    pub width: u32,
    pub height: u32,
    pub point_step: u32,
    pub row_step: u32,
    pub data: *const u8,
    pub data_len: usize,
    pub x_offset: u32,
    pub y_offset: u32,
    pub z_offset: u32,
    pub datatype: u8,
    pub is_bigendian: bool,
}

/// Read an `f32` at byte offset `at` in `data` (little- or big-endian), or `None` if out of bounds.
fn read_f32(data: &[u8], at: usize, big_endian: bool) -> Option<f32> {
    let end = at.checked_add(4)?;
    let bytes: [u8; 4] = data.get(at..end)?.try_into().ok()?;
    Some(if big_endian {
        f32::from_be_bytes(bytes)
    } else {
        f32::from_le_bytes(bytes)
    })
}

/// Apply a row-major 4x4 `f32` transform to `(x, y, z)` (matches `pcl::transformPointCloud` with an
/// Eigen `Matrix4f`). Constant indices into the fixed `[f32; 16]` are not lint-flagged.
fn transform_point(m: &[f32; 16], x: f32, y: f32, z: f32) -> [f32; 3] {
    [
        m[0] * x + m[1] * y + m[2] * z + m[3],
        m[4] * x + m[5] * y + m[6] * z + m[7],
        m[8] * x + m[9] * y + m[10] * z + m[11],
    ]
}

/// The sensor-callback prologue: validate + decode + transform-to-`base_link`. Writes the transformed
/// `base_link` xyz into `out_points_xyz` (`3 * point_count` floats, capacity `out_cap`), sets
/// `*out_count`, emits diagnostics, and returns an `SP_*` status. On any non-`PREPARED` status the
/// out buffer / count are the caller's responsibility to ignore.
///
/// # Safety
/// All pointers must be valid/live for the call (or null → `SP_INVALID`). `view.data` addresses
/// `view.data_len` readable bytes; `out_points_xyz` addresses `out_cap` writable `f32`. The `host`
/// fn-pointers + `ctx` and the `diag` handle outlive the call.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[expect(
    clippy::many_single_char_names,
    reason = "terse locals for the ported prologue (h/ho/d/v handles + x/y/z/p geometry)"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_on_sensor_points_prepare(
    handle: *const NdtScanMatcherRs,
    host: *const AwHost,
    diag: *const Diagnostics,
    view: *const AwPointCloud2View,
    out_points_xyz: *mut f32,
    out_cap: usize,
    out_count: *mut usize,
) -> i32 {
    if handle.is_null()
        || host.is_null()
        || diag.is_null()
        || view.is_null()
        || out_points_xyz.is_null()
        || out_count.is_null()
    {
        return SP_INVALID;
    }
    // SAFETY: non-null per the check; caller guarantees valid, live handle/host/diag/view.
    let (h, ho, d, v) = unsafe { (&*handle, &*host, &*diag, &*view) };
    // SAFETY: `out_points_xyz` addresses `out_cap` writable f32 per the contract.
    let out = unsafe { core::slice::from_raw_parts_mut(out_points_xyz, out_cap) };

    d.add_i64("topic_time_stamp", v.stamp_ns);

    // check sensor_points_size (the C++ uses `width`).
    let width = usize::try_from(v.width).unwrap_or(0);
    let height = usize::try_from(v.height).unwrap_or(0);
    d.add_i64("sensor_points_size", i64::from(v.width));
    if v.width == 0 {
        d.update_level(DIAGNOSTIC_WARN, "Sensor points is empty.");
        return SP_EMPTY;
    }

    // check sensor_points_delay_time_sec (diagnostic only; no early return, mirroring the C++).
    let delay_sec = ns_to_sec(ho.now_ns().saturating_sub(v.stamp_ns));
    d.add_f64("sensor_points_delay_time_sec", delay_sec);
    if delay_sec > h.params.sensor_points_timeout_sec {
        d.update_level(
            DIAGNOSTIC_WARN,
            &alloc::format!(
                "sensor points is experiencing latency.The delay time is {delay_sec}[sec] \
                 (the tolerance is {}[sec]).",
                h.params.sensor_points_timeout_sec
            ),
        );
    }

    if v.datatype != POINT_FIELD_FLOAT32 {
        return SP_INVALID;
    }

    // look up base_link←sensor (target = base_frame, source = the cloud's frame).
    // SAFETY: `frame_id` is a valid AwStr for the call.
    let sensor_frame = unsafe { v.frame_id.as_bytes() };
    let Some(matrix) = ho.lookup_transform(&h.params.base_frame, sensor_frame) else {
        d.add_bool("is_succeed_transform_sensor_points", false);
        ho.log(
            LOG_ERROR,
            "Failed to look up the sensor→base_link transform.",
        );
        return SP_TF_FAILED;
    };
    d.add_bool("is_succeed_transform_sensor_points", true);

    // decode xyz + transform into base_link, tracking the max distance.
    // SAFETY: `data` addresses `data_len` readable bytes per the contract.
    let data = unsafe { core::slice::from_raw_parts(v.data, v.data_len) };
    let point_step = usize::try_from(v.point_step).unwrap_or(0);
    let x_off = usize::try_from(v.x_offset).unwrap_or(0);
    let y_off = usize::try_from(v.y_offset).unwrap_or(0);
    let z_off = usize::try_from(v.z_offset).unwrap_or(0);
    let n_by_data = data.len().checked_div(point_step).unwrap_or(0);
    let n = width
        .saturating_mul(height)
        .min(n_by_data)
        .min(out.len().checked_div(3).unwrap_or(0));

    let mut max_distance = 0.0_f64;
    let mut count = 0usize;
    // `n <= out.len() / 3`, so `take(n)` never exhausts the buffer; each `slot` is exactly 3 wide.
    for (i, slot) in out.chunks_exact_mut(3).take(n).enumerate() {
        let base = i.saturating_mul(point_step);
        let (Some(x), Some(y), Some(z)) = (
            read_f32(data, base.saturating_add(x_off), v.is_bigendian),
            read_f32(data, base.saturating_add(y_off), v.is_bigendian),
            read_f32(data, base.saturating_add(z_off), v.is_bigendian),
        ) else {
            break;
        };
        let p = transform_point(&matrix, x, y, z);
        let [ox, oy, oz] = slot else {
            break;
        };
        *ox = p[0];
        *oy = p[1];
        *oz = p[2];
        let (dx, dy, dz) = (f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
        max_distance = max_distance.max(libm::sqrt(dx * dx + dy * dy + dz * dz));
        count = count.saturating_add(1);
    }

    d.add_f64("sensor_points_max_distance", max_distance);
    if max_distance < h.params.sensor_points_required_distance {
        d.update_level(
            DIAGNOSTIC_WARN,
            &alloc::format!(
                "Max distance of sensor points = {max_distance:.3} [m] < {:.3} [m]",
                h.params.sensor_points_required_distance
            ),
        );
        return SP_TOO_CLOSE;
    }

    // SAFETY: `out_count` is non-null per the check and points to a writable usize.
    unsafe {
        *out_count = count;
    }
    let prepared = if count == 0 {
        &[]
    } else {
        // SAFETY: `out` contains at least `3 * count` initialized f32 values written above.
        unsafe { core::slice::from_raw_parts(out.as_ptr().cast::<[f32; 3]>(), count) }
    };
    h.store_latest_sensor_points(prepared);
    SP_PREPARED
}

// --- Phase 5 sub-slice 2: the sensor-callback *middle* orchestrator ---
// One Rust call replaces the C++ glue between the prologue and the publishers: the activation /
// interpolate / map gates, regularization, `run_align` + convergence, and the covariance block —
// emitting the same `/diagnostics` keys (in order) through the `Diagnostics` vtable. The C++ shell
// still publishes, reading the returned [`AwSensorPointsMatchOutput`]; the publish vtable + collapsing
// the base_link round-trip are later sub-slices. std-only node glue.

/// Outcome of [`autoware_ndt_scan_matcher_rs_node_on_sensor_points_match`]. `MATCHED` = the align ran
/// and `out` is filled (C++ publishes from it — pose publication is still gated on `out.is_converged`).
/// The others mirror the C++ callback's early `return false` gates (no publish).
pub const SM_MATCHED: i32 = 0;
/// The node is not activated (the C++ WARN + `return false`).
pub const SM_NOT_ACTIVATED: i32 = 1;
/// The initial pose could not be interpolated (the C++ WARN + `return false`).
pub const SM_INTERPOLATE_FAILED: i32 = 2;
/// No map tile is loaded yet (the C++ WARN + `return false`).
pub const SM_MAP_NOT_SET: i32 = 3;
/// `converged_param_type` is unknown (the C++ ERROR + `return false`, after align).
pub const SM_INVALID_PARAM_TYPE: i32 = 4;
/// A pointer was null / bad args.
pub const SM_INVALID: i32 = 5;

/// The few scalars the middle needs that are not on the node handle's [`crate::node_handle::Params`]:
/// the map-update `out_of_map_range` radii, the `initial_to_result` distance tolerance, and the
/// regularization scale factor (C++ `param_.ndt.regularization_scale_factor`).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AwSensorPointsMatchParams {
    pub lidar_radius: f64,
    pub map_radius: f64,
    pub initial_to_result_distance_tolerance_m: f64,
    pub regularization_scale_factor: f64,
    pub no_ground_points_enable: bool,
    pub no_ground_points_z_margin_for_ground_removal: f64,
}

/// C-ABI result of the middle (Phase 5 sub-slice 3: the publishers now go through the `AwHost` publish
/// ops, so this shrank to what the C++ shell still needs). `result_pose` (row-major 4x4) lets C++
/// transform the `base_link` cloud → map for the still-C++ cloud publishers; `is_converged` feeds the
/// wrapper's `skipping_publish_num`.
#[repr(C)]
pub struct AwSensorPointsMatchOutput {
    pub result_pose: [f32; 16],
    pub is_converged: bool,
}

/// Build a 4x4 `f32` transform from a position + `[x, y, z, w]` quaternion (mirrors the C++
/// `pose_to_matrix4f`: Affine3d from the pose, `cast<float>`).
#[expect(
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    reason = "deliberate f64->f32 of the interpolated guess pose, mirroring pose_to_matrix4f's cast<float>"
)]
fn pose_to_matrix4(position: &[f64; 3], orientation: &[f64; 4]) -> Matrix4<f32> {
    let q = UnitQuaternion::from_quaternion(Quaternion::new(
        orientation[3],
        orientation[0],
        orientation[1],
        orientation[2],
    ));
    let iso = Isometry3::from_parts(Translation3::new(position[0], position[1], position[2]), q);
    iso.to_homogeneous().map(|v| v as f32)
}

/// Row-major flatten of a 4x4 (nalgebra is column-major, so the transpose's column-major storage IS
/// the original's row-major order).
fn matrix4_to_row_major(m: &Matrix4<f32>) -> [f32; 16] {
    let mut out = [0.0_f32; 16];
    out.copy_from_slice(m.transpose().as_slice());
    out
}

/// Row-major flatten of a 6x6 (see [`matrix4_to_row_major`]).
fn matrix6_to_row_major(m: &Matrix6<f64>) -> [f64; 36] {
    let mut out = [0.0_f64; 36];
    out.copy_from_slice(m.transpose().as_slice());
    out
}

/// The `map`←`base_link` rotation the covariance block rotates the configured 6x6 by: the C++ takes
/// the result-pose quaternion (`matrix4f_to_pose`), normalizes it, and back to a 3x3. Reproduced via
/// nalgebra (matrix → unit quaternion → matrix), row-major.
fn map_to_base_link_rot3x3(result_pose: &Matrix4<f32>) -> [f64; 9] {
    let rot_f64: Matrix3<f64> = result_pose
        .fixed_view::<3, 3>(0, 0)
        .into_owned()
        .map(f64::from);
    let uq = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rot_f64));
    let mut out = [0.0_f64; 9];
    out.copy_from_slice(uq.to_rotation_matrix().matrix().transpose().as_slice());
    out
}

/// Convert a 4x4 `f32` transform to an [`AwPose`] (translation from column 3; rotation → unit
/// quaternion `[x, y, z, w]`), mirroring the C++ `matrix4f_to_pose` used by the publishers.
fn matrix4_to_aw_pose(m: &Matrix4<f32>) -> AwPose {
    let rot: Matrix3<f64> = m.fixed_view::<3, 3>(0, 0).into_owned().map(f64::from);
    let q =
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rot)).into_inner();
    AwPose {
        position: [
            f64::from(m[(0, 3)]),
            f64::from(m[(1, 3)]),
            f64::from(m[(2, 3)]),
        ],
        orientation: [q.i, q.j, q.k, q.w],
    }
}

/// Transform a base_link-frame cloud into map frame with the row-major result pose.
fn transform_cloud_to_map(source: &[[f32; 3]], pose: &Matrix4<f32>) -> alloc::vec::Vec<[f32; 3]> {
    source
        .iter()
        .map(|p| {
            [
                pose[(0, 0)] * p[0] + pose[(0, 1)] * p[1] + pose[(0, 2)] * p[2] + pose[(0, 3)],
                pose[(1, 0)] * p[0] + pose[(1, 1)] * p[1] + pose[(1, 2)] * p[2] + pose[(1, 3)],
                pose[(2, 0)] * p[0] + pose[(2, 1)] * p[1] + pose[(2, 2)] * p[2] + pose[(2, 3)],
            ]
        })
        .collect()
}

/// Keep only points above the result-pose ground-removal plane, matching the C++ no-ground filter.
#[expect(
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    reason = "ported f64->f32 no-ground threshold math mirrors the C++ callback"
)]
fn no_ground_points(
    points_map: &[[f32; 3]],
    result_pose_z: f32,
    margin: f64,
) -> alloc::vec::Vec<[f32; 3]> {
    let threshold = result_pose_z + margin as f32;
    points_map
        .iter()
        .copied()
        .filter(|p| p[2] > threshold)
        .collect()
}

/// Compute and publish the optional score/no-ground clouds that used to live in the C++ epilogue.
fn publish_cloud_outputs(
    eng: &NdtEngine,
    ho: &AwHost,
    prm: &AwSensorPointsMatchParams,
    sensor_stamp_ns: i64,
    result_pose: &Matrix4<f32>,
    points_map: &[[f32; 3]],
) {
    ho.publish_pointcloud_xyz(
        AwPointCloudTopic::PointsAligned,
        sensor_stamp_ns,
        points_map,
    );

    if ho.pointcloud_has_subscribers(AwPointCloudTopic::VoxelScorePoints) {
        let mut scores_all = alloc::vec::Vec::new();
        eng.nearest_voxel_score_each_point(points_map, &mut scores_all);
        let mut scored_points = alloc::vec::Vec::new();
        let mut scored_values = alloc::vec::Vec::new();
        for (point, score) in points_map.iter().copied().zip(scores_all) {
            if score > 0.0 {
                scored_points.push(point);
                scored_values.push(score);
            }
        }
        ho.publish_voxel_score_points(sensor_stamp_ns, &scored_points, &scored_values);
    }

    if prm.no_ground_points_enable {
        let no_ground = no_ground_points(
            points_map,
            result_pose[(2, 3)],
            prm.no_ground_points_z_margin_for_ground_removal,
        );
        ho.publish_pointcloud_xyz(
            AwPointCloudTopic::PointsAlignedNoGround,
            sensor_stamp_ns,
            &no_ground,
        );
        let tp = eng.calc_transformation_probability(&no_ground);
        let nvtl = eng.calc_nearest_voxel_likelihood(&no_ground);
        #[expect(
            clippy::as_conversions,
            clippy::cast_possible_truncation,
            reason = "C++ publishes these score scalars as float32"
        )]
        {
            ho.publish_float32(
                AwFloat32Topic::NoGroundTransformProbability,
                sensor_stamp_ns,
                tp as f32,
            );
            ho.publish_float32(
                AwFloat32Topic::NoGroundNearestVoxelTransformationLikelihood,
                sensor_stamp_ns,
                nvtl as f32,
            );
        }
    }
}

/// Collect a slice of 4x4 poses into `AwPose`s (for the marker trajectory + covariance debug arrays).
fn aw_poses(poses: &[Matrix4<f32>]) -> alloc::vec::Vec<AwPose> {
    poses.iter().map(matrix4_to_aw_pose).collect()
}

/// The sensor-callback middle: gates → align → convergence → covariance, emitting the diagnostics and
/// **publishing the POD results through the `AwHost` publish ops** (Phase 5 sub-slice 3); `out` carries
/// only the result pose + convergence for the still-C++ cloud publishers + `skipping_publish_num`.
/// Returns an `SM_*` status. Mirrors `callback_sensor_points_main` lines ~560–981 in order.
///
/// # Safety
/// All pointers must be valid/live for the call (or null → `SM_INVALID`). `source_xyz` addresses
/// `3 * source_count` readable `f32`; `out` is a writable [`AwSensorPointsMatchOutput`]. The
/// `host`/`diag` vtables + `engine`/`handle` outlive the call.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[expect(
    clippy::too_many_lines,
    reason = "faithful line-by-line port of the C++ sensor-callback middle, kept as one orchestrator so the gate/diagnostic order is obvious"
)]
#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::allow_attributes,
    reason = "f64 geometry math + fixed-size nalgebra/array indexing + the documented f64->f32 regularization/pose casts of the ported middle"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_on_sensor_points_match(
    handle: *const NdtScanMatcherRs,
    engine: *const NdtEngine,
    host: *const AwHost,
    diag: *const Diagnostics,
    params: *const AwSensorPointsMatchParams,
    sensor_stamp_ns: i64,
    source_xyz: *const f32,
    source_count: usize,
    out: *mut AwSensorPointsMatchOutput,
) -> i32 {
    if handle.is_null()
        || engine.is_null()
        || host.is_null()
        || diag.is_null()
        || params.is_null()
        || out.is_null()
        || (source_count > 0 && source_xyz.is_null())
    {
        return SM_INVALID;
    }
    // SAFETY: non-null per the check; caller guarantees valid, live handle/engine/host/diag/params/out.
    let (h, eng, ho, d, prm, o) =
        unsafe { (&*handle, &*engine, &*host, &*diag, &*params, &mut *out) };
    // SAFETY: `source_xyz` addresses `3 * source_count` readable f32 per the contract (empty if null/0).
    let source: &[[f32; 3]] = if source_count == 0 {
        &[]
    } else {
        unsafe { core::slice::from_raw_parts(source_xyz.cast::<[f32; 3]>(), source_count) }
    };

    // check is_activated
    let is_activated = h.is_activated();
    d.add_bool("is_activated", is_activated);
    if !is_activated {
        d.update_level(DIAGNOSTIC_WARN, "Node is not activated.");
        return SM_NOT_ACTIVATED;
    }

    // calculate initial pose (interpolate + pop_old, Rust-owned buffer)
    let Some(interp) = h.interpolate_initial_pose(sensor_stamp_ns) else {
        d.add_bool("is_succeed_interpolate_initial_pose", false);
        d.update_level(
            DIAGNOSTIC_WARN,
            "Couldn't interpolate pose. Please verify that (1) the initial pose topic (primarily \
             come from the EKF) is being published, and (2) the timestamps of the sensor PCD \
             messages and pose messages are synchronized correctly.",
        );
        return SM_INTERPOLATE_FAILED;
    };
    d.add_bool("is_succeed_interpolate_initial_pose", true);
    let initial_pose_matrix = pose_to_matrix4(&interp.position, &interp.orientation);

    // regularization: interpolate the Rust-owned buffer and set it on the engine (else disable).
    if h.params.regularization_enable {
        eng.set_regularization(0.0, 0.0, 0.0);
        if let Some(reg) = h.interpolate_regularization(sensor_stamp_ns) {
            eng.set_regularization(
                reg.position[0] as f32,
                reg.position[1] as f32,
                prm.regularization_scale_factor as f32,
            );
        }
    }

    // Warn if the lidar has gone out of the map range (soft; no gate).
    if h.map_update_out_of_range(
        [interp.position[0], interp.position[1]],
        prm.lidar_radius,
        prm.map_radius,
    ) {
        d.update_level(DIAGNOSTIC_WARN, "Lidar has gone out of the map range");
        ho.log(LOG_WARN, "Lidar has gone out of the map range");
    }

    // check is_set_map_points
    let is_set_map_points = eng.has_target();
    d.add_bool("is_set_map_points", is_set_map_points);
    if !is_set_map_points {
        d.update_level(DIAGNOSTIC_WARN, "Map points is not set.");
        return SM_MAP_NOT_SET;
    }

    // perform ndt scan matching (align + oscillation + convergence verdict)
    let convergence_params = ConvergenceParams {
        converged_param_type: h.params.converged_param_type,
        converged_param_transform_probability: h.params.converged_param_transform_probability,
        converged_param_nearest_voxel_transformation_likelihood: h
            .params
            .converged_param_nearest_voxel_transformation_likelihood,
    };
    // Cross-call scratch reads (`result`/`score_arrays` after `run_align`) are sound here: this
    // module is std-only, so all three hit the same thread-local scratch on the calling thread.
    let outcome = run_align(eng, &initial_pose_matrix, source, &convergence_params);
    let result = eng.result();
    let (tp_array, nvtl_array) = eng.score_arrays();
    let verdict = outcome.verdict;

    // check iteration_num
    d.add_i64("iteration_num", i64::from(result.iteration_num));
    if !verdict.is_ok_iteration_num {
        d.update_level(
            DIAGNOSTIC_WARN,
            &alloc::format!(
                "The number of iterations has reached its upper limit. The number of iterations: \
                 {}, Limit: {}.",
                result.iteration_num,
                outcome.max_iterations
            ),
        );
    }

    // check local_optimal_solution_oscillation_num
    d.add_i64(
        "local_optimal_solution_oscillation_num",
        i64::from(outcome.oscillation_num),
    );
    if verdict.is_local_optimal_solution_oscillation {
        d.update_level(
            DIAGNOSTIC_WARN,
            "There is a possibility of oscillation in a local minimum",
        );
    }

    // check score
    d.add_f64(
        "transform_probability",
        f64::from(result.transform_probability),
    );
    d.add_f64(
        "nearest_voxel_transformation_likelihood",
        f64::from(result.nearest_voxel_likelihood),
    );
    if !verdict.valid_param_type {
        d.update_level(
            DIAGNOSTIC_ERROR,
            "Unknown converged param type. Please check `score_estimation.converged_param_type`",
        );
        return SM_INVALID_PARAM_TYPE;
    }

    // check score diff (diagnostic only; the arrays stay Rust-internal)
    let want_len = i32::try_from(tp_array.len()).unwrap_or(i32::MAX);
    if want_len != result.iteration_num.saturating_add(1) {
        d.update_level(
            DIAGNOSTIC_WARN,
            &alloc::format!(
                "transform_probability_array size is not equal to iteration_num + 1. \
                 transform_probability_array size: {}, iteration_num: {}",
                tp_array.len(),
                result.iteration_num
            ),
        );
    } else if let (Some(&front), Some(&back)) = (tp_array.first(), tp_array.last()) {
        d.add_f64("transform_probability_diff", f64::from(back - front));
        d.add_f64("transform_probability_before", f64::from(front));
    }
    let nvtl_len = i32::try_from(nvtl_array.len()).unwrap_or(i32::MAX);
    if nvtl_len != result.iteration_num.saturating_add(1) {
        d.update_level(
            DIAGNOSTIC_WARN,
            &alloc::format!(
                "nearest_voxel_transformation_likelihood_array size is not equal to \
                 iteration_num + 1. nearest_voxel_transformation_likelihood_array size: {}, \
                 iteration_num: {}",
                nvtl_array.len(),
                result.iteration_num
            ),
        );
    } else if let (Some(&front), Some(&back)) = (nvtl_array.first(), nvtl_array.last()) {
        d.add_f64(
            "nearest_voxel_transformation_likelihood_diff",
            f64::from(back - front),
        );
        d.add_f64(
            "nearest_voxel_transformation_likelihood_before",
            f64::from(front),
        );
    }

    if !verdict.is_ok_score {
        let msg = alloc::format!(
            "Score is below the threshold. Score: {}, Threshold: {}",
            verdict.score,
            verdict.score_threshold
        );
        d.update_level(DIAGNOSTIC_WARN, &msg);
        ho.log(LOG_WARN, &msg);
    }

    // covariance estimation (rotate + dispatch + scale + adjust, against the live map)
    let cov = eng.with_scratch(|scr| {
        estimate_pose_covariance(
            eng,
            &outcome.pose,
            &matrix6_to_row_major(&result.hessian),
            &initial_pose_matrix,
            source,
            &h.params.initial_pose_offset_model_x,
            &h.params.initial_pose_offset_model_y,
            &CovEstimationParams {
                estimation_type: h.params.covariance_estimation_type,
                scale_factor: h.params.covariance_scale_factor,
                temperature: h.params.covariance_temperature,
                main_nvtl: result.nearest_voxel_likelihood,
                output_pose_covariance: h.params.output_pose_covariance,
                map_to_base_link_rot3x3: map_to_base_link_rot3x3(&outcome.pose),
            },
            scr,
        )
    });

    // check distance_initial_to_result (diagnostic only)
    let dx = interp.position[0] - f64::from(outcome.pose[(0, 3)]);
    let dy = interp.position[1] - f64::from(outcome.pose[(1, 3)]);
    let dz = interp.position[2] - f64::from(outcome.pose[(2, 3)]);
    let distance_initial_to_result = libm::sqrt(dx * dx + dy * dy + dz * dz);
    d.add_f64("distance_initial_to_result", distance_initial_to_result);
    if distance_initial_to_result > prm.initial_to_result_distance_tolerance_m {
        d.update_level(
            DIAGNOSTIC_WARN,
            &alloc::format!(
                "distance_initial_to_result is too large ({distance_initial_to_result} [m])."
            ),
        );
    }

    // Publish results through the host (Phase 5 sub-slice 3): the C++ trampolines build the ROS
    // messages + markers and know the frame_ids. `exe_time` + the cloud publishers stay C++ (sub-slice 4).
    let result_pose_aw = matrix4_to_aw_pose(&outcome.pose);
    let initial_pose_aw = AwPose {
        position: interp.position,
        orientation: interp.orientation,
    };
    ho.publish_pose(
        AwPoseTopic::InitialPoseWithCovariance,
        sensor_stamp_ns,
        &initial_pose_aw,
        Some(&interp.covariance),
    );
    ho.publish_float32(
        AwFloat32Topic::TransformProbability,
        sensor_stamp_ns,
        result.transform_probability,
    );
    ho.publish_float32(
        AwFloat32Topic::NearestVoxelTransformationLikelihood,
        sensor_stamp_ns,
        result.nearest_voxel_likelihood,
    );
    ho.publish_int32(
        AwInt32Topic::IterationNum,
        sensor_stamp_ns,
        result.iteration_num,
    );
    ho.publish_tf(sensor_stamp_ns, &result_pose_aw);
    // ndt_pose / ndt_pose_with_covariance publish only on convergence (the C++ `publish_pose` gate).
    if verdict.is_converged {
        ho.publish_pose(AwPoseTopic::NdtPose, sensor_stamp_ns, &result_pose_aw, None);
        ho.publish_pose(
            AwPoseTopic::NdtPoseWithCovariance,
            sensor_stamp_ns,
            &result_pose_aw,
            Some(&cov.ndt_covariance),
        );
    }
    ho.publish_marker(
        sensor_stamp_ns,
        &aw_poses(&result.transformation_array),
        outcome.max_iterations,
    );
    ho.publish_initial_to_result(
        sensor_stamp_ns,
        &result_pose_aw,
        &initial_pose_aw,
        &interp.old.position,
        &interp.new_entry.position,
    );
    // covariance debug pose arrays (kind 1 ⇒ both, 2 ⇒ initial only; 0 ⇒ none).
    if cov.publish_kind == 1 {
        ho.publish_pose_array(
            AwPoseArrayTopic::MultiNdtPose,
            sensor_stamp_ns,
            &aw_poses(&cov.multi_ndt_result_poses),
        );
        ho.publish_pose_array(
            AwPoseArrayTopic::MultiInitialPose,
            sensor_stamp_ns,
            &aw_poses(&cov.multi_initial_poses),
        );
    } else if cov.publish_kind == 2 {
        ho.publish_pose_array(
            AwPoseArrayTopic::MultiInitialPose,
            sensor_stamp_ns,
            &aw_poses(&cov.multi_initial_poses),
        );
    }

    // Cloud publishers (Phase 5 sub-slice 4): transform base_link cloud to map, publish aligned
    // clouds, score-color input, and optional no-ground score scalars through the host.
    let points_map = transform_cloud_to_map(source, &outcome.pose);
    publish_cloud_outputs(eng, ho, prm, sensor_stamp_ns, &outcome.pose, &points_map);

    // Return the minimal state the C++ shell still needs for `skipping_publish_num`.
    o.result_pose = matrix4_to_row_major(&outcome.pose);
    o.is_converged = verdict.is_converged;
    SM_MATCHED
}

/// The full Rust sensor callback for the `NDT_USE_RUST` C++ shell: prepare the borrowed
/// `PointCloud2View`, run the scan-match middle, publish all Rust-owned outputs through `host`, and
/// return whether the match converged via `*out_is_converged`.
///
/// # Safety
/// All pointers must be valid/live for the call (or null → `SM_INVALID`). `view.data` addresses
/// `view.data_len` readable bytes; all host/diagnostics function pointers remain valid for the call.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_on_sensor_points(
    handle: *const NdtScanMatcherRs,
    engine: *const NdtEngine,
    host: *const AwHost,
    diag: *const Diagnostics,
    params: *const AwSensorPointsMatchParams,
    view: *const AwPointCloud2View,
    out_is_converged: *mut bool,
) -> i32 {
    if view.is_null() || out_is_converged.is_null() {
        return SM_INVALID;
    }
    // SAFETY: `view` is non-null per the check; caller guarantees a valid borrowed view.
    let v = unsafe { &*view };
    let width = usize::try_from(v.width).unwrap_or(0);
    let height = usize::try_from(v.height).unwrap_or(0);
    let max_points = width.saturating_mul(height);
    let mut baselink_xyz = alloc::vec![0.0_f32; max_points.saturating_mul(3)];
    let mut count = 0usize;
    // SAFETY: all pointers and out buffers are valid for this call per this function's contract.
    let prep = unsafe {
        autoware_ndt_scan_matcher_rs_node_on_sensor_points_prepare(
            handle,
            host,
            diag,
            view,
            baselink_xyz.as_mut_ptr(),
            baselink_xyz.len(),
            &raw mut count,
        )
    };
    if prep != SP_PREPARED {
        return prep;
    }
    let mut out = AwSensorPointsMatchOutput {
        result_pose: [0.0; 16],
        is_converged: false,
    };
    // SAFETY: `baselink_xyz` owns `3 * count` initialized f32 values written by the prepare step.
    let status = unsafe {
        autoware_ndt_scan_matcher_rs_node_on_sensor_points_match(
            handle,
            engine,
            host,
            diag,
            params,
            v.stamp_ns,
            baselink_xyz.as_ptr(),
            count,
            &raw mut out,
        )
    };
    if status == SM_MATCHED {
        // SAFETY: out pointer is non-null per the check and points to writable bool.
        unsafe {
            *out_is_converged = out.is_converged;
        }
    }
    status
}

/// ns → seconds (see the equivalent in `pose_buffer`; magnitudes are within f64's exact-int range).
#[expect(
    clippy::as_conversions,
    clippy::cast_precision_loss,
    reason = "ns->seconds; realtime stamp magnitudes are well within f64's exact-integer range"
)]
fn ns_to_sec(ns: i64) -> f64 {
    (ns as f64) * 1e-9
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::arithmetic_side_effects,
    clippy::as_conversions,
    clippy::cast_precision_loss,
    clippy::allow_attributes,
    reason = "test code: exact-equal asserts on deterministic math + fixed-size index reads"
)]
mod tests {
    use super::*;

    #[test]
    fn transform_point_applies_row_major_4x4() {
        // Rotate +90° about Z then translate by (10, 20, 30): (1,0,0) -> (0+10, 1+20, 0+30).
        #[rustfmt::skip]
        let m: [f32; 16] = [
            0.0, -1.0, 0.0, 10.0,
            1.0,  0.0, 0.0, 20.0,
            0.0,  0.0, 1.0, 30.0,
            0.0,  0.0, 0.0, 1.0,
        ];
        let p = transform_point(&m, 1.0, 0.0, 0.0);
        assert_eq!(p, [10.0, 21.0, 30.0]);
    }

    #[test]
    fn read_f32_le_be_and_bounds() {
        let v = 1.5_f32;
        let le = v.to_le_bytes();
        assert_eq!(read_f32(&le, 0, false), Some(1.5));
        let be = v.to_be_bytes();
        assert_eq!(read_f32(&be, 0, true), Some(1.5));
        // Out of bounds → None (no panic).
        assert_eq!(read_f32(&le, 2, false), None);
        assert_eq!(read_f32(&le, usize::MAX, false), None);
    }

    #[test]
    fn ns_to_sec_matches() {
        assert_eq!(ns_to_sec(1_500_000_000), 1.5);
        assert_eq!(ns_to_sec(0), 0.0);
    }

    #[test]
    fn matrix4_to_aw_pose_identity_and_quarter_turn() {
        // Identity → origin position + identity quaternion [0,0,0,1].
        let p0 = matrix4_to_aw_pose(&Matrix4::<f32>::identity());
        assert!((p0.position[0]).abs() < 1e-9);
        assert!((p0.orientation[3] - 1.0).abs() < 1e-9);
        // Round-trip a translation + 90° yaw through pose_to_matrix4 → matrix4_to_aw_pose.
        let m = pose_to_matrix4(&[1.0, 2.0, 3.0], &[0.0, 0.0, 0.707_106_77, 0.707_106_77]);
        let p = matrix4_to_aw_pose(&m);
        assert!((p.position[0] - 1.0).abs() < 1e-5);
        assert!((p.position[1] - 2.0).abs() < 1e-5);
        assert!((p.position[2] - 3.0).abs() < 1e-5);
        // 90° about z: quaternion z ≈ w ≈ sin/cos(45°); x, y ≈ 0.
        assert!((p.orientation[0]).abs() < 1e-5);
        assert!((p.orientation[1]).abs() < 1e-5);
        assert!((p.orientation[2] - 0.707_106_77).abs() < 1e-5);
        assert!((p.orientation[3] - 0.707_106_77).abs() < 1e-5);
    }

    #[test]
    fn pose_to_matrix4_identity_rotation_places_translation() {
        // Identity quaternion [x,y,z,w] = [0,0,0,1] + translation (1,2,3).
        let m = pose_to_matrix4(&[1.0, 2.0, 3.0], &[0.0, 0.0, 0.0, 1.0]);
        assert!((m[(0, 3)] - 1.0).abs() < 1e-6);
        assert!((m[(1, 3)] - 2.0).abs() < 1e-6);
        assert!((m[(2, 3)] - 3.0).abs() < 1e-6);
        // Upper-left 3x3 is identity; bottom row is [0,0,0,1].
        for r in 0..3 {
            for c in 0..3 {
                let want = if r == c { 1.0 } else { 0.0 };
                assert!((m[(r, c)] - want).abs() < 1e-6, "({r},{c})");
            }
        }
        assert!((m[(3, 3)] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn matrix4_to_row_major_flattens_row_major() {
        // Distinct entries so any transpose bug is visible: m[(r,c)] = 4r + c.
        let mut m = Matrix4::<f32>::zeros();
        for r in 0..4 {
            for c in 0..4 {
                m[(r, c)] = ((r * 4) + c) as f32;
            }
        }
        let flat = matrix4_to_row_major(&m);
        for (i, &v) in flat.iter().enumerate() {
            assert_eq!(v, i as f32, "index {i}");
        }
    }

    #[test]
    fn matrix6_to_row_major_flattens_row_major() {
        let mut m = Matrix6::<f64>::zeros();
        for r in 0..6 {
            for c in 0..6 {
                m[(r, c)] = ((r * 6) + c) as f64;
            }
        }
        let flat = matrix6_to_row_major(&m);
        for (i, &v) in flat.iter().enumerate() {
            assert_eq!(v, i as f64, "index {i}");
        }
    }

    #[test]
    fn map_to_base_link_rot3x3_of_identity_is_identity() {
        let rot = map_to_base_link_rot3x3(&Matrix4::<f32>::identity());
        let want = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        for (i, (&g, &w)) in rot.iter().zip(want.iter()).enumerate() {
            assert!((g - w).abs() < 1e-12, "index {i}: {g} vs {w}");
        }
    }

    #[test]
    fn map_to_base_link_rot3x3_quarter_turn_about_z() {
        // 90 deg CCW about z: R = [[0,-1,0],[1,0,0],[0,0,1]] (row-major).
        let mut m = Matrix4::<f32>::identity();
        m[(0, 0)] = 0.0;
        m[(0, 1)] = -1.0;
        m[(1, 0)] = 1.0;
        m[(1, 1)] = 0.0;
        let rot = map_to_base_link_rot3x3(&m);
        let want = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        for (i, (&g, &w)) in rot.iter().zip(want.iter()).enumerate() {
            assert!((g - w).abs() < 1e-9, "index {i}: {g} vs {w}");
        }
    }
}
