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

use crate::ffi_host::{AwHost, LOG_ERROR};
use crate::node::{DIAGNOSTIC_WARN, Diagnostics};
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
        ho.log(LOG_ERROR, "Failed to look up the sensor→base_link transform.");
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
    SP_PREPARED
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
}
