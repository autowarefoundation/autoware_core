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

//! Rust port of the Autoware `autoware_ndt_scan_matcher` NDT (Normal Distributions Transform)
//! localization core.
//!
//! The crate is `no_std`-capable (heap only, via `alloc`) and is linked into the C++
//! `autoware_ndt_scan_matcher` package over a C ABI, while remaining usable as a plain Rust library
//! (`cargo test`, downstream `rlib` consumers). Its numeric kernels mirror the C++ `Matrix4f` /
//! `MultiVoxelGridCovariance` pipeline.
//!
//! # Entry points
//!
//! - [`engine::NdtEngine`] — the persistent, `&self`-only NDT handle: load a target map, build the
//!   kd-tree, then align sensor clouds. This is the primary API.
//! - [`scan_matcher::ScanMatcher`] — portable node orchestration over [`engine::NdtEngine`] and the
//!   [`host`] ports (map update + single-scan match); reusable under ROS, a bare-metal kernel, or an
//!   async runtime.
//! - [`ndt::align`] — the RT-critical, WCET-bounded alignment kernel the engine drives.
//! - [`tpe::TreeStructuredParzenEstimator`] — the align-service pose-search sampler.
//!
//! Pose guesses and result matrices use [`nalgebra`] types (`Matrix4<f32>`, `Matrix6<f64>`). The
//! exact `nalgebra` version this crate is built against is re-exported as [`nalgebra`] so callers can
//! construct those matrices without independently pinning a (possibly mismatched) version.
//!
//! # Cargo features
//!
//! | Feature | Default | Effect |
//! |---|---|---|
//! | `std` | yes | Host/ROS build: lock-free `ArcSwap` engine state + thread-local align scratch; the engine is `Sync`. |
//! | `parallel` | yes | rayon-backed derivative reduction (implies `std`); bit-identical to serial, a pure throughput option. |
//! | `mt` | no | Multi-core `no_std` (kernel): `awkernel_sync` mutex cells + **caller-owned** [`engine::MatchScratch`] (the implicit-scratch align API is compiled out); engine is `Sync`. Ignored when `std` is on. |
//! | `ros` | no | rosidl bindgen bindings + `Pose`-pointer FFI shims. Independent of `std`. |
//!
//! With **no** features (`--no-default-features`) the engine is a single-core `no_std` build
//! (`RefCell` cells, engine-owned scratch, intentionally `!Sync`). See the [`engine`] module docs for
//! the full interior-mutability matrix.
//!
//! # Example
//!
//! Load a one-tile target map, build the kd-tree, and align a source cloud from an identity guess:
//!
//! ```
//! use autoware_ndt_scan_matcher_rs::engine::NdtEngine;
//! use autoware_ndt_scan_matcher_rs::nalgebra::Matrix4;
//!
//! // Empty engine: 2.0 m voxels; `MultiVoxelGridCovariance` defaults (min 6 points / eig 0.01).
//! let engine = NdtEngine::new(2.0, 6, 0.01);
//!
//! // Register a target map tile (id 0) and build the kd-tree over the voxel centroids.
//! let target: Vec<[f32; 3]> = (0u8..64).map(|i| [f32::from(i) * 0.05, 0.0, 0.0]).collect();
//! engine.add_target(&target, 0);
//! engine.create_kdtree();
//! assert!(engine.has_target());
//!
//! // Align a source cloud from an identity initial guess, then read the result back.
//! let source = target.clone();
//! engine.align(&Matrix4::identity(), &source);
//! let result = engine.result();
//! assert!(result.iteration_num >= 0);
//! ```

// no_std for the Track B build; `std` (default) for dev and the ROS-node build. The
// `mt` feature makes the no_std engine multi-core-safe (awkernel_sync cells + caller-owned
// `MatchScratch`); without it the no_std build is single-core (`RefCell`, `!Sync`).
// Test builds always use std (the test harness + `Vec`/etc. need it), regardless of features.
#![cfg_attr(not(any(test, feature = "std")), no_std)]

// Heap types (Vec/Box/BTreeMap) for the engine data structures. The allocator is provided by std
// (host) or the kernel; the no_std rlib defers it to the final binary.
extern crate alloc;

// Public API: the pure ports are reused by the Track B engine and exercised by unit tests,
// independently of whether the `ros` FFI shims are built.
// The NDT convergence decision — pure, no_std, reused by the std-gated `node` FFI and the portable
// `scan_matcher`.
pub mod convergence;
pub mod cov_estimate;
pub mod covariance;
pub mod derivatives;
pub mod engine;
// Panic-safe C ABI boundary helpers (catch_unwind) + AwStatus/Error for the object-level FFI; std-only.
#[cfg(feature = "std")]
pub mod ffi;
// ROS side-effects host vtable (AwHost: clock/log/TF) — the C-ABI adapter for the portable Host seam.
#[cfg(feature = "std")]
pub mod ffi_host;
// Audited C-ABI pointer helpers + guard macros — the single home for raw-pointer dereferences at
// the FFI boundary. core + alloc only (NOT std-gated: many FFI fns exist in the no_std build).
pub mod ffi_ptr;
pub mod helper;
// Portable node orchestration: the `Host` port traits + the `no_std` scan matcher over the engine
// (reusable on ROS / bare-metal / the Tokio example).
pub mod host;
mod kdtree;
pub mod ndt;
// no_std port of autoware_localization_util::SmartPoseBuffer (time-ordered pose interpolation buffer);
// reused by the node handle's Rust-owned pose buffers.
pub mod pose_buffer;
// ROS-node callback glue; std-only (not part of the no_std engine path).
#[cfg(feature = "std")]
pub mod node;
// Opaque object-level node handle (NdtScanMatcherRs) + AwNdtParams; the roadmap foundation. std-only.
#[cfg(feature = "std")]
pub mod node_handle;
// ROS map-update glue: drives the portable `apply_map_update` (the `MapSource` port) from C++; std-only.
#[cfg(feature = "std")]
pub mod node_map_update;
// ROS align-service deterministic gate/response decisions; std-only node glue.
#[cfg(feature = "std")]
pub mod node_align_service;
pub mod scan_matcher;
// ROS sensor-callback prologue (decode/TF/transform/validation → base_link cloud); std-only.
#[cfg(feature = "std")]
pub mod sensor_points;
pub mod tpe;
pub mod transform;
pub mod voxel_grid;

// rosidl-generated geometry_msgs C structs (bindgen). ROS-node build only; the no_std
// build leaves `ros` off. bindgen output is allow-listed (its lint profile differs from ours).
#[cfg(feature = "ros")]
#[allow(
    non_upper_case_globals,
    non_camel_case_types,
    non_snake_case,
    dead_code,
    unsafe_code,
    clippy::all,
    clippy::pedantic,
    // bindgen's layout tests use `as` casts / indexing that our restriction lints would flag.
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::indexing_slicing,
    clippy::arithmetic_side_effects,
    clippy::allow_attributes,
    reason = "bindgen-generated"
)]
mod ros_msgs {
    include!(concat!(env!("OUT_DIR"), "/ros_msgs.rs"));
}

use crate::ffi_ptr::ffi_ref;

/// Re-export of the exact [`nalgebra`] version this crate is built against.
///
/// The engine's pose API is expressed in `nalgebra` matrices (`Matrix4<f32>` guesses/results,
/// `Matrix6<f64>` Hessians/covariances). Construct and read them through this re-export so the type
/// identities match the engine's — a locally pinned `nalgebra` of a different version would be a
/// distinct, incompatible type.
pub use nalgebra;

/// Saturating `left + right` — a trivial build/link smoke test mirrored by the
/// [`autoware_ndt_scan_matcher_rs_add`] C ABI shim (used to confirm the staticlib links).
///
/// # Arguments
/// * `left`, `right` — the addends; the sum saturates at [`u64::MAX`] instead of overflowing.
#[must_use]
pub fn add(left: u64, right: u64) -> u64 {
    left.saturating_add(right)
}

/// C ABI entry point for the C++ side of `autoware_ndt_scan_matcher`.
///
/// Both arguments are passed by value and the return is a plain `u64`, so there
/// are no pointers or lifetimes crossing the boundary to validate.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub extern "C" fn autoware_ndt_scan_matcher_rs_add(left: u64, right: u64) -> u64 {
    add(left, right)
}

/// Size rayon's process-global worker pool used by the `parallel` derivative reduction.
///
/// The parallel backend runs on rayon's process-global thread pool; this sizes that pool to
/// `num_threads` workers. It is orthogonal to [`ndt::NdtParams::num_threads`], which only selects
/// serial vs parallel (`> 1`). If never called, the pool defaults to the number of logical CPUs
/// (or `RAYON_NUM_THREADS`, if set).
///
/// Best-effort and idempotent — call it **once, early** (before any align). Returns `true` iff this
/// call initialized the pool; `false` if the pool was already initialized (a previous call, or a
/// prior parallel run initialized it lazily) or `num_threads == 0`. Never panics.
///
/// # Arguments
/// * `num_threads` — desired global worker count; `0` is a no-op returning `false`.
#[cfg(feature = "parallel")]
#[must_use]
pub fn init_thread_pool(num_threads: usize) -> bool {
    num_threads != 0
        && rayon::ThreadPoolBuilder::new()
            .num_threads(num_threads)
            .build_global()
            .is_ok()
}

/// C ABI entry point for [`init_thread_pool`]. Passed by value; no pointers cross the boundary.
/// Returns `true` iff this call initialized the pool (see [`init_thread_pool`]).
#[cfg(feature = "parallel")]
#[expect(
    unsafe_code,
    reason = "C ABI boundary; no pointers cross here (value in, bool out)"
)]
#[unsafe(no_mangle)]
pub extern "C" fn autoware_ndt_scan_matcher_rs_init_thread_pool(num_threads: usize) -> bool {
    init_thread_pool(num_threads)
}

/// Rotate the 3x3 position block of a 6x6 row-major pose covariance: `out = R * C * R^T`.
///
/// # Safety
/// `src_cov` and `out_cov` must each point to a readable/writable array of 36 `f64`, and
/// `rot` to a readable array of 9 `f64` (row-major 3x3). All pointers must be non-null and
/// suitably aligned for `f64`. Does nothing if any pointer is null.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_rotate_covariance(
    src_cov: *const f64,
    rot: *const f64,
    out_cov: *mut f64,
) {
    let src = ffi_ref!(src_cov.cast::<[f64; 36]>(), else return);
    let rotation = ffi_ref!(rot.cast::<[f64; 9]>(), else return);
    let result = helper::rotate_covariance(src, rotation);
    // SAFETY: `out_cov` is a valid, aligned, writable [f64; 36] per the contract, audited in ffi_ptr.
    unsafe {
        ffi_ptr::write_out(out_cov.cast::<[f64; 36]>(), result);
    }
}

/// Count the maximum consecutive direction inversions over a pose trajectory (zero-copy).
///
/// `poses` points to `num_poses` contiguous `geometry_msgs::msg::Pose`; the positions are read
/// in place (no flattening). Only `position.{x,y,z}` is used.
///
/// # Safety
/// When `num_poses > 0`, `poses` must point to `num_poses` contiguous, aligned, initialized
/// `geometry_msgs__msg__Pose` (the layout is asserted on the C++ side). Returns 0 if `poses` is
/// null or `num_poses` is 0.
#[cfg(feature = "ros")]
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_count_oscillation(
    poses: *const core::ffi::c_void,
    num_poses: usize,
) -> i32 {
    if num_poses == 0 {
        return 0;
    }
    // SAFETY: per the `# Safety` contract, `poses` points to `num_poses` valid, aligned Pose values.
    let slice = crate::ffi_ptr::ffi_slice!(poses, num_poses, ros_msgs::geometry_msgs__msg__Pose, else return 0);
    helper::count_oscillation_poses(slice)
}

#[cfg(test)]
// tests call the `extern "C"` shims via raw pointers (unsafe) and assert exact equality between a
// shim and the pure fn it delegates to (identical ops → bit-identical, so float_cmp is intended).
#[allow(
    unsafe_code,
    clippy::float_cmp,
    clippy::allow_attributes,
    reason = "test code"
)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        assert_eq!(add(2, 2), 4);
        assert_eq!(autoware_ndt_scan_matcher_rs_add(2, 2), 4);
    }

    #[cfg(feature = "parallel")]
    #[test]
    fn init_thread_pool_zero_is_noop_and_pool_is_usable() {
        // Zero threads is always a deterministic no-op.
        assert!(!init_thread_pool(0), "0 threads must be a no-op");
        // Best-effort: `sized` is true only if THIS call initialized the process-global pool — a
        // prior test in the same binary may have already done so — so assert the pool is usable
        // rather than the flag value.
        let sized = init_thread_pool(4);
        assert!(
            rayon::current_num_threads() >= 1,
            "global pool usable after init (sized={sized})"
        );
    }

    #[test]
    fn rotate_covariance_ffi_matches_pure() {
        let mut src = [0.0_f64; 36];
        src[0] = 4.0;
        src[7] = 9.0;
        src[14] = 16.0;
        let rot = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];

        let mut out = [0.0_f64; 36];
        // SAFETY: all three pointers are valid, aligned arrays of the documented length.
        unsafe {
            autoware_ndt_scan_matcher_rs_rotate_covariance(
                src.as_ptr(),
                rot.as_ptr(),
                out.as_mut_ptr(),
            );
        }
        assert_eq!(out, helper::rotate_covariance(&src, &rot));
    }

    #[test]
    fn rotate_covariance_ffi_null_is_noop() {
        let mut out = [1.0_f64; 36];
        let rot = [0.0_f64; 9];
        // SAFETY: a null `src_cov` must make the shim return without touching `out`.
        unsafe {
            autoware_ndt_scan_matcher_rs_rotate_covariance(
                core::ptr::null(),
                rot.as_ptr(),
                out.as_mut_ptr(),
            );
        }
        assert_eq!(out, [1.0_f64; 36]);
    }

    #[cfg(feature = "ros")]
    #[test]
    fn count_oscillation_ffi_matches_pure() {
        let xs = [0.0_f64, 1.0, 0.0, 1.0, 0.0];
        let positions: Vec<[f64; 3]> = xs.iter().map(|&x| [x, 0.0, 0.0]).collect();
        let poses: Vec<ros_msgs::geometry_msgs__msg__Pose> = xs
            .iter()
            .map(|&x| {
                let mut p = ros_msgs::geometry_msgs__msg__Pose::default();
                p.position.x = x;
                p
            })
            .collect();

        // SAFETY: `poses` is a valid contiguous array of `poses.len()` Pose values.
        let via_ffi = unsafe {
            autoware_ndt_scan_matcher_rs_count_oscillation(poses.as_ptr().cast(), poses.len())
        };
        assert_eq!(via_ffi, helper::count_oscillation(&positions));
        assert_eq!(via_ffi, 3);

        // SAFETY: null pointer must yield 0.
        assert_eq!(
            unsafe { autoware_ndt_scan_matcher_rs_count_oscillation(core::ptr::null(), 0) },
            0
        );
    }
}
