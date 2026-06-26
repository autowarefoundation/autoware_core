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

// no_std for the awkernel/Track B build; `std` (default) for dev and the ROS-node build.
// Test builds always use std (the test harness + `Vec`/etc. need it), regardless of features.
#![cfg_attr(not(any(test, feature = "std")), no_std)]

// Heap types (Vec/Box/BTreeMap) for the engine data structures. The allocator is provided by std
// (host) or the kernel (awkernel); the no_std rlib defers it to the final binary.
extern crate alloc;

// Public API: the pure ports are reused by the Track B engine and exercised by unit tests,
// independently of whether the `ros` FFI shims are built.
pub mod covariance;
pub mod derivatives;
pub mod helper;
mod kdtree;
pub mod ndt;
pub mod transform;
pub mod voxel_grid;

// rosidl-generated geometry_msgs C structs (bindgen). ROS-node build only; the no_std/awkernel
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
    clippy::arithmetic_side_effects
)]
mod ros_msgs {
    include!(concat!(env!("OUT_DIR"), "/ros_msgs.rs"));
}

#[must_use]
pub fn add(left: u64, right: u64) -> u64 {
    left.saturating_add(right)
}

/// C ABI entry point for the C++ side of `autoware_ndt_scan_matcher`.
///
/// Both arguments are passed by value and the return is a plain `u64`, so there
/// are no pointers or lifetimes crossing the boundary to validate.
#[allow(unsafe_code)]
#[unsafe(no_mangle)]
pub extern "C" fn autoware_ndt_scan_matcher_rs_add(left: u64, right: u64) -> u64 {
    add(left, right)
}

/// Rotate the 3x3 position block of a 6x6 row-major pose covariance: `out = R * C * R^T`.
///
/// # Safety
/// `src_cov` and `out_cov` must each point to a readable/writable array of 36 `f64`, and
/// `rot` to a readable array of 9 `f64` (row-major 3x3). All pointers must be non-null and
/// suitably aligned for `f64`. Does nothing if any pointer is null.
#[allow(unsafe_code)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_rotate_covariance(
    src_cov: *const f64,
    rot: *const f64,
    out_cov: *mut f64,
) {
    if src_cov.is_null() || rot.is_null() || out_cov.is_null() {
        return;
    }
    // SAFETY: the caller guarantees `src_cov`/`rot` point to valid, aligned arrays of 36 and
    // 9 `f64` respectively (see the `# Safety` contract); both are read-only here.
    let (src, rotation) = unsafe { (&*src_cov.cast::<[f64; 36]>(), &*rot.cast::<[f64; 9]>()) };
    let result = helper::rotate_covariance(src, rotation);
    // SAFETY: `out_cov` is a valid, aligned, writable array of 36 `f64` per the contract.
    unsafe {
        *out_cov.cast::<[f64; 36]>() = result;
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
#[allow(unsafe_code)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_count_oscillation(
    poses: *const core::ffi::c_void,
    num_poses: usize,
) -> i32 {
    if poses.is_null() || num_poses == 0 {
        return 0;
    }
    // SAFETY: per the `# Safety` contract, `poses` points to `num_poses` valid, aligned Pose values.
    let slice = unsafe {
        core::slice::from_raw_parts(
            poses.cast::<ros_msgs::geometry_msgs__msg__Pose>(),
            num_poses,
        )
    };
    helper::count_oscillation_poses(slice)
}

#[cfg(test)]
// tests call the `extern "C"` shims via raw pointers (unsafe) and assert exact equality between a
// shim and the pure fn it delegates to (identical ops → bit-identical, so float_cmp is intended).
#[allow(unsafe_code, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        assert_eq!(add(2, 2), 4);
        assert_eq!(autoware_ndt_scan_matcher_rs_add(2, 2), 4);
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
