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

mod helper;

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

/// Count the maximum consecutive direction inversions over a sequence of positions.
///
/// `positions_xyz` is a flat buffer of `3 * num_poses` `f64` laid out as `x, y, z` per pose.
///
/// # Safety
/// When `num_poses > 0`, `positions_xyz` must point to `3 * num_poses` readable, aligned `f64`.
/// Returns 0 if the pointer is null, if `num_poses` is 0, or if `3 * num_poses` overflows.
#[allow(unsafe_code)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_count_oscillation(
    positions_xyz: *const f64,
    num_poses: usize,
) -> i32 {
    let Some(len) = num_poses.checked_mul(3) else {
        return 0;
    };
    if positions_xyz.is_null() || len == 0 {
        return 0;
    }
    // SAFETY: per the `# Safety` contract, `positions_xyz` points to `len` valid, aligned `f64`.
    let flat: &[f64] = unsafe { core::slice::from_raw_parts(positions_xyz, len) };
    let positions: Vec<[f64; 3]> = flat
        .chunks_exact(3)
        .filter_map(|chunk| <[f64; 3]>::try_from(chunk).ok())
        .collect();
    helper::count_oscillation(&positions)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
