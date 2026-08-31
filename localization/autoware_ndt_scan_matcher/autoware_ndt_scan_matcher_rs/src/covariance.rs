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

//! Pure ports of the `pclomp` covariance helpers from `src/ndt_omp/estimate_covariance.cpp`.
//!
//! All public fns take/return plain arrays/slices (row-major for matrices) so the FFI boundary is
//! unambiguous; nalgebra is used internally. `no_std`-clean (fixed-size matrices, `libm`).
//! `test_estimate_covariance` is the differential oracle.

// nalgebra's fixed-size f64 matrix operators (+, *, unary -) are the float-math domain the
// integer-overflow lint targets; they cannot integer-overflow.
// Numeric kernel. Suppressions are scoped per-function (no module-wide `#![allow]`):
// arithmetic_side_effects = nalgebra f64 float math (cannot integer-overflow); doc_markdown =
// domain terms (base_link, etc.) that are not code.

use nalgebra::{Matrix2, Vector2};

use crate::ffi_ptr::{self, ffi_mut_slice, ffi_ref, ffi_slice};

fn mat2(row_major: &[f64; 4]) -> Matrix2<f64> {
    let &[m00, m01, m10, m11] = row_major;
    Matrix2::new(m00, m01, m10, m11)
}

fn to_row_major(m: &Matrix2<f64>) -> [f64; 4] {
    [m.m11, m.m12, m.m21, m.m22]
}

/// Temperature-scaled softmax of `scores` written into `out` (must be the same length).
pub fn calc_weight_vec(scores: &[f64], temperature: f64, out: &mut [f64]) {
    let max_score = scores.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let mut sum = 0.0;
    for (slot, &s) in out.iter_mut().zip(scores.iter()) {
        let e = libm::exp((s - max_score) / temperature);
        *slot = e;
        sum += e;
    }
    for slot in out.iter_mut() {
        *slot /= sum;
    }
}

/// Weighted mean and covariance of 2D points. `poses2d_flat` is `[x0,y0, x1,y1, ...]`
/// (`2 * weights.len()` long). Returns `(mean[x,y], cov row-major 2x2)`.
#[allow(
    clippy::arithmetic_side_effects,
    clippy::allow_attributes,
    reason = "nalgebra f64 matrix math"
)]
#[must_use]
pub fn calculate_weighted_mean_and_cov(
    poses2d_flat: &[f64],
    weights: &[f64],
) -> ([f64; 2], [f64; 4]) {
    let mut mean = Vector2::zeros();
    for (chunk, &w) in poses2d_flat.chunks_exact(2).zip(weights.iter()) {
        let [px, py] = chunk else { continue };
        mean += w * Vector2::new(*px, *py);
    }
    let mut cov = Matrix2::zeros();
    for (chunk, &w) in poses2d_flat.chunks_exact(2).zip(weights.iter()) {
        let [px, py] = chunk else { continue };
        let diff = Vector2::new(*px, *py) - mean;
        cov += w * (diff * diff.transpose());
    }
    ([mean.x, mean.y], to_row_major(&cov))
}

/// `-inverse` of the top-left 2x2 block of a row-major 6x6 Hessian. NaN-filled if singular.
#[allow(
    clippy::arithmetic_side_effects,
    clippy::allow_attributes,
    reason = "nalgebra f64 matrix math"
)]
#[must_use]
pub fn laplace_xy_covariance(hessian_row_major: &[f64; 36]) -> [f64; 4] {
    // top-left 2x2 of the row-major 6x6: indices 0,1 (row 0) and 6,7 (row 1).
    let &[h00, h01, _, _, _, _, h10, h11, ..] = hessian_row_major;
    let hessian_xy = Matrix2::new(h00, h01, h10, h11);
    match hessian_xy.try_inverse() {
        Some(inv) => to_row_major(&(-inv)),
        None => [f64::NAN; 4],
    }
}

/// `R^T * C * R` — rotate a map-frame covariance into base_link. `rot` is the row-major 2x2 yaw block.
#[allow(
    clippy::arithmetic_side_effects,
    clippy::doc_markdown,
    clippy::allow_attributes,
    reason = "nalgebra f64 matrix math; base_link is a domain term, not code"
)]
#[must_use]
pub fn rotate_covariance_to_base_link(cov: &[f64; 4], rot: &[f64; 4]) -> [f64; 4] {
    let r = mat2(rot);
    to_row_major(&(r.transpose() * mat2(cov) * r))
}

/// `R * C * R^T` — rotate a base_link-frame covariance into map.
#[allow(
    clippy::arithmetic_side_effects,
    clippy::allow_attributes,
    reason = "nalgebra f64 matrix math"
)]
#[must_use]
pub fn rotate_covariance_to_map(cov: &[f64; 4], rot: &[f64; 4]) -> [f64; 4] {
    let r = mat2(rot);
    to_row_major(&(r * mat2(cov) * r.transpose()))
}

/// Clamp the base_link-frame diagonal to floors, then rotate back to map.
#[allow(
    clippy::arithmetic_side_effects,
    clippy::allow_attributes,
    reason = "nalgebra f64 matrix math"
)]
#[must_use]
pub fn adjust_diagonal_covariance(
    cov: &[f64; 4],
    rot: &[f64; 4],
    fixed_cov00: f64,
    fixed_cov11: f64,
) -> [f64; 4] {
    let base = mat2(&rotate_covariance_to_base_link(cov, rot));
    let clamped = Matrix2::new(
        base.m11.max(fixed_cov00),
        base.m12,
        base.m21,
        base.m22.max(fixed_cov11),
    );
    let r = mat2(rot);
    to_row_major(&(r * clamped * r.transpose()))
}

// ---- C ABI shims (no_std; pointers validated per rust-c-ffi-safety) ----

/// # Safety
/// `scores`/`out` point to `n` readable/writable `f64`. No-op if either is null.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_calc_weight_vec(
    scores: *const f64,
    n: usize,
    temperature: f64,
    out: *mut f64,
) {
    let scores = ffi_slice!(scores, n, else return);
    let out = ffi_mut_slice!(out, n, else return);
    calc_weight_vec(scores, temperature, out);
}

/// # Safety
/// `poses2d` points to `2*n` `f64`, `weights` to `n` `f64`, `mean_out` to 2, `cov_out` to 4.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_calculate_weighted_mean_and_cov(
    poses2d: *const f64,
    weights: *const f64,
    n: usize,
    mean_out: *mut f64,
    cov_out: *mut f64,
) {
    if mean_out.is_null() || cov_out.is_null() {
        return;
    }
    let Some(poses_len) = n.checked_mul(2) else {
        return;
    };
    let poses = ffi_slice!(poses2d, poses_len, else return);
    let weights = ffi_slice!(weights, n, else return);
    let (mean, cov) = calculate_weighted_mean_and_cov(poses, weights);
    // SAFETY: mean_out has 2, cov_out has 4 f64 per the contract, audited in ffi_ptr.
    unsafe {
        ffi_ptr::write_out(mean_out.cast::<[f64; 2]>(), mean);
        ffi_ptr::write_out(cov_out.cast::<[f64; 4]>(), cov);
    }
}

/// # Safety
/// `hessian` points to 36 `f64` (row-major 6x6); `cov_out` to 4.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_laplace_xy_covariance(
    hessian: *const f64,
    cov_out: *mut f64,
) {
    let hessian = ffi_ref!(hessian.cast::<[f64; 36]>(), else return);
    let result = laplace_xy_covariance(hessian);
    // SAFETY: `cov_out` has 4 f64 per the contract, audited in ffi_ptr.
    unsafe { ffi_ptr::write_out(cov_out.cast::<[f64; 4]>(), result) };
}

/// # Safety
/// `cov`/`rot` point to 4 `f64` (row-major 2x2), `out` to 4. No-op if any is null.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_rotate_covariance_to_base_link(
    cov: *const f64,
    rot: *const f64,
    out: *mut f64,
) {
    let cov = ffi_ref!(cov.cast::<[f64; 4]>(), else return);
    let rot = ffi_ref!(rot.cast::<[f64; 4]>(), else return);
    let result = rotate_covariance_to_base_link(cov, rot);
    // SAFETY: `out` has 4 f64 per the contract, audited in ffi_ptr.
    unsafe { ffi_ptr::write_out(out.cast::<[f64; 4]>(), result) };
}

/// # Safety
/// `cov`/`rot` point to 4 `f64` (row-major 2x2), `out` to 4. No-op if any is null.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_rotate_covariance_to_map(
    cov: *const f64,
    rot: *const f64,
    out: *mut f64,
) {
    let cov = ffi_ref!(cov.cast::<[f64; 4]>(), else return);
    let rot = ffi_ref!(rot.cast::<[f64; 4]>(), else return);
    let result = rotate_covariance_to_map(cov, rot);
    // SAFETY: `out` has 4 f64 per the contract, audited in ffi_ptr.
    unsafe { ffi_ptr::write_out(out.cast::<[f64; 4]>(), result) };
}

/// # Safety
/// `cov`/`rot` point to 4 `f64` (row-major 2x2), `out` to 4. No-op if any is null.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_adjust_diagonal_covariance(
    cov: *const f64,
    rot: *const f64,
    fixed_cov00: f64,
    fixed_cov11: f64,
    out: *mut f64,
) {
    let cov = ffi_ref!(cov.cast::<[f64; 4]>(), else return);
    let rot = ffi_ref!(rot.cast::<[f64; 4]>(), else return);
    let result = adjust_diagonal_covariance(cov, rot, fixed_cov00, fixed_cov11);
    // SAFETY: `out` has 4 f64 per the contract, audited in ffi_ptr.
    unsafe { ffi_ptr::write_out(out.cast::<[f64; 4]>(), result) };
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    clippy::indexing_slicing,
    unsafe_code,
    clippy::allow_attributes,
    reason = "test code"
)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn calc_weight_vec_uniform_for_equal_scores() {
        let scores = [1.5_f64; 4];
        let mut out = [0.0_f64; 4];
        calc_weight_vec(&scores, 1.0, &mut out);
        for w in out {
            assert!(close(w, 0.25));
        }
    }

    #[test]
    fn calc_weight_vec_known_softmax() {
        let scores = [0.0, libm::log(3.0)];
        let mut out = [0.0_f64; 2];
        calc_weight_vec(&scores, 1.0, &mut out);
        assert!(close(out[0], 0.25));
        assert!(close(out[1], 0.75));
    }

    #[test]
    fn weighted_mean_and_cov_known_values() {
        let poses = [0.0, 0.0, 2.0, 0.0];
        let weights = [0.5, 0.5];
        let (mean, cov) = calculate_weighted_mean_and_cov(&poses, &weights);
        assert!(close(mean[0], 1.0) && close(mean[1], 0.0));
        assert!(
            close(cov[0], 1.0) && close(cov[1], 0.0) && close(cov[2], 0.0) && close(cov[3], 0.0)
        );
    }

    #[test]
    fn laplace_negative_inverse_hessian() {
        let mut h = [0.0_f64; 36];
        h[0] = -2.0; // (0,0)
        h[7] = -4.0; // (1,1)
        h[14] = -100.0; // (2,2)
        h[21] = 7.0; // (3,3)
        let cov = laplace_xy_covariance(&h);
        assert!(
            close(cov[0], 0.5) && close(cov[3], 0.25) && close(cov[1], 0.0) && close(cov[2], 0.0)
        );
    }

    #[test]
    fn rotate_map_and_base_link_are_inverses() {
        let theta = 0.3_f64;
        let (c, s) = (libm::cos(theta), libm::sin(theta));
        let rot = [c, -s, s, c];
        let cov = [4.0, 1.0, 1.0, 9.0];
        let map = rotate_covariance_to_map(&cov, &rot);
        let exp00 = c * c * 4.0 - 2.0 * c * s + s * s * 9.0;
        let exp11 = s * s * 4.0 + 2.0 * c * s + c * c * 9.0;
        assert!(close(map[0], exp00) && close(map[3], exp11));
        let round_trip = rotate_covariance_to_base_link(&map, &rot);
        for (a, b) in round_trip.iter().zip(cov.iter()) {
            assert!(close(*a, *b));
        }
    }

    #[test]
    fn adjust_diagonal_clamps_and_keeps() {
        let identity = [1.0, 0.0, 0.0, 1.0];
        let clamped = adjust_diagonal_covariance(&[1.0, 0.0, 0.0, 1.0], &identity, 4.0, 9.0);
        assert!(close(clamped[0], 4.0) && close(clamped[3], 9.0));
        let kept = adjust_diagonal_covariance(&[10.0, 0.0, 0.0, 20.0], &identity, 4.0, 9.0);
        assert!(close(kept[0], 10.0) && close(kept[3], 20.0));
    }

    // ---- FFI shims: marshaling must equal the pure fn; null pointers must be a no-op. ----
    // The pure fns are checked above; these tests would fail if a shim mis-marshaled length /
    // row-col order / which buffer, so the equivalence assertion is load-bearing.

    #[test]
    fn ffi_calc_weight_vec_matches_pure_and_null_is_noop() {
        let scores = [0.0, libm::log(3.0), 0.5];
        let mut expected = [0.0_f64; 3];
        calc_weight_vec(&scores, 1.0, &mut expected);
        let mut got = [0.0_f64; 3];
        unsafe {
            autoware_ndt_scan_matcher_rs_calc_weight_vec(scores.as_ptr(), 3, 1.0, got.as_mut_ptr());
        }
        assert_eq!(got, expected);

        let mut sentinel = [42.0_f64; 3];
        unsafe {
            autoware_ndt_scan_matcher_rs_calc_weight_vec(
                core::ptr::null(),
                3,
                1.0,
                sentinel.as_mut_ptr(),
            );
        }
        assert_eq!(
            sentinel, [42.0_f64; 3],
            "null scores must leave output untouched"
        );
    }

    #[test]
    fn ffi_weighted_mean_and_cov_matches_pure() {
        let poses = [0.0, 0.0, 2.0, 0.0, 1.0, 1.0];
        let weights = [0.25, 0.5, 0.25];
        let (e_mean, e_cov) = calculate_weighted_mean_and_cov(&poses, &weights);
        let (mut mean, mut cov) = ([0.0_f64; 2], [0.0_f64; 4]);
        unsafe {
            autoware_ndt_scan_matcher_rs_calculate_weighted_mean_and_cov(
                poses.as_ptr(),
                weights.as_ptr(),
                3,
                mean.as_mut_ptr(),
                cov.as_mut_ptr(),
            );
        }
        assert_eq!(mean, e_mean);
        assert_eq!(cov, e_cov);
    }

    #[test]
    fn ffi_laplace_matches_pure_and_null_is_noop() {
        let mut h = [0.0_f64; 36];
        h[0] = -2.0;
        h[7] = -4.0;
        let expected = laplace_xy_covariance(&h);
        let mut got = [0.0_f64; 4];
        unsafe { autoware_ndt_scan_matcher_rs_laplace_xy_covariance(h.as_ptr(), got.as_mut_ptr()) };
        assert_eq!(got, expected);

        let mut sentinel = [7.0_f64; 4];
        unsafe {
            autoware_ndt_scan_matcher_rs_laplace_xy_covariance(
                core::ptr::null(),
                sentinel.as_mut_ptr(),
            );
        };
        assert_eq!(sentinel, [7.0_f64; 4]);
    }

    #[test]
    fn ffi_rotate_matches_pure_and_null_is_noop() {
        let cov = [4.0, 1.0, 1.0, 9.0];
        let (c, s) = (libm::cos(0.3), libm::sin(0.3));
        let rot = [c, -s, s, c];

        let e_map = rotate_covariance_to_map(&cov, &rot);
        let e_base = rotate_covariance_to_base_link(&cov, &rot);
        let (mut map, mut base) = ([0.0_f64; 4], [0.0_f64; 4]);
        unsafe {
            autoware_ndt_scan_matcher_rs_rotate_covariance_to_map(
                cov.as_ptr(),
                rot.as_ptr(),
                map.as_mut_ptr(),
            );
            autoware_ndt_scan_matcher_rs_rotate_covariance_to_base_link(
                cov.as_ptr(),
                rot.as_ptr(),
                base.as_mut_ptr(),
            );
        }
        assert_eq!(map, e_map);
        assert_eq!(base, e_base);

        // Both shims must leave their output untouched on a null input (each has its own branch).
        let mut s_map = [5.0_f64; 4];
        let mut s_base = [6.0_f64; 4];
        unsafe {
            autoware_ndt_scan_matcher_rs_rotate_covariance_to_map(
                core::ptr::null(),
                rot.as_ptr(),
                s_map.as_mut_ptr(),
            );
            autoware_ndt_scan_matcher_rs_rotate_covariance_to_base_link(
                core::ptr::null(),
                rot.as_ptr(),
                s_base.as_mut_ptr(),
            );
        }
        assert_eq!(s_map, [5.0_f64; 4]);
        assert_eq!(s_base, [6.0_f64; 4]);
    }

    #[test]
    fn ffi_adjust_diagonal_matches_pure_and_null_is_noop() {
        let cov = [1.0, 0.0, 0.0, 1.0];
        let identity = [1.0, 0.0, 0.0, 1.0];
        let expected = adjust_diagonal_covariance(&cov, &identity, 4.0, 9.0);
        let mut got = [0.0_f64; 4];
        unsafe {
            autoware_ndt_scan_matcher_rs_adjust_diagonal_covariance(
                cov.as_ptr(),
                identity.as_ptr(),
                4.0,
                9.0,
                got.as_mut_ptr(),
            );
        }
        assert_eq!(got, expected);

        let mut sentinel = [3.0_f64; 4];
        unsafe {
            autoware_ndt_scan_matcher_rs_adjust_diagonal_covariance(
                core::ptr::null(),
                identity.as_ptr(),
                4.0,
                9.0,
                sentinel.as_mut_ptr(),
            );
        }
        assert_eq!(sentinel, [3.0_f64; 4]);
    }

    // Error path: a singular top-left 2x2 Hessian block has no inverse, so the Laplace covariance
    // is NaN-filled (the documented contract). Oracle: every entry is NaN.
    #[test]
    fn laplace_singular_hessian_is_nan() {
        let mut h = [0.0_f64; 36];
        // top-left block [[1,1],[1,1]] is rank-1 (det 0) -> not invertible.
        h[0] = 1.0;
        h[1] = 1.0;
        h[6] = 1.0;
        h[7] = 1.0;
        let cov = laplace_xy_covariance(&h);
        assert!(
            cov.iter().all(|v| v.is_nan()),
            "singular Hessian -> all NaN"
        );
    }
}
