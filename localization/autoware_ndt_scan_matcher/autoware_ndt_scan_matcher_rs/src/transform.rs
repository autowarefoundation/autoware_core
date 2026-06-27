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

//! SE3 transforms and the NDT Gaussian fitting constants (E4a), ported from
//! `multigrid_ndt_omp_impl.hpp`. The 6-vector parameterization is `p = [tx, ty, tz, roll, pitch,
//! yaw]` and the rotation is `Rx(roll) * Ry(pitch) * Rz(yaw)` (the C++ `AngleAxis` order at
//! `computeTransformation` lines 332-339; the same intrinsic XYZ order Eigen's
//! `eulerAngles(0, 1, 2)` produces at line 283). All math is `f64`; `no_std` via `libm`.

// Numeric kernel: nalgebra f64 matrix/scalar operators are the float-math domain the
// integer-overflow lint targets; they cannot integer-overflow. Indexing is into fixed-size
// vectors/matrices with constant indices (statically in bounds). The f32 cloud transform mirrors the
// C++ `Matrix4f` pipeline, so the f64->f32 narrowing is intentional.
#![allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::many_single_char_names
)]

use alloc::vec::Vec;

use nalgebra::{Matrix3, Matrix4, Vector3, Vector6};

/// The Gaussian fitting constants `d1`, `d2`, `d3` (Magnusson 2009 eq. 6.8), recomputed per align
/// from the outlier ratio and voxel resolution (`computeTransformation` lines 256-260).
#[derive(Clone, Copy, Debug)]
pub struct GaussConstants {
    pub d1: f64,
    pub d2: f64,
    pub d3: f64,
}

/// Compute the Gaussian fitting constants. `outlier_ratio` defaults to 0.55 in Autoware;
/// `resolution` is the voxel size (also the neighbor search radius).
#[must_use]
pub fn gauss_constants(outlier_ratio: f64, resolution: f64) -> GaussConstants {
    let gauss_c1 = 10.0 * (1.0 - outlier_ratio);
    let gauss_c2 = outlier_ratio / (resolution * resolution * resolution);
    let d3 = -libm::log(gauss_c2);
    let d1 = -libm::log(gauss_c1 + gauss_c2) - d3;
    let d2 = -2.0 * libm::log((-libm::log(gauss_c1 * libm::exp(-0.5) + gauss_c2) - d3) / d1);
    GaussConstants { d1, d2, d3 }
}

/// `Rx(roll) * Ry(pitch) * Rz(yaw)` — the 3x3 rotation for the angle triple `(roll, pitch, yaw)`.
fn rotation(roll: f64, pitch: f64, yaw: f64) -> Matrix3<f64> {
    let (sx, cx) = (libm::sin(roll), libm::cos(roll));
    let (sy, cy) = (libm::sin(pitch), libm::cos(pitch));
    let (sz, cz) = (libm::sin(yaw), libm::cos(yaw));

    let rx = Matrix3::new(1.0, 0.0, 0.0, 0.0, cx, -sx, 0.0, sx, cx);
    let ry = Matrix3::new(cy, 0.0, sy, 0.0, 1.0, 0.0, -sy, 0.0, cy);
    let rz = Matrix3::new(cz, -sz, 0.0, sz, cz, 0.0, 0.0, 0.0, 1.0);
    rx * ry * rz
}

/// Build the 4x4 affine transform `Translation(tx,ty,tz) * Rx(roll) * Ry(pitch) * Rz(yaw)` from the
/// 6-vector `p` (matches `computeTransformation` lines 332-339).
#[must_use]
pub fn euler_to_matrix(p: &Vector6<f64>) -> Matrix4<f64> {
    let r = rotation(p[3], p[4], p[5]);
    let mut m = Matrix4::identity();
    m.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
    m[(0, 3)] = p[0];
    m[(1, 3)] = p[1];
    m[(2, 3)] = p[2];
    m
}

/// Extract `p = [tx, ty, tz, roll, pitch, yaw]` from a 4x4 affine transform — the inverse of
/// [`euler_to_matrix`] for `pitch` in `(-pi/2, pi/2)`.
///
/// NOTE: this is *a* valid inverse (the intrinsic-XYZ extraction), sufficient for round-trip tests.
/// E4d (initial-guess extraction) must reconcile the range convention with Eigen's
/// `eulerAngles(0, 1, 2)` (line 283), which folds angles differently for large rotations.
#[must_use]
pub fn matrix_to_euler(m: &Matrix4<f64>) -> Vector6<f64> {
    // For R = Rx(a)*Ry(b)*Rz(c): R[0][2] = sin(b); R[1][2] = -sin(a)cos(b); R[2][2] = cos(a)cos(b);
    // R[0][1] = -cos(b)sin(c); R[0][0] = cos(b)cos(c).
    let pitch = libm::asin(m[(0, 2)]);
    let roll = libm::atan2(-m[(1, 2)], m[(2, 2)]);
    let yaw = libm::atan2(-m[(0, 1)], m[(0, 0)]);
    Vector6::new(m[(0, 3)], m[(1, 3)], m[(2, 3)], roll, pitch, yaw)
}

/// Apply the transform of `p` to a point: `R(p) * x + t(p)`.
#[must_use]
pub fn transform_point(p: &Vector6<f64>, x: &Vector3<f64>) -> Vector3<f64> {
    let r = rotation(p[3], p[4], p[5]);
    (r * x) + Vector3::new(p[0], p[1], p[2])
}

/// The 4x4 affine of `p` built in **f32** — the C++ `final_transformation_` (`Matrix4f`):
/// `Translation(f32) * AngleAxis<float>` for roll/pitch/yaw. The angles are narrowed to f32 and the
/// trig is f32, matching the C++ float pipeline so the transformed cloud agrees bit-closely.
#[must_use]
pub fn se3_matrix_f32(p: &Vector6<f64>) -> Matrix4<f32> {
    let (sx, cx) = (libm::sinf(p[3] as f32), libm::cosf(p[3] as f32));
    let (sy, cy) = (libm::sinf(p[4] as f32), libm::cosf(p[4] as f32));
    let (sz, cz) = (libm::sinf(p[5] as f32), libm::cosf(p[5] as f32));
    let rx = Matrix3::<f32>::new(1.0, 0.0, 0.0, 0.0, cx, -sx, 0.0, sx, cx);
    let ry = Matrix3::<f32>::new(cy, 0.0, sy, 0.0, 1.0, 0.0, -sy, 0.0, cy);
    let rz = Matrix3::<f32>::new(cz, -sz, 0.0, sz, cz, 0.0, 0.0, 0.0, 1.0);
    let r = rx * ry * rz;
    let mut m = Matrix4::<f32>::identity();
    m.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
    m[(0, 3)] = p[0] as f32;
    m[(1, 3)] = p[1] as f32;
    m[(2, 3)] = p[2] as f32;
    m
}

/// Transform `source` by the pose of `p` into the reused buffer `out`, in **f32** — mirrors the C++
/// `pcl::transformPointCloud(source, trans_cloud, final_transformation_)`. Clears + reserves `out`
/// (capacity is retained across calls), so after the first call this performs no allocation.
pub fn transform_cloud_f32(p: &Vector6<f64>, source: &[[f32; 3]], out: &mut Vec<[f32; 3]>) {
    let m = se3_matrix_f32(p);
    let r = m.fixed_view::<3, 3>(0, 0).into_owned();
    let t = Vector3::new(m[(0, 3)], m[(1, 3)], m[(2, 3)]);
    out.clear();
    out.reserve(source.len()); // len == 0 after clear, so this reserves the full size (no-op once warm)
    for &s in source {
        let v = r * Vector3::new(s[0], s[1], s[2]) + t;
        out.push([v[0], v[1], v[2]]);
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::unreadable_literal
)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    // Oracle: an independent recomputation of eq. 6.8 using std transcendentals (the impl uses
    // libm), plus a hand-anchored magnitude for the default (0.55, 1.0) case.
    #[test]
    fn gauss_constants_matches_reference_formula() {
        let reference = |outlier: f64, res: f64| {
            let c1 = 10.0 * (1.0 - outlier);
            let c2 = outlier / res.powi(3);
            let d3 = -c2.ln();
            let d1 = -(c1 + c2).ln() - d3;
            let d2 = -2.0 * ((-(c1 * (-0.5f64).exp() + c2).ln() - d3) / d1).ln();
            (d1, d2, d3)
        };
        for (o, r) in [(0.55, 1.0), (0.55, 2.0), (0.4, 1.5)] {
            let g = gauss_constants(o, r);
            let (d1, d2, d3) = reference(o, r);
            assert!(close(g.d1, d1, 1e-12), "d1={} ref={d1}", g.d1);
            assert!(close(g.d2, d2, 1e-12), "d2={} ref={d2}", g.d2);
            assert!(close(g.d3, d3, 1e-12), "d3={} ref={d3}", g.d3);
        }
        // Hand-anchored magnitude (catches an identical-transcription error in both impl + ref).
        let g = gauss_constants(0.55, 1.0);
        assert!(close(g.d1, -2.2172252, 1e-6), "d1={}", g.d1);
        assert!(close(g.d3, -(0.55_f64.ln()), 1e-12), "d3={}", g.d3);
    }

    // Oracle: a hand-built Rx*Ry*Rz product and a manual R*x + t.
    #[test]
    fn euler_matrix_and_transform_point() {
        let p = Vector6::new(1.0, 2.0, 3.0, 0.2, -0.3, 0.4);
        let (sx, cx) = (0.2_f64.sin(), 0.2_f64.cos());
        let (sy, cy) = ((-0.3_f64).sin(), (-0.3_f64).cos());
        let (sz, cz) = (0.4_f64.sin(), 0.4_f64.cos());
        let rx = Matrix3::new(1.0, 0.0, 0.0, 0.0, cx, -sx, 0.0, sx, cx);
        let ry = Matrix3::new(cy, 0.0, sy, 0.0, 1.0, 0.0, -sy, 0.0, cy);
        let rz = Matrix3::new(cz, -sz, 0.0, sz, cz, 0.0, 0.0, 0.0, 1.0);
        let r = rx * ry * rz;

        let m = euler_to_matrix(&p);
        for row in 0..3 {
            for col in 0..3 {
                assert!(
                    close(m[(row, col)], r[(row, col)], 1e-12),
                    "R[{row}][{col}]"
                );
            }
        }
        assert!(
            close(m[(0, 3)], 1.0, 0.0) && close(m[(1, 3)], 2.0, 0.0) && close(m[(2, 3)], 3.0, 0.0)
        );

        let x = Vector3::new(0.5, -1.5, 2.0);
        let manual = (r * x) + Vector3::new(1.0, 2.0, 3.0);
        let got = transform_point(&p, &x);
        for d in 0..3 {
            assert!(close(got[d], manual[d], 1e-12), "tp[{d}]");
        }
    }

    // matrix_to_euler must invert euler_to_matrix for non-gimbal angles (round-trip oracle).
    #[test]
    fn euler_round_trips_for_small_angles() {
        let p = Vector6::new(-4.0, 0.5, 7.0, 0.3, -0.6, 1.1);
        let back = matrix_to_euler(&euler_to_matrix(&p));
        for d in 0..6 {
            assert!(close(back[d], p[d], 1e-9), "p[{d}] {} vs {}", back[d], p[d]);
        }
    }
}
