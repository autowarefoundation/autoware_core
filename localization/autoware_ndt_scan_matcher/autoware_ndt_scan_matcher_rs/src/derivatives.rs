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

//! NDT score/gradient/Hessian derivative kernels, ported verbatim from
//! `multigrid_ndt_omp_impl.hpp` (Magnusson 2009, eq. 6.9-6.21):
//! - [`compute_angle_derivatives`] — the per-iteration angular Jacobian/Hessian (`j_ang`/`h_ang`,
//!   lines 537-611).
//! - [`compute_point_derivatives`] — the per-source-point transform derivatives (lines 615-659).
//! - [`update_derivatives`] — the per-point/per-cell score + gradient + Hessian accumulation
//!   (lines 663-717).
//!
//! All `f64`; `no_std` via `libm`. These are internal building blocks for the derivative-assembly +
//! optimization loop (no FFI here — the C++ counterparts are private methods).

// Numeric kernel: nalgebra f64 matrix operators are the float-math domain the integer-overflow lint
// targets; they cannot integer-overflow. Indexing is into fixed-size const-generic matrices with
// constant / 0..6 indices (statically in bounds); x/y/z and the eq. 6.21 a..f are domain notation.
// `compute_angle_derivatives` is one long verbatim transcription of the 23 j_ang/h_ang rows
// (too_many_lines); `x_j_ang`/`x_h_ang` are the C++ variable names (similar_names); the validity
// guard mirrors C++'s explicit `> 1 || < 0` with an added NaN check (manual_range_contains).
// Suppressions are scoped per-function (no module-wide `#![allow]`); rationale per the comment above.

use nalgebra::{Matrix3, Matrix4, Matrix6, RowVector4, SMatrix, Vector4, Vector6};

use crate::transform::GaussConstants;

/// Per-iteration precomputed angular Jacobian (`j_ang`, 8x4) and Hessian (`h_ang`, 16x4).
#[derive(Clone, Debug)]
pub struct AngleDerivatives {
    j_ang: SMatrix<f64, 8, 4>,
    h_ang: SMatrix<f64, 16, 4>,
}

/// Per-source-point transform derivatives: `gradient` (4x6, eq. 6.18/6.19) and `hessian`
/// (24x6 = six stacked 4x6 blocks, eq. 6.20/6.21).
#[derive(Clone, Debug)]
pub struct PointDerivatives {
    gradient: SMatrix<f64, 4, 6>,
    hessian: SMatrix<f64, 24, 6>,
}

/// Precompute the angular Jacobian/Hessian for transform vector `p = [tx,ty,tz,roll,pitch,yaw]`
/// (`computeAngleDerivatives`, lines 537-611). Angles within `10e-5` of zero are snapped to the
/// `(c, s) = (1, 0)` simplification, matching C++.
///
/// WCET: fixed-size `f64` matrix math (once per iteration) — `O(1)`, no allocation, no panic.
#[must_use]
#[allow(
    clippy::too_many_lines,
    clippy::allow_attributes,
    reason = "verbatim transcription of the 23 j_ang/h_ang rows (Magnusson eq. 6.19-6.21)"
)]
pub fn compute_angle_derivatives(p: &Vector6<f64>) -> AngleDerivatives {
    let (cx, sx) = if libm::fabs(p[3]) < 10e-5 {
        (1.0, 0.0)
    } else {
        (libm::cos(p[3]), libm::sin(p[3]))
    };
    let (cy, sy) = if libm::fabs(p[4]) < 10e-5 {
        (1.0, 0.0)
    } else {
        (libm::cos(p[4]), libm::sin(p[4]))
    };
    let (cz, sz) = if libm::fabs(p[5]) < 10e-5 {
        (1.0, 0.0)
    } else {
        (libm::cos(p[5]), libm::sin(p[5]))
    };

    let mut j_ang = SMatrix::<f64, 8, 4>::zeros();
    j_ang.set_row(
        0,
        &RowVector4::new(
            -sx * sz + cx * sy * cz,
            -sx * cz - cx * sy * sz,
            -cx * cy,
            0.0,
        ),
    );
    j_ang.set_row(
        1,
        &RowVector4::new(
            cx * sz + sx * sy * cz,
            cx * cz - sx * sy * sz,
            -sx * cy,
            0.0,
        ),
    );
    j_ang.set_row(2, &RowVector4::new(-sy * cz, sy * sz, cy, 0.0));
    j_ang.set_row(
        3,
        &RowVector4::new(sx * cy * cz, -sx * cy * sz, sx * sy, 0.0),
    );
    j_ang.set_row(
        4,
        &RowVector4::new(-cx * cy * cz, cx * cy * sz, -cx * sy, 0.0),
    );
    j_ang.set_row(5, &RowVector4::new(-cy * sz, -cy * cz, 0.0, 0.0));
    j_ang.set_row(
        6,
        &RowVector4::new(cx * cz - sx * sy * sz, -cx * sz - sx * sy * cz, 0.0, 0.0),
    );
    j_ang.set_row(
        7,
        &RowVector4::new(sx * cz + cx * sy * sz, cx * sy * cz - sx * sz, 0.0, 0.0),
    );

    let mut h_ang = SMatrix::<f64, 16, 4>::zeros();
    h_ang.set_row(
        0,
        &RowVector4::new(
            -cx * sz - sx * sy * cz,
            -cx * cz + sx * sy * sz,
            sx * cy,
            0.0,
        ),
    ); // a2
    h_ang.set_row(
        1,
        &RowVector4::new(
            -sx * sz + cx * sy * cz,
            -cx * sy * sz - sx * cz,
            -cx * cy,
            0.0,
        ),
    ); // a3
    h_ang.set_row(
        2,
        &RowVector4::new(cx * cy * cz, -cx * cy * sz, cx * sy, 0.0),
    ); // b2
    h_ang.set_row(
        3,
        &RowVector4::new(sx * cy * cz, -sx * cy * sz, sx * sy, 0.0),
    ); // b3
    h_ang.set_row(
        4,
        &RowVector4::new(-sx * cz - cx * sy * sz, sx * sz - cx * sy * cz, 0.0, 0.0),
    ); // c2
    h_ang.set_row(
        5,
        &RowVector4::new(cx * cz - sx * sy * sz, -sx * sy * cz - cx * sz, 0.0, 0.0),
    ); // c3
    h_ang.set_row(6, &RowVector4::new(-cy * cz, cy * sz, sy, 0.0)); // d1
    h_ang.set_row(
        7,
        &RowVector4::new(-sx * sy * cz, sx * sy * sz, sx * cy, 0.0),
    ); // d2
    h_ang.set_row(
        8,
        &RowVector4::new(cx * sy * cz, -cx * sy * sz, -cx * cy, 0.0),
    ); // d3
    h_ang.set_row(9, &RowVector4::new(sy * sz, sy * cz, 0.0, 0.0)); // e1
    h_ang.set_row(10, &RowVector4::new(-sx * cy * sz, -sx * cy * cz, 0.0, 0.0)); // e2
    h_ang.set_row(11, &RowVector4::new(cx * cy * sz, cx * cy * cz, 0.0, 0.0)); // e3
    h_ang.set_row(12, &RowVector4::new(-cy * cz, cy * sz, 0.0, 0.0)); // f1
    h_ang.set_row(
        13,
        &RowVector4::new(-cx * sz - sx * sy * cz, -cx * cz + sx * sy * sz, 0.0, 0.0),
    ); // f2
    h_ang.set_row(
        14,
        &RowVector4::new(-sx * sz + cx * sy * cz, -cx * sy * sz - sx * cz, 0.0, 0.0),
    ); // f3

    AngleDerivatives { j_ang, h_ang }
}

/// Per-source-point transform derivatives for the (untransformed) point `x`
/// (`computePointDerivatives`, lines 615-659). The `gradient` translation block (rows 0-2,
/// cols 0-2 = identity) is set here, where C++ sets it once in `computeDerivatives`.
///
/// WCET: fixed-size `f64` matrix math — `O(1)`, no allocation, no panic, no blocking.
#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::allow_attributes,
    reason = "fixed-size nalgebra matrix math: constant indices, float ops, eq.6.21 a..f names"
)]
#[must_use]
pub fn compute_point_derivatives(
    x: &nalgebra::Vector3<f64>,
    ad: &AngleDerivatives,
) -> PointDerivatives {
    let x4 = Vector4::new(x[0], x[1], x[2], 0.0);

    let mut gradient = SMatrix::<f64, 4, 6>::zeros();
    // Translation block: dT/d{tx,ty,tz} = identity in rows 0-2.
    gradient[(0, 0)] = 1.0;
    gradient[(1, 1)] = 1.0;
    gradient[(2, 2)] = 1.0;

    let x_j_ang = ad.j_ang * x4; // 8-vector
    gradient[(1, 3)] = x_j_ang[0];
    gradient[(2, 3)] = x_j_ang[1];
    gradient[(0, 4)] = x_j_ang[2];
    gradient[(1, 4)] = x_j_ang[3];
    gradient[(2, 4)] = x_j_ang[4];
    gradient[(0, 5)] = x_j_ang[5];
    gradient[(1, 5)] = x_j_ang[6];
    gradient[(2, 5)] = x_j_ang[7];

    let mut hessian = SMatrix::<f64, 24, 6>::zeros();
    let x_h_ang = ad.h_ang * x4; // 16-vector, entries 0..14 used
    let a = Vector4::new(0.0, x_h_ang[0], x_h_ang[1], 0.0);
    let b = Vector4::new(0.0, x_h_ang[2], x_h_ang[3], 0.0);
    let c = Vector4::new(0.0, x_h_ang[4], x_h_ang[5], 0.0);
    let d = Vector4::new(x_h_ang[6], x_h_ang[7], x_h_ang[8], 0.0);
    let e = Vector4::new(x_h_ang[9], x_h_ang[10], x_h_ang[11], 0.0);
    let f = Vector4::new(x_h_ang[12], x_h_ang[13], x_h_ang[14], 0.0);

    // 4x1 blocks at (row, col); rows 0-11 (translation second derivatives) stay zero.
    let mut set_block = |row: usize, col: usize, v: &Vector4<f64>| {
        for k in 0..4 {
            hessian[(row + k, col)] = v[k];
        }
    };
    set_block(12, 3, &a);
    set_block(16, 3, &b);
    set_block(20, 3, &c);
    set_block(12, 4, &b);
    set_block(16, 4, &d);
    set_block(20, 4, &e);
    set_block(12, 5, &c);
    set_block(16, 5, &e);
    set_block(20, 5, &f);

    PointDerivatives { gradient, hessian }
}

/// Per-point/per-cell update (`updateDerivatives`, lines 663-717). `x_trans = transformed_point -
/// cell.mean`, `c_inv` is the cell's 3x3 inverse covariance. Returns the score increment and, when
/// the point is valid, accumulates into `score_gradient` (6) and `hessian` (6x6). On an invalid
/// `e_x_cov_x` (outside `[0,1]` or NaN) returns `0.0` and leaves the accumulators untouched.
///
/// WCET: fixed-size `f64` matrix math — `O(1)`, no allocation, no panic, no blocking.
#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::manual_range_contains,
    clippy::allow_attributes,
    reason = "fixed-size nalgebra matrix math; the [0,1] guard mirrors C++'s explicit < 0 || > 1"
)]
pub fn update_derivatives(
    x_trans: &nalgebra::Vector3<f64>,
    c_inv: &Matrix3<f64>,
    pd: &PointDerivatives,
    g: &GaussConstants,
    score_gradient: &mut Vector6<f64>,
    hessian: &mut Matrix6<f64>,
) -> f64 {
    let x_trans4 = Vector4::new(x_trans[0], x_trans[1], x_trans[2], 0.0);
    let mut c_inv4 = Matrix4::<f64>::zeros();
    c_inv4.fixed_view_mut::<3, 3>(0, 0).copy_from(c_inv);

    // e^(-d2/2 * x_transᵀ Σ⁻¹ x_trans) (eq. 6.9). c_inv4 is symmetric, so the row*M*col form of C++
    // equals this quadratic form.
    let quad = x_trans4.dot(&(c_inv4 * x_trans4));
    let mut e_x_cov_x = libm::exp(-g.d2 * quad * 0.5);
    let score_inc = -g.d1 * e_x_cov_x;

    e_x_cov_x *= g.d2;
    // Error checking for invalid values (mirrors the C++ guard on d2*e_x_cov_x).
    if e_x_cov_x > 1.0 || e_x_cov_x < 0.0 || e_x_cov_x.is_nan() {
        return 0.0;
    }
    e_x_cov_x *= g.d1;

    let c_inv4_x_pg: SMatrix<f64, 4, 6> = c_inv4 * pd.gradient;
    // x_trans4ᵀ (c_inv4 * point_gradient): 1x6 -> stored as 6x1.
    let g6: Vector6<f64> = c_inv4_x_pg.transpose() * x_trans4;

    *score_gradient += e_x_cov_x * g6;

    // x_trans4ᵀ c_inv4 (1x4) as a column (c_inv4 symmetric).
    let v = c_inv4 * x_trans4;
    // point_gradientᵀ (c_inv4 * point_gradient): 6x6; C++ indexes (j, i).
    let pgcg: Matrix6<f64> = pd.gradient.transpose() * c_inv4_x_pg;

    for i in 0..6 {
        // x_trans4ᵀ c_inv4 * point_hessian.block<4,6>(4i, 0): 1x6 -> 6x1.
        let ph_block = pd.hessian.fixed_view::<4, 6>(i * 4, 0);
        let h6: Vector6<f64> = ph_block.transpose() * v;
        for j in 0..6 {
            hessian[(i, j)] += e_x_cov_x * (-g.d2 * g6[i] * g6[j] + h6[j] + pgcg[(j, i)]);
        }
    }

    score_inc
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::arithmetic_side_effects,
    clippy::unreadable_literal,
    clippy::needless_range_loop,
    clippy::allow_attributes,
    reason = "test code"
)]
mod tests {
    use super::*;
    use crate::transform::{gauss_constants, transform_point};
    use nalgebra::Vector3;

    // The single-point/single-cell NDT score S(p) used as the finite-difference oracle.
    fn score_at(
        p: &Vector6<f64>,
        x: &Vector3<f64>,
        mean: &Vector3<f64>,
        c_inv: &Matrix3<f64>,
        g: &GaussConstants,
    ) -> f64 {
        let xt = transform_point(p, x) - mean;
        let quad = (xt.transpose() * c_inv * xt)[(0, 0)];
        -g.d1 * libm::exp(-g.d2 * quad * 0.5)
    }

    // A symmetric positive-definite 3x3 inverse covariance for tests.
    fn spd_c_inv() -> Matrix3<f64> {
        Matrix3::new(2.0, 0.3, 0.1, 0.3, 1.5, -0.2, 0.1, -0.2, 1.0)
    }

    // compute_point_derivatives columns 3-5 must equal the finite difference of transform_point
    // w.r.t. each angle. Oracle: numerical differentiation, independent of the analytic j_ang.
    #[test]
    fn point_gradient_matches_finite_difference() {
        let p = Vector6::new(0.7, -0.4, 1.2, 0.25, -0.35, 0.5);
        let x = Vector3::new(1.3, -2.1, 0.6);
        let ad = compute_angle_derivatives(&p);
        let pd = compute_point_derivatives(&x, &ad);

        let eps = 1e-6;
        for (col, angle_idx) in [(3, 3), (4, 4), (5, 5)] {
            let mut pp = p;
            let mut pm = p;
            pp[angle_idx] += eps;
            pm[angle_idx] -= eps;
            let fd = (transform_point(&pp, &x) - transform_point(&pm, &x)) / (2.0 * eps);
            for row in 0..3 {
                assert!(
                    (pd.gradient[(row, col)] - fd[row]).abs() < 1e-5,
                    "grad[{row}][{col}] {} vs fd {}",
                    pd.gradient[(row, col)],
                    fd[row]
                );
            }
        }
        // Translation block is the identity.
        for d in 0..3 {
            assert!(pd.gradient[(d, d)] == 1.0);
        }
    }

    // The headline oracle: update_derivatives' accumulated gradient/Hessian must equal the central
    // finite differences of the scalar score S(p). Validates angle->point->update end-to-end.
    #[test]
    fn update_derivatives_gradient_and_hessian_match_finite_difference() {
        let p = Vector6::new(0.5, 0.2, -0.3, 0.2, -0.15, 0.3);
        let x = Vector3::new(0.8, 1.1, -0.5);
        let mean = Vector3::new(0.6, 1.4, -0.2);
        let c_inv = spd_c_inv();
        let g = gauss_constants(0.55, 1.0);

        let ad = compute_angle_derivatives(&p);
        let pd = compute_point_derivatives(&x, &ad);
        let x_trans = transform_point(&p, &x) - mean;

        let mut grad = Vector6::zeros();
        let mut hess = Matrix6::zeros();
        let score = update_derivatives(&x_trans, &c_inv, &pd, &g, &mut grad, &mut hess);
        // score equals the direct closed form.
        assert!((score - score_at(&p, &x, &mean, &c_inv, &g)).abs() < 1e-12);

        // Gradient vs central FD of S(p).
        let eps = 1e-6;
        for k in 0..6 {
            let mut pp = p;
            let mut pm = p;
            pp[k] += eps;
            pm[k] -= eps;
            let fd = (score_at(&pp, &x, &mean, &c_inv, &g) - score_at(&pm, &x, &mean, &c_inv, &g))
                / (2.0 * eps);
            assert!(
                (grad[k] - fd).abs() < 1e-5,
                "grad[{k}] {} vs fd {}",
                grad[k],
                fd
            );
        }

        // The Hessian is symmetric (true of the pcl formula: every term is symmetric in i,j).
        for i in 0..6 {
            for j in 0..6 {
                assert!(
                    (hess[(i, j)] - hess[(j, i)]).abs() < 1e-12,
                    "hessian not symmetric at ({i},{j})"
                );
            }
        }

        // Hessian rows 0-2 (translation derivatives) vs second central FD of S(p). These rows have
        // ZERO dependence on the angle second-derivative `point_hessian` blocks (rows 0-11 are
        // zero), so they equal the exact analytic Hessian and FD-validate cleanly.
        //
        // The angle-angle 3x3 block (i,j in 3..6) is intentionally NOT FD-tested: pcl/Autoware's
        // NDT Hessian is the *approximate* pcl form (its `h_ang` second-derivative terms differ
        // from the exact d^2T/dp^2 — e.g. row 6 `d1` uses +sy where the exact value is -sy), so FD
        // is the wrong oracle there. That block is verified against the C++ `NdtResult.hessian`.
        // The translation rows below + the gradient above pin down everything FD can judge.
        let h = 1e-4;
        let second_fd = |i: usize, j: usize| {
            let (mut ppp, mut ppm, mut pmp, mut pmm) = (p, p, p, p);
            ppp[i] += h;
            ppp[j] += h;
            ppm[i] += h;
            ppm[j] -= h;
            pmp[i] -= h;
            pmp[j] += h;
            pmm[i] -= h;
            pmm[j] -= h;
            (score_at(&ppp, &x, &mean, &c_inv, &g)
                - score_at(&ppm, &x, &mean, &c_inv, &g)
                - score_at(&pmp, &x, &mean, &c_inv, &g)
                + score_at(&pmm, &x, &mean, &c_inv, &g))
                / (4.0 * h * h)
        };
        for i in 0..3 {
            for j in 0..6 {
                let fd = second_fd(i, j);
                assert!(
                    (hess[(i, j)] - fd).abs() < 5e-3,
                    "hess[{i}][{j}] {} vs fd {}",
                    hess[(i, j)],
                    fd
                );
            }
        }
    }

    // Validity guard: an invalid e_x_cov_x returns 0.0 and must not mutate the accumulators.
    #[test]
    fn invalid_point_is_no_op() {
        let g = gauss_constants(0.55, 1.0);
        // d2 is small (~0.43); to push d2*e_x_cov_x above 1 we need e_x_cov_x large, which exp can't
        // exceed 1 for a non-negative quadratic. Instead force NaN via a non-finite x_trans.
        let p = Vector6::zeros();
        let x = Vector3::new(0.1, 0.2, 0.3);
        let ad = compute_angle_derivatives(&p);
        let pd = compute_point_derivatives(&x, &ad);
        let c_inv = spd_c_inv();
        let x_trans = Vector3::new(f64::NAN, 0.0, 0.0);

        let mut grad = Vector6::repeat(7.0);
        let mut hess = Matrix6::repeat(7.0);
        let score = update_derivatives(&x_trans, &c_inv, &pd, &g, &mut grad, &mut hess);
        assert_eq!(score, 0.0);
        assert!(
            grad.iter().all(|&v| v == 7.0),
            "gradient mutated on invalid point"
        );
        assert!(
            hess.iter().all(|&v| v == 7.0),
            "hessian mutated on invalid point"
        );
    }
}
