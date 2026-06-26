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

//! NDT derivative assembly (E4c), ported from `multigrid_ndt_omp_impl.hpp`:
//! - [`compute_derivatives`] — the source-point loop that queries the target map and accumulates the
//!   score, 6-gradient, and 6x6 Hessian (`computeDerivatives`, lines 378-533, incl. the
//!   regularization term 486-519).
//! - [`transformation_probability`] — score-only loop (`calculateTransformationProbability`, 1100).
//! - [`nearest_voxel_transformation_likelihood`] — per-point max-cell-score loop
//!   (`calculateNearestVoxelTransformationLikelihood`, 1153).
//!
//! Serial (the `ParReduce` parallel backends come at E4e). Per the zero-alloc policy, the per-point
//! loop reuses an [`AlignWorkspace`] buffer (`clear()` keeps capacity) so steady state allocates
//! nothing. `no_std` + `alloc`; the per-point math is `f64`, point clouds are `f32` (matching the
//! C++ `PointXYZ` → `Vector3d` promotion).

// Numeric kernel: f64 matrix/scalar operators (overflow lint), fixed-array / 0..6 indexing, f32<->f64
// casts inherent to point-cloud math + the regularization (mirrors C++ float arithmetic), and x/y/z
// single-char geometry names.
#![allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::many_single_char_names,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]

use alloc::vec::Vec;

use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};

use crate::derivatives::{
    compute_angle_derivatives, compute_point_derivatives, update_derivatives,
};
use crate::transform::GaussConstants;
use crate::voxel_grid::VoxelGridMap;

/// Reusable scratch buffers for the per-frame derivative pass. Hold one per engine and reuse across
/// frames; `compute_derivatives` `clear()`s (keeps capacity) per point, so after warmup the hot loop
/// performs no heap allocation. See `plan/ndt_in_rust.md` → "Runtime allocation policy".
#[derive(Debug, Default)]
pub struct AlignWorkspace {
    neighbor_idx: Vec<usize>,
}

impl AlignWorkspace {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            neighbor_idx: Vec::new(),
        }
    }
}

/// Optional longitudinal-distance regularization toward a reference pose
/// (`setRegularizationPose`). `scale_factor == 0` (the Autoware default) makes it a no-op.
#[derive(Clone, Copy, Debug)]
pub struct Regularization {
    /// Reference pose translation `(x, y)` in the map frame.
    pub pose_xy: [f32; 2],
    pub scale_factor: f32,
}

/// Result of one derivative pass at a pose.
#[derive(Clone, Copy, Debug)]
pub struct Derivatives {
    pub score: f64,
    pub gradient: Vector6<f64>,
    pub hessian: Matrix6<f64>,
    pub nearest_voxel_likelihood: f64,
    pub transform_probability: f64,
}

/// Row-major `[f64; 9]` (a `Leaf.icov`) → `Matrix3`.
fn mat3(icov: &[f64; 9]) -> Matrix3<f64> {
    Matrix3::new(
        icov[0], icov[1], icov[2], icov[3], icov[4], icov[5], icov[6], icov[7], icov[8],
    )
}

/// `[f32; 3]` point → `Vector3<f64>` (the C++ `Vector3d(pt.x, pt.y, pt.z)` promotion).
fn vec3(p: [f32; 3]) -> Vector3<f64> {
    Vector3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]))
}

/// Per-cell NDT score `-d1 · exp(-d2/2 · x_transᵀ Σ⁻¹ x_trans)` (eq. 6.9). `x_trans` is already
/// `transformed_point − cell.mean`. Shared by the score-only loops; matches the score
/// `update_derivatives` computes internally.
fn score_increment(x_trans: &Vector3<f64>, c_inv: &Matrix3<f64>, g: &GaussConstants) -> f64 {
    let quad = (x_trans.transpose() * c_inv * x_trans)[(0, 0)];
    -g.d1 * libm::exp(-g.d2 * quad * 0.5)
}

/// Score + gradient + Hessian over the source cloud (`computeDerivatives`). `source` are the
/// original points (for the transform derivatives); `trans_cloud` are the same points already
/// transformed by the current pose (for the neighbor search and `x_trans`); `p` is the 6-vector of
/// that pose (for the angular derivatives). `resolution` is the neighbor radius.
// These are the distinct inputs of the C++ `computeDerivatives`; the E4d engine struct will own the
// map / params / workspace and expose a narrower method, so the arg count is wrapped away there.
#[allow(clippy::too_many_arguments)]
#[must_use]
pub fn compute_derivatives(
    map: &VoxelGridMap,
    source: &[[f32; 3]],
    trans_cloud: &[[f32; 3]],
    p: &Vector6<f64>,
    resolution: f64,
    gauss: &GaussConstants,
    reg: Option<&Regularization>,
    ws: &mut AlignWorkspace,
) -> Derivatives {
    let mut gradient = Vector6::zeros();
    let mut hessian = Matrix6::zeros();
    let mut score = 0.0_f64;
    let mut nearest_voxel_score = 0.0_f64;
    let mut found: usize = 0;
    let mut total_neighborhood: usize = 0;

    let ad = compute_angle_derivatives(p);

    let n = source.len().min(trans_cloud.len());
    for idx in 0..n {
        let tp = trans_cloud[idx];
        ws.neighbor_idx.clear();
        map.radius_search(tp, resolution, 0, &mut ws.neighbor_idx);
        if ws.neighbor_idx.is_empty() {
            continue;
        }

        let x = vec3(source[idx]);
        let pd = compute_point_derivatives(&x, &ad);
        let x_trans = vec3(tp);

        let mut sum_pt = 0.0_f64;
        let mut nearest_pt = 0.0_f64;
        for &li in &ws.neighbor_idx {
            if let Some(leaf) = map.leaf(li) {
                let mean = Vector3::new(leaf.mean[0], leaf.mean[1], leaf.mean[2]);
                let c_inv = mat3(&leaf.icov);
                let s = update_derivatives(
                    &(x_trans - mean),
                    &c_inv,
                    &pd,
                    gauss,
                    &mut gradient,
                    &mut hessian,
                );
                sum_pt += s;
                if s > nearest_pt {
                    nearest_pt = s;
                }
            }
        }

        found += 1;
        score += sum_pt;
        nearest_voxel_score += nearest_pt;
        total_neighborhood += ws.neighbor_idx.len();
    }

    if let Some(r) = reg {
        // Mirrors C++ 486-519: float arithmetic, with sin/cos computed in f64 then cast to f32
        // (C++ `static_cast<float>(sin(p(5,0)))`). No-op when scale_factor == 0.
        let dx = r.pose_xy[0] - p[0] as f32;
        let dy = r.pose_xy[1] - p[1] as f32;
        let sin_yaw = libm::sin(p[5]) as f32;
        let cos_yaw = libm::cos(p[5]) as f32;
        let longitudinal = dy * sin_yaw + dx * cos_yaw;
        let w = total_neighborhood as f32;
        let sf = r.scale_factor;

        score += f64::from(-sf * w * longitudinal * longitudinal);
        gradient[0] += f64::from(sf * w * 2.0 * cos_yaw * longitudinal);
        gradient[1] += f64::from(sf * w * 2.0 * sin_yaw * longitudinal);
        let h01 = f64::from(-sf * w * 2.0 * cos_yaw * sin_yaw);
        hessian[(0, 0)] += f64::from(-sf * w * 2.0 * cos_yaw * cos_yaw);
        hessian[(0, 1)] += h01;
        hessian[(1, 1)] += f64::from(-sf * w * 2.0 * sin_yaw * sin_yaw);
        hessian[(1, 0)] += h01;
    }

    let nearest_voxel_likelihood = if found != 0 {
        nearest_voxel_score / found as f64
    } else {
        0.0
    };
    let transform_probability = if source.is_empty() {
        0.0
    } else {
        score / source.len() as f64
    };

    Derivatives {
        score,
        gradient,
        hessian,
        nearest_voxel_likelihood,
        transform_probability,
    }
}

/// Score-only transformation probability: sum over all neighbor cells of the per-cell score,
/// divided by the cloud size (`calculateTransformationProbability`).
#[must_use]
pub fn transformation_probability(
    map: &VoxelGridMap,
    trans_cloud: &[[f32; 3]],
    resolution: f64,
    gauss: &GaussConstants,
    ws: &mut AlignWorkspace,
) -> f64 {
    let mut score = 0.0_f64;
    for &tp in trans_cloud {
        ws.neighbor_idx.clear();
        map.radius_search(tp, resolution, 0, &mut ws.neighbor_idx);
        if ws.neighbor_idx.is_empty() {
            continue;
        }
        let x_trans = vec3(tp);
        for &li in &ws.neighbor_idx {
            if let Some(leaf) = map.leaf(li) {
                let mean = Vector3::new(leaf.mean[0], leaf.mean[1], leaf.mean[2]);
                score += score_increment(&(x_trans - mean), &mat3(&leaf.icov), gauss);
            }
        }
    }
    if trans_cloud.is_empty() {
        0.0
    } else {
        score / trans_cloud.len() as f64
    }
}

/// Per-point maximum cell score, averaged over points that found a neighbor
/// (`calculateNearestVoxelTransformationLikelihood`).
#[must_use]
pub fn nearest_voxel_transformation_likelihood(
    map: &VoxelGridMap,
    trans_cloud: &[[f32; 3]],
    resolution: f64,
    gauss: &GaussConstants,
    ws: &mut AlignWorkspace,
) -> f64 {
    let mut nearest_voxel_score = 0.0_f64;
    let mut found: usize = 0;
    for &tp in trans_cloud {
        ws.neighbor_idx.clear();
        map.radius_search(tp, resolution, 0, &mut ws.neighbor_idx);
        if ws.neighbor_idx.is_empty() {
            continue;
        }
        let x_trans = vec3(tp);
        let mut nearest_pt = 0.0_f64;
        for &li in &ws.neighbor_idx {
            if let Some(leaf) = map.leaf(li) {
                let mean = Vector3::new(leaf.mean[0], leaf.mean[1], leaf.mean[2]);
                let s = score_increment(&(x_trans - mean), &mat3(&leaf.icov), gauss);
                if s > nearest_pt {
                    nearest_pt = s;
                }
            }
        }
        nearest_voxel_score += nearest_pt;
        found += 1;
    }
    if found != 0 {
        nearest_voxel_score / found as f64
    } else {
        0.0
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    clippy::expect_used,
    clippy::unreadable_literal,
    clippy::needless_range_loop
)]
mod tests {
    use super::*;
    use crate::transform::{gauss_constants, transform_point};

    // 8 points packed inside one voxel around (cx,cy,cz) for leaf_size 1.0.
    fn dense_cluster(cx: f32, cy: f32, cz: f32) -> alloc::vec::Vec<[f32; 3]> {
        (0..8)
            .map(|i| {
                let f = i as f32 * 0.02;
                [cx + f, cy - f, cz + 0.5 * f]
            })
            .collect()
    }

    fn test_map() -> VoxelGridMap {
        let mut map = VoxelGridMap::new([1.0, 1.0, 1.0], 6, 0.01);
        map.add_target(&dense_cluster(0.5, 0.5, 0.5), 0);
        map.add_target(&dense_cluster(2.5, 0.5, 0.5), 1);
        map.create_kdtree();
        map
    }

    fn test_source() -> alloc::vec::Vec<[f32; 3]> {
        alloc::vec![[0.55, 0.5, 0.5], [0.5, 0.45, 0.52], [2.55, 0.5, 0.5]]
    }

    // trans_cloud = transform_point(p, source) stored as f32.
    fn transform_cloud(source: &[[f32; 3]], p: &Vector6<f64>) -> alloc::vec::Vec<[f32; 3]> {
        source
            .iter()
            .map(|s| {
                let t = transform_point(p, &vec3(*s));
                [t[0] as f32, t[1] as f32, t[2] as f32]
            })
            .collect()
    }

    // Pure-f64 reference score: the per-cell score math runs in f64 (smooth in `p`), while neighbors
    // are found with the f32 query (matching `compute_derivatives`' neighbor set). Used as the FD
    // oracle so f32 cloud quantization doesn't add finite-difference noise.
    fn ref_score(
        map: &VoxelGridMap,
        source: &[[f32; 3]],
        p: &Vector6<f64>,
        g: &GaussConstants,
    ) -> f64 {
        let mut s = 0.0_f64;
        let mut ws = AlignWorkspace::new();
        for &sp in source {
            let xt = transform_point(p, &vec3(sp));
            let q = [xt[0] as f32, xt[1] as f32, xt[2] as f32];
            ws.neighbor_idx.clear();
            map.radius_search(q, 1.0, 0, &mut ws.neighbor_idx);
            for &li in &ws.neighbor_idx {
                if let Some(leaf) = map.leaf(li) {
                    let mean = Vector3::new(leaf.mean[0], leaf.mean[1], leaf.mean[2]);
                    s += score_increment(&(xt - mean), &mat3(&leaf.icov), g);
                }
            }
        }
        s
    }

    // Headline oracle: the assembled gradient equals the central finite difference of the full
    // multi-point score; the translation Hessian rows equal the second FD (the angle-angle block is
    // the pcl form, validated vs C++ at E4d). Also asserts Hessian symmetry.
    #[test]
    fn compute_derivatives_matches_finite_difference() {
        let map = test_map();
        let source = test_source();
        let g = gauss_constants(0.55, 1.0);
        let p = Vector6::new(0.05, -0.03, 0.02, 0.04, -0.02, 0.03);

        let trans = transform_cloud(&source, &p);
        let mut ws = AlignWorkspace::new();
        let d = compute_derivatives(&map, &source, &trans, &p, 1.0, &g, None, &mut ws);
        assert!(d.score > 0.0, "score should be positive for a near match");

        // gradient vs central FD of the score.
        let eps = 1e-6;
        for k in 0..6 {
            let mut pp = p;
            let mut pm = p;
            pp[k] += eps;
            pm[k] -= eps;
            let fd = (ref_score(&map, &source, &pp, &g) - ref_score(&map, &source, &pm, &g))
                / (2.0 * eps);
            assert!(
                (d.gradient[k] - fd).abs() < 1e-4,
                "grad[{k}] {} vs fd {}",
                d.gradient[k],
                fd
            );
        }

        // symmetry.
        for i in 0..6 {
            for j in 0..6 {
                assert!((d.hessian[(i, j)] - d.hessian[(j, i)]).abs() < 1e-9);
            }
        }

        // translation Hessian rows (0..3) vs second central FD (exact part).
        let h = 1e-4;
        for i in 0..3 {
            for j in 0..6 {
                let (mut ppp, mut ppm, mut pmp, mut pmm) = (p, p, p, p);
                ppp[i] += h;
                ppp[j] += h;
                ppm[i] += h;
                ppm[j] -= h;
                pmp[i] -= h;
                pmp[j] += h;
                pmm[i] -= h;
                pmm[j] -= h;
                let fd = (ref_score(&map, &source, &ppp, &g)
                    - ref_score(&map, &source, &ppm, &g)
                    - ref_score(&map, &source, &pmp, &g)
                    + ref_score(&map, &source, &pmm, &g))
                    / (4.0 * h * h);
                assert!(
                    (d.hessian[(i, j)] - fd).abs() < 5e-3,
                    "hess[{i}][{j}] {} vs fd {}",
                    d.hessian[(i, j)],
                    fd
                );
            }
        }
    }

    // The score-only loops are independent code paths; they must agree with compute_derivatives.
    #[test]
    fn score_only_loops_agree_with_compute_derivatives() {
        let map = test_map();
        let source = test_source();
        let g = gauss_constants(0.55, 1.0);
        let p = Vector6::new(0.05, -0.03, 0.02, 0.04, -0.02, 0.03);
        let trans = transform_cloud(&source, &p);

        let mut ws = AlignWorkspace::new();
        let d = compute_derivatives(&map, &source, &trans, &p, 1.0, &g, None, &mut ws);

        let tp = transformation_probability(&map, &trans, 1.0, &g, &mut ws);
        assert!(
            (tp - d.transform_probability).abs() < 1e-12,
            "{tp} vs {}",
            d.transform_probability
        );

        let nvl = nearest_voxel_transformation_likelihood(&map, &trans, 1.0, &g, &mut ws);
        assert!(
            (nvl - d.nearest_voxel_likelihood).abs() < 1e-12,
            "{nvl} vs {}",
            d.nearest_voxel_likelihood
        );
    }

    #[test]
    fn empty_cloud_and_no_neighbor_are_zero() {
        let map = test_map();
        let g = gauss_constants(0.55, 1.0);
        let p = Vector6::zeros();
        let mut ws = AlignWorkspace::new();

        // Empty cloud.
        let d = compute_derivatives(&map, &[], &[], &p, 1.0, &g, None, &mut ws);
        assert_eq!(d.score, 0.0);
        assert_eq!(d.transform_probability, 0.0);
        assert_eq!(d.nearest_voxel_likelihood, 0.0);

        // A point far from any voxel contributes nothing and is not counted.
        let source = alloc::vec![[100.0_f32, 100.0, 100.0]];
        let trans = source.clone();
        let d2 = compute_derivatives(&map, &source, &trans, &p, 1.0, &g, None, &mut ws);
        assert_eq!(d2.score, 0.0);
        assert_eq!(d2.gradient, Vector6::zeros());
        assert_eq!(d2.nearest_voxel_likelihood, 0.0);

        // The standalone score-only loops honor the same empty / no-neighbor contract (0.0).
        assert_eq!(transformation_probability(&map, &[], 1.0, &g, &mut ws), 0.0);
        assert_eq!(
            nearest_voxel_transformation_likelihood(&map, &[], 1.0, &g, &mut ws),
            0.0
        );
        assert_eq!(
            transformation_probability(&map, &source, 1.0, &g, &mut ws),
            0.0
        );
        assert_eq!(
            nearest_voxel_transformation_likelihood(&map, &source, 1.0, &g, &mut ws),
            0.0
        );
    }

    // reg = None and a zero-scale Regularization must produce identical results.
    #[test]
    fn zero_scale_regularization_is_noop() {
        let map = test_map();
        let source = test_source();
        let g = gauss_constants(0.55, 1.0);
        let p = Vector6::new(0.05, -0.03, 0.02, 0.04, -0.02, 0.03);
        let trans = transform_cloud(&source, &p);
        let mut ws = AlignWorkspace::new();

        let d_none = compute_derivatives(&map, &source, &trans, &p, 1.0, &g, None, &mut ws);
        let reg = Regularization {
            pose_xy: [0.0, 0.0],
            scale_factor: 0.0,
        };
        let d_zero = compute_derivatives(&map, &source, &trans, &p, 1.0, &g, Some(&reg), &mut ws);
        assert_eq!(d_none.score, d_zero.score);
        assert_eq!(d_none.gradient, d_zero.gradient);
        assert_eq!(d_none.hessian, d_zero.hessian);
    }
}
