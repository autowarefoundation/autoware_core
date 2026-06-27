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
// Suppressions are scoped per-function (no module-wide `#![allow]`); rationale per the comment above.

use alloc::vec::Vec;

use nalgebra::{Matrix3, Matrix4, Matrix6, SVD, Vector3, Vector6};

use crate::derivatives::{
    AngleDerivatives, compute_angle_derivatives, compute_point_derivatives, update_derivatives,
};
use crate::transform::{
    GaussConstants, gauss_constants, matrix_to_euler, se3_matrix_f32, transform_cloud_f32,
};
use crate::voxel_grid::VoxelGridMap;

/// Per-point neighbor cap for the RT-critical `radius_search` (WCET bound on `K`). The kd-tree stops
/// collecting once `MAX_NEIGHBORS` are found, bounding both the result size and the traversal. Chosen
/// well above the physical worst case (a radius == `leaf_size` voxel neighborhood is ≤ 27 voxels), so
/// it does not truncate for real maps — if it ever does, the result deviates from the unbounded C++
/// `radiusSearch` (first-N in traversal order), which should be treated as a misconfiguration.
const MAX_NEIGHBORS: usize = 64;

/// Reusable scratch buffers for the per-frame derivative pass and the align loop. Hold one per engine
/// and reuse across frames; the buffers `clear()` (keep capacity) per call, so after warmup the hot
/// loop performs no heap allocation. See `plan/ndt_in_rust.md` → "Bounded WCET hot path".
#[derive(Debug, Default)]
pub struct AlignWorkspace {
    neighbor_idx: Vec<usize>,
    trans_cloud: Vec<[f32; 3]>,
    /// Per-point contributions for the parallel backend (reused across frames; order-preserving
    /// `collect_into_vec` fills it). Only the `parallel` feature touches it — the serial backend
    /// stays allocation-free.
    #[cfg(feature = "parallel")]
    contribs: Vec<PointContribution>,
}

impl AlignWorkspace {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            neighbor_idx: Vec::new(),
            trans_cloud: Vec::new(),
            #[cfg(feature = "parallel")]
            contribs: Vec::new(),
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

/// Per-align scoring configuration, fixed across the iteration loop (the C++ NDT object holds these
/// as members): the neighbor-search `resolution`, the Gaussian fitting `gauss` constants, and the
/// optional `reg`ularization.
#[derive(Clone, Copy, Debug)]
pub struct ScoreConfig<'a> {
    pub resolution: f64,
    pub gauss: &'a GaussConstants,
    pub reg: Option<&'a Regularization>,
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
#[allow(
    clippy::arithmetic_side_effects,
    clippy::allow_attributes,
    reason = "nalgebra f64 matrix product (quadratic form)"
)]
fn score_increment(x_trans: &Vector3<f64>, c_inv: &Matrix3<f64>, g: &GaussConstants) -> f64 {
    // x_transᵀ Σ⁻¹ x_trans as a scalar (avoids indexing the 1x1 product matrix).
    let quad = x_trans.dot(&(c_inv * x_trans));
    -g.d1 * libm::exp(-g.d2 * quad * 0.5)
}

/// One source point's independent contribution to the derivative reduction: the score, the local
/// 6-gradient and 6x6 Hessian (folded over the point's neighbor cells from zero), the per-point max
/// cell score (`nearest`), and the neighbor count. Computed independently of every other point, so
/// the serial and parallel backends fold these in identical point-index order and agree
/// **bit-for-bit** — parallel is a pure performance option, never a numeric change.
#[derive(Clone, Copy, Debug)]
struct PointContribution {
    score: f64,
    gradient: Vector6<f64>,
    hessian: Matrix6<f64>,
    nearest: f64,
    neighborhood: usize,
    found: bool,
}

/// Compute one source point's [`PointContribution`]. `nbr` is caller-owned neighbor-search scratch
/// (the serial backend reuses one buffer; the parallel backend gives each rayon worker its own).
/// Reads the map and writes only `nbr` + the return value, so it is safe to run concurrently for
/// distinct points.
#[allow(
    clippy::arithmetic_side_effects,
    clippy::allow_attributes,
    reason = "nalgebra fixed-size f64 matrix math"
)]
fn point_contribution(
    map: &VoxelGridMap,
    sp: [f32; 3],
    tp: [f32; 3],
    ad: &AngleDerivatives,
    cfg: &ScoreConfig,
    nbr: &mut Vec<usize>,
) -> PointContribution {
    nbr.clear();
    map.radius_search(tp, cfg.resolution, MAX_NEIGHBORS, nbr);
    if nbr.is_empty() {
        return PointContribution {
            score: 0.0,
            gradient: Vector6::zeros(),
            hessian: Matrix6::zeros(),
            nearest: 0.0,
            neighborhood: 0,
            found: false,
        };
    }
    let pd = compute_point_derivatives(&vec3(sp), ad);
    let x_trans = vec3(tp);
    let mut gradient = Vector6::zeros();
    let mut hessian = Matrix6::zeros();
    let mut score = 0.0_f64;
    let mut nearest = 0.0_f64;
    for &li in nbr.iter() {
        if let Some(leaf) = map.leaf(li) {
            let mean = Vector3::new(leaf.mean[0], leaf.mean[1], leaf.mean[2]);
            let c_inv = mat3(&leaf.icov);
            let s = update_derivatives(
                &(x_trans - mean),
                &c_inv,
                &pd,
                cfg.gauss,
                &mut gradient,
                &mut hessian,
            );
            score += s;
            if s > nearest {
                nearest = s;
            }
        }
    }
    PointContribution {
        score,
        gradient,
        hessian,
        nearest,
        neighborhood: nbr.len(),
        found: true,
    }
}

/// Running reduction of per-point contributions (the C++ thread accumulators). Both backends fold
/// into this in point-index order, so the assembled result is independent of the backend.
struct Reduction {
    score: f64,
    gradient: Vector6<f64>,
    hessian: Matrix6<f64>,
    nearest_voxel_score: f64,
    found: usize,
    total_neighborhood: usize,
}

impl Reduction {
    fn new() -> Self {
        Self {
            score: 0.0,
            gradient: Vector6::zeros(),
            hessian: Matrix6::zeros(),
            nearest_voxel_score: 0.0,
            found: 0,
            total_neighborhood: 0,
        }
    }

    #[allow(
        clippy::arithmetic_side_effects,
        clippy::allow_attributes,
        reason = "nalgebra fixed-size f64 matrix sums; integer counters use saturating_add"
    )]
    fn add(&mut self, c: &PointContribution) {
        self.score += c.score;
        self.gradient += c.gradient;
        self.hessian += c.hessian;
        self.nearest_voxel_score += c.nearest;
        if c.found {
            self.found = self.found.saturating_add(1);
        }
        self.total_neighborhood = self.total_neighborhood.saturating_add(c.neighborhood);
    }
}

/// Apply the optional regularization and assemble [`Derivatives`] from a completed reduction. Shared
/// by both backends; runs after the reduce, so it is backend-independent.
#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::allow_attributes,
    reason = "numeric kernel: nalgebra fixed-size math + deliberate C++-parity f32 regularization casts"
)]
fn finalize(
    mut red: Reduction,
    p: &Vector6<f64>,
    cfg: &ScoreConfig,
    source_len: usize,
) -> Derivatives {
    if let Some(r) = cfg.reg {
        // Mirrors C++ 486-519: float arithmetic, with sin/cos computed in f64 then cast to f32
        // (C++ `static_cast<float>(sin(p(5,0)))`). No-op when scale_factor == 0.
        let dx = r.pose_xy[0] - p[0] as f32;
        let dy = r.pose_xy[1] - p[1] as f32;
        let sin_yaw = libm::sin(p[5]) as f32;
        let cos_yaw = libm::cos(p[5]) as f32;
        let longitudinal = dy * sin_yaw + dx * cos_yaw;
        let w = red.total_neighborhood as f32;
        let sf = r.scale_factor;

        red.score += f64::from(-sf * w * longitudinal * longitudinal);
        red.gradient[0] += f64::from(sf * w * 2.0 * cos_yaw * longitudinal);
        red.gradient[1] += f64::from(sf * w * 2.0 * sin_yaw * longitudinal);
        let h01 = f64::from(-sf * w * 2.0 * cos_yaw * sin_yaw);
        red.hessian[(0, 0)] += f64::from(-sf * w * 2.0 * cos_yaw * cos_yaw);
        red.hessian[(0, 1)] += h01;
        red.hessian[(1, 1)] += f64::from(-sf * w * 2.0 * sin_yaw * sin_yaw);
        red.hessian[(1, 0)] += h01;
    }

    let nearest_voxel_likelihood = if red.found != 0 {
        red.nearest_voxel_score / red.found as f64
    } else {
        0.0
    };
    let transform_probability = if source_len == 0 {
        0.0
    } else {
        red.score / source_len as f64
    };

    Derivatives {
        score: red.score,
        gradient: red.gradient,
        hessian: red.hessian,
        nearest_voxel_likelihood,
        transform_probability,
    }
}

/// Score + gradient + Hessian over the source cloud (`computeDerivatives`), **serial** backend.
/// `source` are the original points (transform derivatives); `trans_cloud` the same points already
/// transformed by the current pose (neighbor search + `x_trans`); `p` the 6-vector of that pose;
/// `cfg` the neighbor radius / Gaussian constants / optional regularization (fixed across the loop).
///
/// WCET contract (RT-critical; the predictable baseline; see `porting_notes/ndt_wcet_audit.md`):
/// `O(P · K)` where `P` = source points (caller-bounded by downsampling) and `K` = neighbors/point ≤
/// `MAX_NEIGHBORS` (the `radius_search` cap); reuses `ws.neighbor_idx` → **no allocation** (measured);
/// no panic/block/logging; per-point math is fixed-size `O(1)`. The map is read-only.
#[must_use]
pub fn compute_derivatives(
    map: &VoxelGridMap,
    source: &[[f32; 3]],
    trans_cloud: &[[f32; 3]],
    p: &Vector6<f64>,
    cfg: &ScoreConfig,
    ws: &mut AlignWorkspace,
) -> Derivatives {
    let ad = compute_angle_derivatives(p);
    let mut red = Reduction::new();
    // Per-point-local contributions, folded in point-index order (zip stops at the shorter cloud).
    for (&tp, &sp) in trans_cloud.iter().zip(source.iter()) {
        let c = point_contribution(map, sp, tp, &ad, cfg, &mut ws.neighbor_idx);
        red.add(&c);
    }
    finalize(red, p, cfg, source.len())
}

/// **Parallel** (rayon) backend for [`compute_derivatives`] — bit-identical to the serial version
/// (per-point contributions collected in point-index order, then folded in the same order). NOT the
/// WCET baseline: it allocates `ws.contribs` + per-worker neighbor buffers and adds scheduling
/// jitter, so it is a throughput option only. `align` selects it when `params.num_threads > 1`.
#[cfg(feature = "parallel")]
#[must_use]
pub fn compute_derivatives_parallel(
    map: &VoxelGridMap,
    source: &[[f32; 3]],
    trans_cloud: &[[f32; 3]],
    p: &Vector6<f64>,
    cfg: &ScoreConfig,
    ws: &mut AlignWorkspace,
) -> Derivatives {
    use rayon::prelude::*;

    let ad = compute_angle_derivatives(p);
    // `IndexedParallelIterator` preserves order, so `contribs` is in point-index order and the fold
    // below matches the serial backend bit-for-bit. `map_init` gives each worker a reusable buffer.
    trans_cloud
        .par_iter()
        .zip(source.par_iter())
        .map_init(
            || Vec::<usize>::with_capacity(MAX_NEIGHBORS),
            |nbr, (&tp, &sp)| point_contribution(map, sp, tp, &ad, cfg, nbr),
        )
        .collect_into_vec(&mut ws.contribs);

    let mut red = Reduction::new();
    for c in &ws.contribs {
        red.add(c);
    }
    finalize(red, p, cfg, source.len())
}

/// Score-only transformation probability: sum over all neighbor cells of the per-cell score,
/// divided by the cloud size (`calculateTransformationProbability`).
#[allow(
    clippy::arithmetic_side_effects,
    clippy::as_conversions,
    clippy::cast_precision_loss,
    clippy::allow_attributes,
    reason = "nalgebra f64 vector math; the score/len average is a deliberate usize->f64 cast"
)]
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
        map.radius_search(tp, resolution, MAX_NEIGHBORS, &mut ws.neighbor_idx);
        if ws.neighbor_idx.is_empty() {
            continue;
        }
        let x_trans = vec3(tp);
        // Per-point-local sum, then add to the running score — matches `compute_derivatives`'
        // grouping so the two agree (and so serial == parallel stays bit-for-bit).
        let mut pt = 0.0_f64;
        for &li in &ws.neighbor_idx {
            if let Some(leaf) = map.leaf(li) {
                let mean = Vector3::new(leaf.mean[0], leaf.mean[1], leaf.mean[2]);
                pt += score_increment(&(x_trans - mean), &mat3(&leaf.icov), gauss);
            }
        }
        score += pt;
    }
    if trans_cloud.is_empty() {
        0.0
    } else {
        score / trans_cloud.len() as f64
    }
}

/// Per-point maximum cell score, averaged over points that found a neighbor
/// (`calculateNearestVoxelTransformationLikelihood`).
#[allow(
    clippy::arithmetic_side_effects,
    clippy::as_conversions,
    clippy::cast_precision_loss,
    clippy::allow_attributes,
    reason = "nalgebra f64 vector math; the score/found average is a deliberate usize->f64 cast"
)]
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
        map.radius_search(tp, resolution, MAX_NEIGHBORS, &mut ws.neighbor_idx);
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
        found = found.saturating_add(1);
    }
    if found != 0 {
        nearest_voxel_score / found as f64
    } else {
        0.0
    }
}

/// NDT parameters (`NdtParams` + the `outlier_ratio_` member, default 0.55).
#[derive(Clone, Copy, Debug)]
pub struct NdtParams {
    pub trans_epsilon: f64,
    pub step_size: f64,
    pub resolution: f64,
    pub max_iterations: i32,
    pub outlier_ratio: f64,
    pub regularization: Option<Regularization>,
    /// Worker count for the derivative reduction (mirrors C++ `NdtParams.num_threads`). `> 1` uses
    /// the rayon backend when the `parallel` feature is on; otherwise the serial backend runs. The
    /// result is bit-identical either way, so this only trades WCET predictability for throughput.
    pub num_threads: usize,
}

impl Default for NdtParams {
    fn default() -> Self {
        Self {
            trans_epsilon: 0.1,
            step_size: 0.1,
            resolution: 1.0,
            max_iterations: 35,
            outlier_ratio: 0.55,
            regularization: None,
            num_threads: 1,
        }
    }
}

/// Result of `align` (`NdtResult`). The `Vec`s are reused across calls (`align` clears them).
#[derive(Clone, Debug, Default)]
pub struct AlignResult {
    pub pose: Matrix4<f32>,
    pub transform_probability: f32,
    pub nearest_voxel_likelihood: f32,
    pub iteration_num: i32,
    pub hessian: Matrix6<f64>,
    pub transformation_array: Vec<Matrix4<f32>>,
    pub transform_probability_array: Vec<f32>,
    pub nearest_voxel_likelihood_array: Vec<f32>,
}

/// Solve `H · delta = -gradient` via SVD (mirrors the C++ `JacobiSVD(ComputeFullU|ComputeFullV)`).
/// Returns `None` if the solve fails (singular / non-finite).
fn svd_solve(hessian: &Matrix6<f64>, neg_gradient: &Vector6<f64>) -> Option<Vector6<f64>> {
    SVD::new(*hessian, true, true)
        .solve(neg_gradient, 1e-9)
        .ok()
}

/// Transform the cloud by `p` and compute the derivatives there. Moves `ws.trans_cloud` out for the
/// `compute_derivatives` call (so `ws.neighbor_idx` can be borrowed mutably without aliasing) and
/// puts it back — a buffer move, no allocation.
fn derivatives_at(
    map: &VoxelGridMap,
    source: &[[f32; 3]],
    p: &Vector6<f64>,
    cfg: &ScoreConfig,
    ws: &mut AlignWorkspace,
    parallel: bool,
) -> Derivatives {
    let mut trans = core::mem::take(&mut ws.trans_cloud);
    transform_cloud_f32(p, source, &mut trans);
    // `parallel` selects the rayon backend (bit-identical to serial) when the feature is built.
    #[cfg(feature = "parallel")]
    let d = if parallel {
        compute_derivatives_parallel(map, source, &trans, p, cfg, ws)
    } else {
        compute_derivatives(map, source, &trans, p, cfg, ws)
    };
    #[cfg(not(feature = "parallel"))]
    let d = {
        let _ = parallel; // no rayon backend in this build; always serial
        compute_derivatives(map, source, &trans, p, cfg, ws)
    };
    ws.trans_cloud = trans;
    d
}

/// NDT alignment (`align` / `computeTransformation` + the default-path `computeStepLengthMT`,
/// `use_line_search = false`). `guess` is the initial pose (the C++ `Matrix4f` guess); `source` is
/// the original cloud. Fills `out` (its `Vec`s are reused). The per-iteration cloud transform is f32
/// (C++ `Matrix4f` pipeline).
///
/// WCET contract (RT-critical; serial; see `porting_notes/ndt_wcet_audit.md`):
/// - Outer loop runs at most `params.max_iterations` times (static cap).
/// - Each iteration does one `compute_derivatives` pass (`O(P · K)`: `P` = source points,
///   `K` = neighbors/point) + one 6×6 SVD solve + one f32 cloud transform (`O(P)`).
/// - `K` (neighbors/point) ≤ `MAX_NEIGHBORS` (the `radius_search` cap); `P` must be bounded by the
///   caller (downsample). The kd-tree traversal is worst-case `O(N_leaves)` for adversarial point
///   distributions — an **accepted residual** (benign for physical, roughly-uniform voxel maps).
/// - No panic (the crate's deny-`unwrap`/`expect`/`panic`/`indexing_slicing` lints); no blocking; no
///   logging/formatting; no user callbacks. Only fixed-width float math.
/// - **Zero allocation** per frame after warmup — all buffers (result `Vec`s, `trans_cloud`,
///   `neighbor_idx` bounded by `MAX_NEIGHBORS`) are pre-reserved + reused, and the fixed-size 6×6 SVD
///   is stack-only (measured: `tests/zero_alloc.rs`).
/// - rt-core (this runtime path) vs control-plane (map build/update) — the map is read-only here.
#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::allow_attributes,
    reason = "numeric kernel: nalgebra fixed-size math + deliberate C++-parity f32/usize<->int casts"
)]
pub fn align(
    map: &VoxelGridMap,
    source: &[[f32; 3]],
    guess: &Matrix4<f32>,
    params: &NdtParams,
    ws: &mut AlignWorkspace,
    out: &mut AlignResult,
) {
    let gauss = gauss_constants(params.outlier_ratio, params.resolution);
    let reg = params.regularization.as_ref();
    let cfg = ScoreConfig {
        resolution: params.resolution,
        gauss: &gauss,
        reg,
    };
    // Use the rayon backend when the caller asks for >1 thread (bit-identical to serial; ignored
    // when the `parallel` feature is off).
    let parallel = params.num_threads > 1;
    let len = source.len();

    // Pre-reserve the per-iteration result buffers so no growth occurs after warmup (at most
    // max_iterations+1 entries). These `reserve`s follow `clear()` (len == 0), so they reserve the
    // full capacity and are no-ops once warm. (`ws.trans_cloud` is reserved inside
    // `transform_cloud_f32` after its own clear; `ws.neighbor_idx` is bounded by MAX_NEIGHBORS via
    // `radius_search` and warms on the first frame — reserving here would over-reserve a non-empty
    // buffer.)
    let cap = (params.max_iterations.max(0) as usize).saturating_add(1);
    out.transformation_array.clear();
    out.transform_probability_array.clear();
    out.nearest_voxel_likelihood_array.clear();
    out.transformation_array.reserve(cap);
    out.transform_probability_array.reserve(cap);
    out.nearest_voxel_likelihood_array.reserve(cap);

    // Initial guess -> 6-vector (matches the C++ eulerAngles(0,1,2) extraction for non-gimbal angles).
    let guess_f64 = guess.map(f64::from);
    let mut p = matrix_to_euler(&guess_f64);

    out.transformation_array.push(se3_matrix_f32(&p));

    // Derivatives at the initial guess.
    let mut d = derivatives_at(map, source, &p, &cfg, ws, parallel);
    out.transform_probability_array
        .push(d.transform_probability as f32);
    out.nearest_voxel_likelihood_array
        .push(d.nearest_voxel_likelihood as f32);
    let mut score = d.score;

    let mut iter: i32 = 0;
    let mut nvl = d.nearest_voxel_likelihood;
    // Newton descent direction (negative for maximization), C++ line 307-310; exits if the SVD solve
    // fails (singular / non-finite).
    while let Some(delta_full) = svd_solve(&d.hessian, &(-d.gradient)) {
        let delta_norm = delta_full.norm();

        if delta_norm == 0.0 || delta_norm.is_nan() {
            break;
        }

        // Default-path step length (computeStepLengthMT, use_line_search = false).
        let mut dir = delta_full / delta_norm; // normalize
        let d_phi_0 = -(d.gradient.dot(&dir));
        if d_phi_0 >= 0.0 {
            if d_phi_0 == 0.0 {
                break;
            }
            dir = -dir;
        }
        let step_min = params.trans_epsilon / 2.0;
        let a_t = delta_norm.clamp(step_min, params.step_size);

        let x_t = p + dir * a_t;
        d = derivatives_at(map, source, &x_t, &cfg, ws, parallel);
        out.transform_probability_array
            .push(d.transform_probability as f32);
        out.nearest_voxel_likelihood_array
            .push(d.nearest_voxel_likelihood as f32);
        score = d.score;
        nvl = d.nearest_voxel_likelihood;

        out.transformation_array.push(se3_matrix_f32(&x_t));
        p = x_t;
        iter = iter.saturating_add(1);

        if iter >= params.max_iterations || a_t.abs() < params.trans_epsilon {
            break;
        }
    }

    out.pose = se3_matrix_f32(&p);
    out.iteration_num = iter;
    out.hessian = d.hessian;
    out.nearest_voxel_likelihood = nvl as f32;
    out.transform_probability = if len == 0 {
        0.0
    } else {
        (score / len as f64) as f32
    };
}

// ---- C ABI: full align entry (test-scope; the start of the E6 engine FFI) ----

/// Inputs to the align FFI. Point clouds are `len + *const f32` (xyz triples); `guess` is 16 `f32`
/// (row-major 4x4). `regularization_scale == 0` disables regularization.
#[repr(C)]
pub struct AwNdtAlignInput {
    pub target_xyz: *const f32,
    pub n_target: usize,
    pub source_xyz: *const f32,
    pub n_source: usize,
    pub resolution: f64,
    pub step_size: f64,
    pub trans_epsilon: f64,
    pub max_iterations: i32,
    pub outlier_ratio: f64,
    pub regularization_scale: f32,
    pub regularization_pose_x: f32,
    pub regularization_pose_y: f32,
    pub guess: *const f32,
}

/// Output buffers for the align FFI (all optional / null-skippable). `pose` 16, `hessian` 36 (row-
/// major), `transformation_array` holds up to `transforms_cap` poses (16 `f32` each); the true count
/// is written to `transforms_count`.
#[repr(C)]
pub struct AwNdtAlignOutput {
    pub pose: *mut f32,
    pub iteration_num: *mut i32,
    pub transform_probability: *mut f32,
    pub nearest_voxel_likelihood: *mut f32,
    pub hessian: *mut f64,
    pub transformation_array: *mut f32,
    pub transforms_cap: u32,
    pub transforms_count: *mut u32,
}

/// Run NDT `align` from flat C inputs (builds the target `VoxelGridMap` with the C++ defaults
/// `min_points = 6`, `eig_mult = 0.01`, `leaf_size = resolution`).
///
/// # Safety
/// `input`/`output` must be valid pointers (or null → no-op). Each non-null pointer must address the
/// documented length: `target_xyz` `3*n_target` f32, `source_xyz` `3*n_source` f32, `guess` 16,
/// `pose` 16, `hessian` 36, `transformation_array` `transforms_cap*16`. No-op if `target_xyz`,
/// `source_xyz`, or `guess` is null.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::allow_attributes,
    reason = "marshaling fixed-size pose/hessian/transform arrays across the C ABI: nalgebra matrix indexing, bounded counts"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_align(
    input: *const AwNdtAlignInput,
    output: *const AwNdtAlignOutput,
) {
    if input.is_null() || output.is_null() {
        return;
    }
    // SAFETY: both are non-null per the check; the caller guarantees they point to valid structs.
    let (inp, outp) = unsafe { (&*input, &*output) };
    if inp.target_xyz.is_null() || inp.source_xyz.is_null() || inp.guess.is_null() {
        return;
    }

    // SAFETY: caller guarantees the documented cloud / guess lengths.
    let (target, source, guess_arr) = unsafe {
        (
            core::slice::from_raw_parts(inp.target_xyz.cast::<[f32; 3]>(), inp.n_target),
            core::slice::from_raw_parts(inp.source_xyz.cast::<[f32; 3]>(), inp.n_source),
            core::slice::from_raw_parts(inp.guess, 16),
        )
    };

    let mut guess = Matrix4::<f32>::zeros();
    for r in 0..4 {
        for c in 0..4 {
            guess[(r, c)] = guess_arr[(r * 4) + c];
        }
    }

    let mut map = VoxelGridMap::new([inp.resolution; 3], 6, 0.01);
    map.add_target(target, 0);
    map.create_kdtree();

    let reg = if inp.regularization_scale == 0.0 {
        None
    } else {
        Some(Regularization {
            pose_xy: [inp.regularization_pose_x, inp.regularization_pose_y],
            scale_factor: inp.regularization_scale,
        })
    };
    let params = NdtParams {
        trans_epsilon: inp.trans_epsilon,
        step_size: inp.step_size,
        resolution: inp.resolution,
        max_iterations: inp.max_iterations,
        outlier_ratio: inp.outlier_ratio,
        regularization: reg,
        // FFI defaults to the serial backend; wiring a num_threads field through the C ABI is a
        // separate follow-up (bit-for-bit serial==parallel is covered by the Rust-side test).
        num_threads: 1,
    };

    let mut ws = AlignWorkspace::new();
    let mut result = AlignResult::default();
    align(&map, source, &guess, &params, &mut ws, &mut result);

    // SAFETY: each output pointer is either null (skipped) or valid for its documented length.
    unsafe {
        if !outp.pose.is_null() {
            let pose = core::slice::from_raw_parts_mut(outp.pose, 16);
            for r in 0..4 {
                for c in 0..4 {
                    pose[(r * 4) + c] = result.pose[(r, c)];
                }
            }
        }
        if !outp.iteration_num.is_null() {
            *outp.iteration_num = result.iteration_num;
        }
        if !outp.transform_probability.is_null() {
            *outp.transform_probability = result.transform_probability;
        }
        if !outp.nearest_voxel_likelihood.is_null() {
            *outp.nearest_voxel_likelihood = result.nearest_voxel_likelihood;
        }
        if !outp.hessian.is_null() {
            let h = core::slice::from_raw_parts_mut(outp.hessian, 36);
            for r in 0..6 {
                for c in 0..6 {
                    h[(r * 6) + c] = result.hessian[(r, c)];
                }
            }
        }
        if !outp.transformation_array.is_null() && !outp.transforms_count.is_null() {
            let cap = outp.transforms_cap as usize;
            let buf = core::slice::from_raw_parts_mut(outp.transformation_array, cap * 16);
            for (k, m) in result.transformation_array.iter().take(cap).enumerate() {
                for r in 0..4 {
                    for c in 0..4 {
                        buf[(k * 16) + (r * 4) + c] = m[(r, c)];
                    }
                }
            }
            *outp.transforms_count = result.transformation_array.len() as u32;
        }
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::unreadable_literal,
    clippy::needless_range_loop,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::as_conversions,
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::borrow_as_ptr,
    unsafe_code,
    clippy::allow_attributes,
    reason = "test code"
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
        let d = compute_derivatives(
            &map,
            &source,
            &trans,
            &p,
            &ScoreConfig {
                resolution: 1.0,
                gauss: &g,
                reg: None,
            },
            &mut ws,
        );
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
        let d = compute_derivatives(
            &map,
            &source,
            &trans,
            &p,
            &ScoreConfig {
                resolution: 1.0,
                gauss: &g,
                reg: None,
            },
            &mut ws,
        );

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
        let d = compute_derivatives(
            &map,
            &[],
            &[],
            &p,
            &ScoreConfig {
                resolution: 1.0,
                gauss: &g,
                reg: None,
            },
            &mut ws,
        );
        assert_eq!(d.score, 0.0);
        assert_eq!(d.transform_probability, 0.0);
        assert_eq!(d.nearest_voxel_likelihood, 0.0);

        // A point far from any voxel contributes nothing and is not counted.
        let source = alloc::vec![[100.0_f32, 100.0, 100.0]];
        let trans = source.clone();
        let d2 = compute_derivatives(
            &map,
            &source,
            &trans,
            &p,
            &ScoreConfig {
                resolution: 1.0,
                gauss: &g,
                reg: None,
            },
            &mut ws,
        );
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

        let d_none = compute_derivatives(
            &map,
            &source,
            &trans,
            &p,
            &ScoreConfig {
                resolution: 1.0,
                gauss: &g,
                reg: None,
            },
            &mut ws,
        );
        let reg = Regularization {
            pose_xy: [0.0, 0.0],
            scale_factor: 0.0,
        };
        let d_zero = compute_derivatives(
            &map,
            &source,
            &trans,
            &p,
            &ScoreConfig {
                resolution: 1.0,
                gauss: &g,
                reg: Some(&reg),
            },
            &mut ws,
        );
        assert_eq!(d_none.score, d_zero.score);
        assert_eq!(d_none.gradient, d_zero.gradient);
        assert_eq!(d_none.hessian, d_zero.hessian);
    }

    // ---- align (E4d) ----

    // Target points spread in 3D (constrains translation), built into a map; the originals are
    // returned so tests can synthesize a source by transforming them.
    fn spread_target() -> (VoxelGridMap, alloc::vec::Vec<[f32; 3]>) {
        let mut pts: alloc::vec::Vec<[f32; 3]> = alloc::vec::Vec::new();
        for &(cx, cy, cz) in &[
            (0.5_f32, 0.5_f32, 0.5_f32),
            (4.5, 0.5, 0.5),
            (0.5, 4.5, 0.5),
            (0.5, 0.5, 4.5),
            (4.5, 4.5, 4.5),
        ] {
            pts.extend(dense_cluster(cx, cy, cz));
        }
        let mut map = VoxelGridMap::new([1.0, 1.0, 1.0], 6, 0.01);
        map.add_target(&pts, 0);
        map.create_kdtree();
        (map, pts)
    }

    fn tight_params() -> NdtParams {
        NdtParams {
            trans_epsilon: 1e-3,
            step_size: 0.2,
            resolution: 1.0,
            max_iterations: 50,
            outlier_ratio: 0.55,
            regularization: None,
            num_threads: 1,
        }
    }

    // ParReduce (E4e): the rayon backend must equal the serial backend BIT-FOR-BIT — enabling
    // parallelism is a pure performance change. Exact `==` on every field (not approx).
    #[cfg(feature = "parallel")]
    #[test]
    fn serial_and_parallel_compute_derivatives_are_bit_identical() {
        let (map, source) = spread_target(); // 40 points across 5 clusters -> rayon splits the work
        let g = gauss_constants(0.55, 1.0);
        let p = Vector6::new(0.1, -0.05, 0.03, 0.02, -0.01, 0.04);
        let trans = transform_cloud(&source, &p);
        let cfg = ScoreConfig {
            resolution: 1.0,
            gauss: &g,
            reg: None,
        };
        let mut ws = AlignWorkspace::new();
        let s = compute_derivatives(&map, &source, &trans, &p, &cfg, &mut ws);
        let par = compute_derivatives_parallel(&map, &source, &trans, &p, &cfg, &mut ws);

        assert_eq!(s.score, par.score, "score");
        assert_eq!(s.transform_probability, par.transform_probability, "tp");
        assert_eq!(
            s.nearest_voxel_likelihood, par.nearest_voxel_likelihood,
            "nvl"
        );
        for i in 0..6 {
            assert_eq!(s.gradient[i], par.gradient[i], "gradient[{i}]");
        }
        for i in 0..6 {
            for j in 0..6 {
                assert_eq!(s.hessian[(i, j)], par.hessian[(i, j)], "hessian[{i}][{j}]");
            }
        }
    }

    // The same property at the `align` level: num_threads = 1 (serial) vs 8 (rayon) must produce an
    // identical AlignResult (pose / iterations / Hessian / per-iteration arrays).
    #[cfg(feature = "parallel")]
    #[test]
    fn align_serial_equals_parallel_bit_identical() {
        let (map, target) = spread_target();
        let t = [0.2_f32, -0.15, 0.1];
        let source: alloc::vec::Vec<[f32; 3]> = target
            .iter()
            .map(|p| [p[0] + t[0], p[1] + t[1], p[2] + t[2]])
            .collect();
        let guess = Matrix4::identity();

        let run = |num_threads: usize| {
            let mut params = tight_params();
            params.num_threads = num_threads;
            let mut ws = AlignWorkspace::new();
            let mut out = AlignResult::default();
            align(&map, &source, &guess, &params, &mut ws, &mut out);
            out
        };
        let serial = run(1);
        let parallel = run(8);

        assert_eq!(serial.iteration_num, parallel.iteration_num, "iterations");
        assert_eq!(serial.pose, parallel.pose, "pose");
        assert_eq!(serial.hessian, parallel.hessian, "hessian");
        assert_eq!(
            serial.transform_probability_array, parallel.transform_probability_array,
            "tp array"
        );
        assert_eq!(
            serial.nearest_voxel_likelihood_array, parallel.nearest_voxel_likelihood_array,
            "nvl array"
        );
    }

    // Recover-known-transform oracle: source = target + t; align maps source back, so the recovered
    // pose's translation ≈ -t (and rotation ≈ identity). Strong independent oracle.
    #[test]
    fn align_recovers_known_translation() {
        let (map, target) = spread_target();
        let t = [0.15_f32, -0.10, 0.05];
        let source: alloc::vec::Vec<[f32; 3]> = target
            .iter()
            .map(|p| [p[0] + t[0], p[1] + t[1], p[2] + t[2]])
            .collect();

        let mut ws = AlignWorkspace::new();
        let mut out = AlignResult::default();
        align(
            &map,
            &source,
            &Matrix4::identity(),
            &tight_params(),
            &mut ws,
            &mut out,
        );

        assert!(
            out.iteration_num <= 50,
            "did not stop within max_iterations"
        );
        // Final translation ≈ -t.
        assert!(
            (out.pose[(0, 3)] - (-t[0])).abs() < 3e-2,
            "tx {}",
            out.pose[(0, 3)]
        );
        assert!(
            (out.pose[(1, 3)] - (-t[1])).abs() < 3e-2,
            "ty {}",
            out.pose[(1, 3)]
        );
        assert!(
            (out.pose[(2, 3)] - (-t[2])).abs() < 3e-2,
            "tz {}",
            out.pose[(2, 3)]
        );
        // Rotation block ≈ identity.
        for i in 0..3 {
            for j in 0..3 {
                let want = if i == j { 1.0 } else { 0.0 };
                assert!((out.pose[(i, j)] - want).abs() < 2e-2, "R[{i}][{j}]");
            }
        }
        // Score improved from the (misaligned) guess to convergence.
        let tpa = &out.transform_probability_array;
        assert!(
            tpa.len() == out.iteration_num as usize + 1,
            "array length == iter+1"
        );
        assert!(
            *tpa.last().unwrap() >= *tpa.first().unwrap(),
            "transform_probability did not improve: {tpa:?}"
        );
    }

    // Already aligned (source == target): align stays at identity and converges immediately.
    #[test]
    fn align_identity_when_already_aligned() {
        let (map, target) = spread_target();
        let mut ws = AlignWorkspace::new();
        let mut out = AlignResult::default();
        align(
            &map,
            &target,
            &Matrix4::identity(),
            &tight_params(),
            &mut ws,
            &mut out,
        );
        for i in 0..3 {
            assert!(
                out.pose[(i, 3)].abs() < 2e-2,
                "translation drifted: {}",
                out.pose[(i, 3)]
            );
        }
        assert!(out.transform_probability > 0.0);
    }

    // FFI shim must equal the pure `align` on identical inputs (catches struct-layout / flat-buffer /
    // row-major marshaling bugs); null input/output must be a no-op. llvm-cov only sees Rust tests,
    // so this covers the FFI shim that the C++ differential test exercises out-of-process.
    #[test]
    fn ffi_ndt_align_matches_pure() {
        let (_, target) = spread_target();
        let t = [0.2_f32, -0.15, 0.1];
        let source: alloc::vec::Vec<[f32; 3]> = target
            .iter()
            .map(|p| [p[0] + t[0], p[1] + t[1], p[2] + t[2]])
            .collect();
        let params = NdtParams {
            trans_epsilon: 0.01,
            step_size: 0.1,
            resolution: 2.0,
            max_iterations: 30,
            outlier_ratio: 0.55,
            regularization: None,
            num_threads: 1,
        };

        // Pure reference: same map the FFI builds internally (leaf = resolution, min 6, eig 0.01).
        let mut map = VoxelGridMap::new([2.0, 2.0, 2.0], 6, 0.01);
        map.add_target(&target, 0);
        map.create_kdtree();
        let mut ws = AlignWorkspace::new();
        let mut pure = AlignResult::default();
        align(
            &map,
            &source,
            &Matrix4::identity(),
            &params,
            &mut ws,
            &mut pure,
        );

        // FFI call.
        let target_flat: alloc::vec::Vec<f32> =
            target.iter().flat_map(|p| p.iter().copied()).collect();
        let source_flat: alloc::vec::Vec<f32> =
            source.iter().flat_map(|p| p.iter().copied()).collect();
        let guess16: [f32; 16] = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let input = AwNdtAlignInput {
            target_xyz: target_flat.as_ptr(),
            n_target: target.len(),
            source_xyz: source_flat.as_ptr(),
            n_source: source.len(),
            resolution: 2.0,
            step_size: 0.1,
            trans_epsilon: 0.01,
            max_iterations: 30,
            outlier_ratio: 0.55,
            regularization_scale: 0.0,
            regularization_pose_x: 0.0,
            regularization_pose_y: 0.0,
            guess: guess16.as_ptr(),
        };
        let mut pose = [0.0_f32; 16];
        let mut iter = 0_i32;
        let mut tp = 0.0_f32;
        let mut nvl = 0.0_f32;
        let mut hess = [0.0_f64; 36];
        let mut transforms = [0.0_f32; 64 * 16];
        let mut count = 0_u32;
        let output = AwNdtAlignOutput {
            pose: pose.as_mut_ptr(),
            iteration_num: &mut iter,
            transform_probability: &mut tp,
            nearest_voxel_likelihood: &mut nvl,
            hessian: hess.as_mut_ptr(),
            transformation_array: transforms.as_mut_ptr(),
            transforms_cap: 64,
            transforms_count: &mut count,
        };
        // SAFETY: all pointers are valid for their documented lengths.
        unsafe { autoware_ndt_scan_matcher_rs_ndt_align(&input, &output) };

        // Same code path -> exact equality.
        assert_eq!(iter, pure.iteration_num);
        assert_eq!(tp, pure.transform_probability);
        assert_eq!(nvl, pure.nearest_voxel_likelihood);
        assert_eq!(count as usize, pure.transformation_array.len());
        for r in 0..4 {
            for c in 0..4 {
                assert_eq!(pose[(r * 4) + c], pure.pose[(r, c)]);
            }
        }
        for r in 0..6 {
            for c in 0..6 {
                assert_eq!(hess[(r * 6) + c], pure.hessian[(r, c)]);
            }
        }

        // Null input / output must be a no-op (no crash, no write).
        // SAFETY: null pointers exercise the documented no-op contract.
        unsafe {
            autoware_ndt_scan_matcher_rs_ndt_align(core::ptr::null(), &output);
            autoware_ndt_scan_matcher_rs_ndt_align(&input, core::ptr::null());
        }
    }
}
