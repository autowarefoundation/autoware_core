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

//! Single voxel-grid covariance (NDT leaves), ported from `MultiVoxelGridCovariance`
//! (`src/ndt_omp/multi_voxel_grid_covariance_omp_impl.hpp`). Builds per-voxel mean / covariance /
//! eigenvalue-regularized inverse covariance. `no_std` + `alloc`; nalgebra internal.
//!
//! Multi-grid id-keyed add/remove and the kdtree/radiusSearch are intentionally out of scope here
//! (E2b / E3); the C++ engine keeps using its own grid until the full engine swap.

// Numeric kernel: nalgebra f64 matrix operators are the float-math domain the integer-overflow lint
// targets; the voxel-index integer math below uses explicit checked ops where overflow could occur.
#![allow(clippy::arithmetic_side_effects)]
// f32<->f64<->i64 conversions are inherent to voxelization/accumulation; the int32 voxel-count guard
// bounds the indices so the floor->i64 narrowing cannot truncate meaningfully.
#![allow(clippy::as_conversions, clippy::cast_possible_truncation, clippy::cast_precision_loss)]
// Numeric kernel: indexing into fixed `[_; 3]` arrays with constant / `0..3` indices is statically
// in-bounds, and x/y/z single-char names are the natural geometry notation.
#![allow(clippy::indexing_slicing, clippy::many_single_char_names)]

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use nalgebra::{Matrix3, Vector3};

const DEFAULT_MIN_POINTS_PER_VOXEL: i32 = 6;

/// A finalized NDT leaf: point count, 3D mean, and row-major inverse covariance.
pub struct Leaf {
    pub n: i32,
    pub mean: [f64; 3],
    pub icov: [f64; 9],
}

struct Accumulator {
    n: i64,
    sum: Vector3<f64>,
    sum_outer: Matrix3<f64>,
}

impl Accumulator {
    fn new() -> Self {
        Self { n: 0, sum: Vector3::zeros(), sum_outer: Matrix3::zeros() }
    }
    fn add(&mut self, p: Vector3<f64>) {
        self.n += 1;
        self.sum += p;
        self.sum_outer += p * p.transpose();
    }
}

/// A built single voxel grid with point->leaf lookup by voxel id.
pub struct VoxelGrid {
    leaves: Vec<Leaf>,
    index: BTreeMap<i64, usize>,
    inverse_leaf_size: [f64; 3],
    bbox_min: [i64; 3],
    div_mul: [i64; 3],
}

/// floor(x) as i64. The caller bounds x via the bounding-box / int32 voxel-count guard.
fn floor_i64(x: f64) -> i64 {
    libm::floor(x) as i64
}

impl VoxelGrid {
    /// Voxel id for a point under this grid's bbox/leaf layout, or `None` on index overflow.
    fn leaf_id(&self, x: f64, y: f64, z: f64) -> Option<i64> {
        let ijk = [
            floor_i64(x * self.inverse_leaf_size[0]) - self.bbox_min[0],
            floor_i64(y * self.inverse_leaf_size[1]) - self.bbox_min[1],
            floor_i64(z * self.inverse_leaf_size[2]) - self.bbox_min[2],
        ];
        let a = ijk[0].checked_mul(self.div_mul[0])?;
        let b = ijk[1].checked_mul(self.div_mul[1])?;
        let c = ijk[2].checked_mul(self.div_mul[2])?;
        a.checked_add(b)?.checked_add(c)
    }

    /// Build a voxel grid from `points`, mirroring `MultiVoxelGridCovariance::apply_filter`.
    #[must_use]
    pub fn build(points: &[[f32; 3]], leaf_size: [f64; 3], min_points: i32, eig_mult: f64) -> Self {
        let inverse_leaf_size =
            [1.0 / leaf_size[0], 1.0 / leaf_size[1], 1.0 / leaf_size[2]];

        let empty = Self {
            leaves: Vec::new(),
            index: BTreeMap::new(),
            inverse_leaf_size,
            bbox_min: [0; 3],
            div_mul: [1, 1, 1],
        };

        // Bounding box over finite points.
        let mut min_p = [f64::INFINITY; 3];
        let mut max_p = [f64::NEG_INFINITY; 3];
        let mut any = false;
        for &[px, py, pz] in points {
            if !(px.is_finite() && py.is_finite() && pz.is_finite()) {
                continue;
            }
            let p = [f64::from(px), f64::from(py), f64::from(pz)];
            for d in 0..3 {
                if p[d] < min_p[d] {
                    min_p[d] = p[d];
                }
                if p[d] > max_p[d] {
                    max_p[d] = p[d];
                }
            }
            any = true;
        }
        if !any {
            return empty;
        }

        // Reject if the voxel count would overflow int32 (matches the C++ guard).
        let mut div_b = [0_i64; 3];
        let mut voxel_count: i64 = 1;
        for d in 0..3 {
            let span = floor_i64(max_p[d] * inverse_leaf_size[d])
                - floor_i64(min_p[d] * inverse_leaf_size[d])
                + 1;
            div_b[d] = span;
            match voxel_count.checked_mul(span) {
                Some(v) if v <= i64::from(i32::MAX) => voxel_count = v,
                _ => return empty,
            }
        }

        let bbox_min = [
            floor_i64(min_p[0] * inverse_leaf_size[0]),
            floor_i64(min_p[1] * inverse_leaf_size[1]),
            floor_i64(min_p[2] * inverse_leaf_size[2]),
        ];
        let div_mul = [1, div_b[0], div_b[0] * div_b[1]];

        let mut grid = Self { leaves: Vec::new(), index: BTreeMap::new(), inverse_leaf_size, bbox_min, div_mul };

        // First pass: accumulate per leaf (ordered map mirrors std::map<int64, Leaf>).
        let mut acc: BTreeMap<i64, Accumulator> = BTreeMap::new();
        for &[px, py, pz] in points {
            if !(px.is_finite() && py.is_finite() && pz.is_finite()) {
                continue;
            }
            let (x, y, z) = (f64::from(px), f64::from(py), f64::from(pz));
            if let Some(id) = grid.leaf_id(x, y, z) {
                acc.entry(id).or_insert_with(Accumulator::new).add(Vector3::new(x, y, z));
            }
        }

        let min_points = if min_points <= 0 { DEFAULT_MIN_POINTS_PER_VOXEL } else { min_points };

        // Second pass: finalize leaf params.
        for (id, a) in &acc {
            if a.n < i64::from(min_points) {
                continue;
            }
            let n_f = a.n as f64;
            let mean = a.sum / n_f;
            // The C++ `Leaf` ctor initializes `cov_` to Identity (not Zero), so the accumulated
            // `Σppᵀ` yields `(Σppᵀ + I - Σp·meanᵀ)/(n-1)` = sample_cov + I/(n-1). Replicate that
            // `+ I` for behavioral equivalence (see multi_voxel_grid_covariance_omp.h Leaf()).
            let cov = (a.sum_outer + Matrix3::identity() - a.sum * mean.transpose()) / (n_f - 1.0);
            if let Some(icov) = compute_icov(&cov, eig_mult) {
                let idx = grid.leaves.len();
                grid.leaves.push(Leaf {
                    n: a.n.min(i64::from(i32::MAX)) as i32,
                    mean: [mean.x, mean.y, mean.z],
                    icov,
                });
                grid.index.insert(*id, idx);
            }
        }
        grid
    }

    /// Leaf whose voxel contains `point`, if any (and it passed the point/eigenvalue filters).
    #[must_use]
    pub fn leaf_at(&self, point: [f32; 3]) -> Option<&Leaf> {
        let [px, py, pz] = point;
        if !(px.is_finite() && py.is_finite() && pz.is_finite()) {
            return None;
        }
        let id = self.leaf_id(f64::from(px), f64::from(py), f64::from(pz))?;
        self.index.get(&id).and_then(|&i| self.leaves.get(i))
    }
}

/// Eigenvalue-regularized inverse covariance (Magnusson eq 6.11), or `None` if the voxel is
/// degenerate (negative/zero eigenvalue or non-finite inverse). Mirrors `computeLeafParams`.
fn compute_icov(cov: &Matrix3<f64>, eig_mult: f64) -> Option<[f64; 9]> {
    let se = cov.symmetric_eigen();
    // nalgebra does not guarantee ordering; sort ascending to match Eigen's SelfAdjointEigenSolver.
    let mut order = [0_usize, 1, 2];
    order.sort_by(|&i, &j| se.eigenvalues[i].total_cmp(&se.eigenvalues[j]));
    let mut evals = [se.eigenvalues[order[0]], se.eigenvalues[order[1]], se.eigenvalues[order[2]]];
    let c0: Vector3<f64> = se.eigenvectors.column(order[0]).into_owned();
    let c1: Vector3<f64> = se.eigenvectors.column(order[1]).into_owned();
    let c2: Vector3<f64> = se.eigenvectors.column(order[2]).into_owned();
    let evecs = Matrix3::from_columns(&[c0, c1, c2]);

    if evals[0] < 0.0 || evals[1] < 0.0 || evals[2] <= 0.0 {
        return None;
    }

    let mut cov = *cov;
    let min_covar_eigvalue = eig_mult * evals[2];
    if evals[0] < min_covar_eigvalue {
        evals[0] = min_covar_eigvalue;
        if evals[1] < min_covar_eigvalue {
            evals[1] = min_covar_eigvalue;
        }
        let diag = Matrix3::from_diagonal(&Vector3::new(evals[0], evals[1], evals[2]));
        // Eigenvectors are orthonormal, so the inverse is the transpose.
        cov = evecs * diag * evecs.transpose();
    }

    let icov = cov.try_inverse()?;
    if icov.iter().any(|v| !v.is_finite()) {
        return None;
    }
    // Row-major.
    Some([
        icov.m11, icov.m12, icov.m13, icov.m21, icov.m22, icov.m23, icov.m31, icov.m32, icov.m33,
    ])
}

// ---- C ABI (opaque handle; the shape the NDT engine will reuse) ----

/// # Safety
/// `points` points to `n` `[f32;3]` triples, `leaf_size` to 3 `f64`. Returns an owned grid handle
/// (free with `..._voxel_grid_free`), or null if `points`/`leaf_size` is null.
#[allow(unsafe_code)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_voxel_grid_build(
    points: *const f32,
    n: usize,
    leaf_size: *const f64,
    min_points: i32,
    eig_mult: f64,
) -> *mut VoxelGrid {
    if points.is_null() || leaf_size.is_null() {
        return core::ptr::null_mut();
    }
    let Some(flat_len) = n.checked_mul(3) else { return core::ptr::null_mut() };
    // SAFETY: caller guarantees `3*n` f32 at `points` and 3 f64 at `leaf_size`.
    let (pts, ls) = unsafe {
        (
            core::slice::from_raw_parts(points.cast::<[f32; 3]>(), n),
            &*leaf_size.cast::<[f64; 3]>(),
        )
    };
    let _ = flat_len;
    Box::into_raw(Box::new(VoxelGrid::build(pts, *ls, min_points, eig_mult)))
}

/// # Safety
/// `grid` is a handle from `..._voxel_grid_build`; `point` points to 3 `f32`; `mean_out`/`icov_out`
/// to 3 / 9 writable `f64`. Returns true and writes outputs iff a leaf covers `point`.
#[allow(unsafe_code)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_voxel_grid_leaf_at(
    grid: *const VoxelGrid,
    point: *const f32,
    mean_out: *mut f64,
    icov_out: *mut f64,
) -> bool {
    if grid.is_null() || point.is_null() || mean_out.is_null() || icov_out.is_null() {
        return false;
    }
    // SAFETY: valid handle + 3 f32 at `point` per the contract.
    let (grid, p) = unsafe { (&*grid, &*point.cast::<[f32; 3]>()) };
    match grid.leaf_at(*p) {
        Some(leaf) => {
            // SAFETY: `mean_out`/`icov_out` have 3 / 9 f64 per the contract.
            unsafe {
                *mean_out.cast::<[f64; 3]>() = leaf.mean;
                *icov_out.cast::<[f64; 9]>() = leaf.icov;
            }
            true
        }
        None => false,
    }
}

/// # Safety
/// `grid` must be a handle from `..._voxel_grid_build` (or null); it must not be used afterwards.
#[allow(unsafe_code)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_voxel_grid_free(grid: *mut VoxelGrid) {
    if grid.is_null() {
        return;
    }
    // SAFETY: `grid` came from `Box::into_raw` and is dropped exactly once.
    drop(unsafe { Box::from_raw(grid) });
}

#[cfg(test)]
#[allow(clippy::float_cmp, clippy::expect_used, clippy::needless_range_loop)]
mod tests {
    use super::*;

    // A single dense voxel: mean is the point centroid; the inverse covariance is finite.
    #[test]
    fn single_voxel_mean_matches_brute_force() {
        // All within voxel (10, 20, 30) for leaf_size 1.0 (each coord stays in [k, k+1)).
        let pts: alloc::vec::Vec<[f32; 3]> = (0..20)
            .map(|i| {
                let f = i as f32 * 0.01;
                [10.0 + f, 20.0 + f, 30.0 + 0.5 * f]
            })
            .collect();
        let grid = VoxelGrid::build(&pts, [1.0, 1.0, 1.0], 6, 0.01);
        let leaf = grid.leaf_at([10.0, 20.0, 30.0]).expect("leaf");
        // Brute-force mean over all points (they all fall in one voxel given leaf_size 1.0).
        let mut m = [0.0_f64; 3];
        for p in &pts {
            for d in 0..3 {
                m[d] += f64::from(p[d]);
            }
        }
        for d in 0..3 {
            m[d] /= pts.len() as f64;
            assert!((leaf.mean[d] - m[d]).abs() < 1e-9);
        }
        assert_eq!(leaf.n, 20);
    }

    #[test]
    fn sparse_voxel_is_dropped() {
        let pts = [[0.0_f32, 0.0, 0.0], [0.1, 0.1, 0.1]]; // only 2 points < min 6
        let grid = VoxelGrid::build(&pts, [1.0, 1.0, 1.0], 6, 0.01);
        assert!(grid.leaf_at([0.0, 0.0, 0.0]).is_none());
    }

    // Well-conditioned voxel: leaf icov must equal the brute-force sample-covariance inverse.
    #[test]
    fn icov_matches_brute_force_inverse() {
        let mut pts: alloc::vec::Vec<[f32; 3]> = alloc::vec::Vec::new();
        // deterministic spread around (5,5,5), all inside voxel [0,10) for leaf_size 10.
        for i in 0..50_i32 {
            let f = i as f32;
            pts.push([5.0 + 0.2 * (f * 0.7).sin(), 5.0 + 0.15 * (f * 1.3).cos(), 5.0 + 0.1 * (f * 0.31).sin()]);
        }
        let grid = VoxelGrid::build(&pts, [10.0, 10.0, 10.0], 6, 0.01);
        let leaf = grid.leaf_at([5.0, 5.0, 5.0]).expect("leaf");

        // Brute-force sample covariance + inverse.
        let n = pts.len() as f64;
        let mut mean = Vector3::<f64>::zeros();
        for p in &pts {
            mean += Vector3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
        }
        mean /= n;
        let mut cov = Matrix3::<f64>::zeros();
        for p in &pts {
            let d = Vector3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2])) - mean;
            cov += d * d.transpose();
        }
        // Match the C++ Identity-initialized covariance accumulator: sample_cov + I/(n-1).
        cov += Matrix3::identity();
        cov /= n - 1.0;
        let bf_icov = cov.try_inverse().expect("inverse");
        let bf = [
            bf_icov.m11, bf_icov.m12, bf_icov.m13, bf_icov.m21, bf_icov.m22, bf_icov.m23, bf_icov.m31,
            bf_icov.m32, bf_icov.m33,
        ];
        for k in 0..9 {
            assert!((leaf.icov[k] - bf[k]).abs() <= 1e-6 * bf[k].abs() + 1e-9, "k={k} {} vs {}", leaf.icov[k], bf[k]);
        }
    }

    #[test]
    fn empty_cloud_builds_empty_grid() {
        let grid = VoxelGrid::build(&[], [1.0, 1.0, 1.0], 6, 0.01);
        assert!(grid.leaf_at([0.0, 0.0, 0.0]).is_none());
    }
}
