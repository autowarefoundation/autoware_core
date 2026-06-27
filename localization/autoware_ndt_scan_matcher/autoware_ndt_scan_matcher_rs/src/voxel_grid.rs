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
// Numeric kernel. Suppressions are scoped per-function (no module-wide `#![allow]`):
// arithmetic_side_effects = nalgebra f64 float math + checked integer voxel-index math;
// as_conversions/cast_* = deliberate f32<->f64<->i64 voxelization conversions (the int32 voxel-count
// guard bounds the floor->i64 narrowing); indexing_slicing = constant indices into fixed `[_; 3]`;
// many_single_char_names = x/y/z geometry notation.

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use nalgebra::{Matrix3, Vector3};

use crate::kdtree::KdTree;

const DEFAULT_MIN_POINTS_PER_VOXEL: i32 = 6;

/// A finalized NDT leaf: point count, 3D mean, row-major inverse covariance, and the f32 voxel
/// centroid (used to build the kd-tree, matching the C++ `centroid_`).
#[derive(Clone)]
pub struct Leaf {
    pub n: i32,
    pub mean: [f64; 3],
    pub icov: [f64; 9],
    pub centroid: [f32; 3],
}

struct Accumulator {
    n: i64,
    sum: Vector3<f64>,
    sum_outer: Matrix3<f64>,
    centroid_sum: [f32; 3],
}

impl Accumulator {
    fn new() -> Self {
        Self {
            n: 0,
            sum: Vector3::zeros(),
            sum_outer: Matrix3::zeros(),
            centroid_sum: [0.0; 3],
        }
    }
    #[allow(
        clippy::arithmetic_side_effects,
        clippy::allow_attributes,
        reason = "nalgebra f64 vector/matrix accumulation"
    )]
    fn add(&mut self, p: Vector3<f64>, raw: [f32; 3]) {
        self.n = self.n.saturating_add(1);
        self.sum += p;
        self.sum_outer += p * p.transpose();
        self.centroid_sum[0] += raw[0];
        self.centroid_sum[1] += raw[1];
        self.centroid_sum[2] += raw[2];
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
#[allow(
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::allow_attributes,
    reason = "deliberate floor->i64; the caller's int32 voxel-count guard bounds the magnitude"
)]
fn floor_i64(x: f64) -> i64 {
    libm::floor(x) as i64
}

impl VoxelGrid {
    /// Voxel id for a point under this grid's bbox/leaf layout, or `None` on index overflow.
    #[allow(
        clippy::many_single_char_names,
        clippy::allow_attributes,
        reason = "x/y/z coordinates and a/b/c voxel-index lattice terms"
    )]
    fn leaf_id(&self, x: f64, y: f64, z: f64) -> Option<i64> {
        let i = floor_i64(x * self.inverse_leaf_size[0]).checked_sub(self.bbox_min[0])?;
        let j = floor_i64(y * self.inverse_leaf_size[1]).checked_sub(self.bbox_min[1])?;
        let k = floor_i64(z * self.inverse_leaf_size[2]).checked_sub(self.bbox_min[2])?;
        let a = i.checked_mul(self.div_mul[0])?;
        let b = j.checked_mul(self.div_mul[1])?;
        let c = k.checked_mul(self.div_mul[2])?;
        a.checked_add(b)?.checked_add(c)
    }

    /// Build a voxel grid from `points`, mirroring `MultiVoxelGridCovariance::apply_filter`.
    #[must_use]
    #[allow(
        clippy::arithmetic_side_effects,
        clippy::indexing_slicing,
        clippy::as_conversions,
        clippy::cast_possible_truncation,
        clippy::cast_precision_loss,
        clippy::allow_attributes,
        reason = "control-plane map build: nalgebra f64 math; i64 voxel-index arithmetic bounded by the int32 voxel-count guard; deliberate count casts; d ∈ 0..3 indexing of fixed [_; 3]"
    )]
    pub fn build(points: &[[f32; 3]], leaf_size: [f64; 3], min_points: i32, eig_mult: f64) -> Self {
        let inverse_leaf_size = [1.0 / leaf_size[0], 1.0 / leaf_size[1], 1.0 / leaf_size[2]];

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

        let mut grid = Self {
            leaves: Vec::new(),
            index: BTreeMap::new(),
            inverse_leaf_size,
            bbox_min,
            div_mul,
        };

        // First pass: accumulate per leaf (ordered map mirrors std::map<int64, Leaf>).
        let mut acc: BTreeMap<i64, Accumulator> = BTreeMap::new();
        for &[px, py, pz] in points {
            if !(px.is_finite() && py.is_finite() && pz.is_finite()) {
                continue;
            }
            let (x, y, z) = (f64::from(px), f64::from(py), f64::from(pz));
            if let Some(id) = grid.leaf_id(x, y, z) {
                acc.entry(id)
                    .or_insert_with(Accumulator::new)
                    .add(Vector3::new(x, y, z), [px, py, pz]);
            }
        }

        let min_points = if min_points <= 0 {
            DEFAULT_MIN_POINTS_PER_VOXEL
        } else {
            min_points
        };

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
                let n_f32 = a.n as f32;
                let idx = grid.leaves.len();
                grid.leaves.push(Leaf {
                    n: a.n.min(i64::from(i32::MAX)) as i32,
                    mean: [mean.x, mean.y, mean.z],
                    icov,
                    centroid: [
                        a.centroid_sum[0] / n_f32,
                        a.centroid_sum[1] / n_f32,
                        a.centroid_sum[2] / n_f32,
                    ],
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
#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::allow_attributes,
    reason = "nalgebra f64 eigen/matrix math; order[..] ∈ 0..3 indexes the 3-element eigen results"
)]
fn compute_icov(cov: &Matrix3<f64>, eig_mult: f64) -> Option<[f64; 9]> {
    let se = cov.symmetric_eigen();
    // nalgebra does not guarantee ordering; sort ascending to match Eigen's SelfAdjointEigenSolver.
    let mut order = [0_usize, 1, 2];
    order.sort_by(|&i, &j| se.eigenvalues[i].total_cmp(&se.eigenvalues[j]));
    let mut evals = [
        se.eigenvalues[order[0]],
        se.eigenvalues[order[1]],
        se.eigenvalues[order[2]],
    ];
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
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
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
    let Some(flat_len) = n.checked_mul(3) else {
        return core::ptr::null_mut();
    };
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
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
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
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_voxel_grid_free(grid: *mut VoxelGrid) {
    if grid.is_null() {
        return;
    }
    // SAFETY: `grid` came from `Box::into_raw` and is dropped exactly once.
    drop(unsafe { Box::from_raw(grid) });
}

// ---- multi-grid map + kd-tree (the MultiVoxelGridCovariance equivalent) ----

/// Id-keyed collection of per-cloud voxel grids plus a kd-tree over all voxel centroids for radius
/// search. Mirrors `MultiVoxelGridCovariance` (`sid_to_iid_`/`grid_list_` + `KdTreeFLANN`).
pub struct VoxelGridMap {
    leaf_size: [f64; 3],
    min_points: i32,
    eig_mult: f64,
    grids: BTreeMap<u64, VoxelGrid>,
    flat_leaves: Vec<Leaf>,
    kdtree: Option<KdTree>,
}

impl VoxelGridMap {
    #[must_use]
    pub fn new(leaf_size: [f64; 3], min_points: i32, eig_mult: f64) -> Self {
        Self {
            leaf_size,
            min_points,
            eig_mult,
            grids: BTreeMap::new(),
            flat_leaves: Vec::new(),
            kdtree: None,
        }
    }

    /// Build a grid from `points` and register it under `id` (replacing any existing one).
    pub fn add_target(&mut self, points: &[[f32; 3]], id: u64) {
        let grid = VoxelGrid::build(points, self.leaf_size, self.min_points, self.eig_mult);
        self.grids.insert(id, grid);
        self.invalidate();
    }

    pub fn remove_target(&mut self, id: u64) {
        self.grids.remove(&id);
        self.invalidate();
    }

    fn invalidate(&mut self) {
        self.kdtree = None;
        self.flat_leaves.clear();
    }

    /// Flatten all grids' leaves (id order, ↔ C++ `std::map`) and build the kd-tree over centroids.
    pub fn create_kdtree(&mut self) {
        let mut centroids: Vec<[f32; 3]> = Vec::new();
        let mut flat: Vec<Leaf> = Vec::new();
        for grid in self.grids.values() {
            for leaf in &grid.leaves {
                centroids.push(leaf.centroid);
                flat.push(leaf.clone());
            }
        }
        self.kdtree = Some(KdTree::build(&centroids));
        self.flat_leaves = flat;
    }

    /// Flat indices of leaves whose centroid is within `radius` of `point` (needs `create_kdtree`).
    pub fn radius_search(&self, point: [f32; 3], radius: f64, max_nn: usize, out: &mut Vec<usize>) {
        if let Some(kt) = &self.kdtree {
            kt.radius_search(&point, radius, max_nn, out);
        }
    }

    #[must_use]
    pub fn leaf(&self, idx: usize) -> Option<&Leaf> {
        self.flat_leaves.get(idx)
    }
}

/// # Safety
/// `leaf_size` points to 3 readable `f64` (or null -> returns null). Returns an owned handle
/// (free with `..._voxel_grid_map_free`).
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_voxel_grid_map_new(
    leaf_size: *const f64,
    min_points: i32,
    eig_mult: f64,
) -> *mut VoxelGridMap {
    if leaf_size.is_null() {
        return core::ptr::null_mut();
    }
    // SAFETY: caller guarantees 3 f64 at `leaf_size`.
    let ls = unsafe { *leaf_size.cast::<[f64; 3]>() };
    Box::into_raw(Box::new(VoxelGridMap::new(ls, min_points, eig_mult)))
}

/// # Safety
/// `map` is a valid handle; `points` points to `3*n` `f32`. No-op if `map`/`points` is null.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_voxel_grid_map_add_target(
    map: *mut VoxelGridMap,
    points: *const f32,
    n: usize,
    id: u64,
) {
    if map.is_null() || points.is_null() {
        return;
    }
    // SAFETY: valid handle + `n` xyz triples per the contract.
    let (map, pts) = unsafe {
        (
            &mut *map,
            core::slice::from_raw_parts(points.cast::<[f32; 3]>(), n),
        )
    };
    map.add_target(pts, id);
}

/// # Safety
/// `map` is a valid handle (or null). Removes the grid registered under `id`.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_voxel_grid_map_remove_target(
    map: *mut VoxelGridMap,
    id: u64,
) {
    if map.is_null() {
        return;
    }
    // SAFETY: valid handle per the contract.
    unsafe { &mut *map }.remove_target(id);
}

/// # Safety
/// `map` is a valid handle (or null). Builds the kd-tree over current grids' centroids.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_voxel_grid_map_create_kdtree(
    map: *mut VoxelGridMap,
) {
    if map.is_null() {
        return;
    }
    // SAFETY: valid handle per the contract.
    unsafe { &mut *map }.create_kdtree();
}

/// # Safety
/// `map` valid; `point` points to 3 `f32`; `out_idx` to `cap` writable `u32`. Writes up to `cap`
/// leaf indices and returns the total number found (`max_nn == 0` = unlimited).
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[allow(
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::allow_attributes,
    reason = "C ABI count marshaling: u32<->usize at the boundary (writes bounded by cap)"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_voxel_grid_map_radius_search(
    map: *const VoxelGridMap,
    point: *const f32,
    radius: f64,
    max_nn: u32,
    out_idx: *mut u32,
    cap: u32,
) -> u32 {
    if map.is_null() || point.is_null() {
        return 0;
    }
    // SAFETY: valid handle + 3 f32 at `point`.
    let (map, p) = unsafe { (&*map, *point.cast::<[f32; 3]>()) };
    let mut found: Vec<usize> = Vec::new();
    map.radius_search(p, radius, max_nn as usize, &mut found);
    if !out_idx.is_null() {
        for (k, &leaf_idx) in found.iter().take(cap as usize).enumerate() {
            // SAFETY: k < cap, so `out_idx.add(k)` is in bounds of the caller's buffer.
            unsafe { *out_idx.add(k) = leaf_idx as u32 };
        }
    }
    found.len() as u32
}

/// # Safety
/// `map` valid; `mean_out`/`icov_out` to 3 / 9 writable `f64`. Writes them and returns true iff
/// `idx` is a valid leaf index.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[allow(
    clippy::as_conversions,
    clippy::allow_attributes,
    reason = "C ABI: idx u32->usize"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_voxel_grid_map_leaf(
    map: *const VoxelGridMap,
    idx: u32,
    mean_out: *mut f64,
    icov_out: *mut f64,
) -> bool {
    if map.is_null() || mean_out.is_null() || icov_out.is_null() {
        return false;
    }
    // SAFETY: valid handle per the contract.
    let map = unsafe { &*map };
    match map.leaf(idx as usize) {
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
/// `map` must be a handle from `..._voxel_grid_map_new` (or null); not used afterwards.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_voxel_grid_map_free(map: *mut VoxelGridMap) {
    if map.is_null() {
        return;
    }
    // SAFETY: `map` came from `Box::into_raw` and is dropped exactly once.
    drop(unsafe { Box::from_raw(map) });
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    clippy::expect_used,
    clippy::needless_range_loop,
    clippy::unreadable_literal,
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    unsafe_code,
    clippy::allow_attributes,
    reason = "test code"
)]
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

    // Robustness against malformed sensor input: non-finite points (NaN/inf, common LiDAR
    // dropouts) must be excluded from the bbox and accumulation, `min_points <= 0` must fall
    // back to the default threshold, and a non-finite query must return None (never index a leaf).
    #[test]
    fn non_finite_points_filtered_and_min_points_defaults() {
        let mut pts: alloc::vec::Vec<[f32; 3]> = (0..6)
            .map(|i| {
                let f = i as f32 * 0.01;
                [0.4 + f, 0.4 + f, 0.4 + f]
            })
            .collect();
        pts.push([f32::NAN, 0.0, 0.0]); // skipped in both the bbox and accumulation passes
        pts.push([f32::INFINITY, 0.0, 0.0]);

        // min_points = 0 -> default (6); the voxel has exactly 6 finite points -> kept.
        let grid = VoxelGrid::build(&pts, [1.0, 1.0, 1.0], 0, 0.01);
        let leaf = grid
            .leaf_at([0.4, 0.4, 0.4])
            .expect("finite cluster kept at default min_points");
        assert_eq!(leaf.n, 6, "non-finite points must not be counted");

        // Brute-force mean over the 6 finite points only.
        let mut m = 0.0_f64;
        for i in 0..6 {
            m += f64::from(0.4 + i as f32 * 0.01);
        }
        m /= 6.0;
        assert!(
            (leaf.mean[0] - m).abs() < 1e-6,
            "mean corrupted by non-finite points"
        );

        // A non-finite query must be rejected, not turned into a garbage voxel index.
        assert!(grid.leaf_at([f32::NAN, 0.4, 0.4]).is_none());
    }

    // Well-conditioned voxel: leaf icov must equal the brute-force sample-covariance inverse.
    // Heavy (50-point eigen + matrix inverse) — skipped under Miri to keep the unsafe-UB run fast.
    #[cfg_attr(miri, ignore)]
    #[test]
    fn icov_matches_brute_force_inverse() {
        let mut pts: alloc::vec::Vec<[f32; 3]> = alloc::vec::Vec::new();
        // deterministic spread around (5,5,5), all inside voxel [0,10) for leaf_size 10.
        for i in 0..50_i32 {
            let f = i as f32;
            pts.push([
                5.0 + 0.2 * (f * 0.7).sin(),
                5.0 + 0.15 * (f * 1.3).cos(),
                5.0 + 0.1 * (f * 0.31).sin(),
            ]);
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
            bf_icov.m11,
            bf_icov.m12,
            bf_icov.m13,
            bf_icov.m21,
            bf_icov.m22,
            bf_icov.m23,
            bf_icov.m31,
            bf_icov.m32,
            bf_icov.m33,
        ];
        for k in 0..9 {
            assert!(
                (leaf.icov[k] - bf[k]).abs() <= 1e-6 * bf[k].abs() + 1e-9,
                "k={k} {} vs {}",
                leaf.icov[k],
                bf[k]
            );
        }
    }

    #[test]
    fn empty_cloud_builds_empty_grid() {
        let grid = VoxelGrid::build(&[], [1.0, 1.0, 1.0], 6, 0.01);
        assert!(grid.leaf_at([0.0, 0.0, 0.0]).is_none());
    }

    // 8 points packed inside the single voxel containing (cx,cy,cz) for leaf_size 2.0.
    fn dense_cluster(cx: f32, cy: f32, cz: f32) -> alloc::vec::Vec<[f32; 3]> {
        (0..8)
            .map(|i| {
                let f = i as f32 * 0.02;
                [cx + f, cy - f, cz + 0.5 * f]
            })
            .collect()
    }

    // ---- VoxelGridMap: public API + state transitions ----

    #[test]
    fn map_add_create_then_radius_search_finds_cluster() {
        let mut map = VoxelGridMap::new([2.0, 2.0, 2.0], 6, 0.01);
        map.add_target(&dense_cluster(1.0, 1.0, 1.0), 0);
        map.add_target(&dense_cluster(21.0, 1.0, 1.0), 1);
        map.create_kdtree();

        let mut hits = alloc::vec::Vec::new();
        map.radius_search([1.0, 1.0, 1.0], 1.5, 0, &mut hits);
        assert_eq!(hits.len(), 1, "exactly one leaf near cluster A");
        let leaf = map.leaf(hits[0]).expect("leaf");
        assert!((leaf.mean[0] - 1.07).abs() < 0.2 && (leaf.mean[1] - 0.93).abs() < 0.2);

        // A query far from every centroid finds nothing.
        let mut none = alloc::vec::Vec::new();
        map.radius_search([100.0, 100.0, 100.0], 1.5, 0, &mut none);
        assert!(none.is_empty());
    }

    #[test]
    fn map_remove_target_drops_its_leaves() {
        let mut map = VoxelGridMap::new([2.0, 2.0, 2.0], 6, 0.01);
        map.add_target(&dense_cluster(1.0, 1.0, 1.0), 0);
        map.add_target(&dense_cluster(21.0, 1.0, 1.0), 1);
        map.remove_target(0);
        map.create_kdtree();

        let mut a = alloc::vec::Vec::new();
        map.radius_search([1.0, 1.0, 1.0], 1.5, 0, &mut a);
        assert!(a.is_empty(), "removed grid's leaf must be gone");
        let mut b = alloc::vec::Vec::new();
        map.radius_search([21.0, 1.0, 1.0], 1.5, 0, &mut b);
        assert_eq!(b.len(), 1, "remaining grid still searchable");
    }

    #[test]
    fn map_search_before_create_kdtree_is_empty() {
        let mut map = VoxelGridMap::new([2.0, 2.0, 2.0], 6, 0.01);
        map.add_target(&dense_cluster(1.0, 1.0, 1.0), 0);
        // No create_kdtree() yet.
        let mut hits = alloc::vec::Vec::new();
        map.radius_search([1.0, 1.0, 1.0], 1.5, 0, &mut hits);
        assert!(hits.is_empty());
        assert!(map.leaf(0).is_none(), "no flat leaves before create_kdtree");
    }

    // Reference-model property: kd-tree radius search over the map == brute force over flat leaves.
    // Heavy (9 clusters of eigen + 40 queries) — skipped under Miri to keep the unsafe-UB run fast.
    #[cfg_attr(miri, ignore)]
    #[test]
    fn map_radius_search_matches_brute_force() {
        let mut map = VoxelGridMap::new([2.0, 2.0, 2.0], 6, 0.01);
        let mut id = 0_u64;
        for i in 0..3 {
            for j in 0..3 {
                let (cx, cy) = ((3 * i + 1) as f32, (3 * j + 1) as f32);
                map.add_target(&dense_cluster(cx, cy, 1.0), id);
                id += 1;
            }
        }
        map.create_kdtree();

        // Deterministic queries spanning the populated region.
        let mut state = 0x9E37_79B9_u64;
        for _ in 0..40 {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let qx = ((state >> 33) % 1000) as f32 / 100.0; // [0,10)
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let qy = ((state >> 33) % 1000) as f32 / 100.0;
            let radius = 2.5_f64;

            let mut got = alloc::vec::Vec::new();
            map.radius_search([qx, qy, 1.0], radius, 0, &mut got);
            got.sort_unstable();

            let r2 = radius * radius;
            let mut expected: alloc::vec::Vec<usize> = (0..map.flat_leaves.len())
                .filter(|&k| {
                    let c = map.flat_leaves[k].centroid;
                    let (dx, dy, dz) = (
                        f64::from(c[0]) - f64::from(qx),
                        f64::from(c[1]) - f64::from(qy),
                        f64::from(c[2]) - 1.0,
                    );
                    (dx * dx) + (dy * dy) + (dz * dz) <= r2
                })
                .collect();
            expected.sort_unstable();
            assert_eq!(got, expected);
        }
    }

    // ---- FFI shims: round-trip equals the pure path; null/cap contracts ----

    #[test]
    fn ffi_voxel_grid_build_leaf_at_matches_pure() {
        let pts = dense_cluster(1.0, 1.0, 1.0);
        let flat: alloc::vec::Vec<f32> = pts.iter().flat_map(|p| p.iter().copied()).collect();
        let leaf_size = [2.0_f64, 2.0, 2.0];

        let pure = VoxelGrid::build(&pts, leaf_size, 6, 0.01);
        let pure_leaf = pure.leaf_at([1.0, 1.0, 1.0]).expect("pure leaf");

        let grid = unsafe {
            autoware_ndt_scan_matcher_rs_voxel_grid_build(
                flat.as_ptr(),
                pts.len(),
                leaf_size.as_ptr(),
                6,
                0.01,
            )
        };
        assert!(!grid.is_null());
        let q = [1.0_f32, 1.0, 1.0];
        let (mut mean, mut icov) = ([0.0_f64; 3], [0.0_f64; 9]);
        let hit = unsafe {
            autoware_ndt_scan_matcher_rs_voxel_grid_leaf_at(
                grid,
                q.as_ptr(),
                mean.as_mut_ptr(),
                icov.as_mut_ptr(),
            )
        };
        assert!(hit);
        assert_eq!(mean, pure_leaf.mean);
        assert_eq!(icov, pure_leaf.icov);
        unsafe { autoware_ndt_scan_matcher_rs_voxel_grid_free(grid) };

        // null contracts.
        assert!(
            unsafe {
                autoware_ndt_scan_matcher_rs_voxel_grid_build(
                    core::ptr::null(),
                    0,
                    leaf_size.as_ptr(),
                    6,
                    0.01,
                )
            }
            .is_null()
        );
        assert!(!unsafe {
            autoware_ndt_scan_matcher_rs_voxel_grid_leaf_at(
                core::ptr::null(),
                q.as_ptr(),
                mean.as_mut_ptr(),
                icov.as_mut_ptr(),
            )
        });
        unsafe { autoware_ndt_scan_matcher_rs_voxel_grid_free(core::ptr::null_mut()) }; // no-op, must not crash
    }

    #[test]
    fn ffi_map_radius_search_and_leaf_match_pure_with_cap() {
        let a = dense_cluster(1.0, 1.0, 1.0);
        let b = dense_cluster(21.0, 1.0, 1.0);
        let leaf_size = [2.0_f64, 2.0, 2.0];

        // Pure reference map.
        let mut pure = VoxelGridMap::new(leaf_size, 6, 0.01);
        pure.add_target(&a, 0);
        pure.add_target(&b, 1);
        pure.create_kdtree();

        // FFI map, same inputs.
        let fa: alloc::vec::Vec<f32> = a.iter().flat_map(|p| p.iter().copied()).collect();
        let fb: alloc::vec::Vec<f32> = b.iter().flat_map(|p| p.iter().copied()).collect();
        let map =
            unsafe { autoware_ndt_scan_matcher_rs_voxel_grid_map_new(leaf_size.as_ptr(), 6, 0.01) };
        assert!(!map.is_null());
        unsafe {
            autoware_ndt_scan_matcher_rs_voxel_grid_map_add_target(map, fa.as_ptr(), a.len(), 0);
            autoware_ndt_scan_matcher_rs_voxel_grid_map_add_target(map, fb.as_ptr(), b.len(), 1);
            autoware_ndt_scan_matcher_rs_voxel_grid_map_create_kdtree(map);
        }

        // Query near A: FFI count == pure count, and the returned leaf mean/icov match pure.
        let q = [1.0_f32, 1.0, 1.0];
        let mut idx = [u32::MAX; 8];
        let n = unsafe {
            autoware_ndt_scan_matcher_rs_voxel_grid_map_radius_search(
                map,
                q.as_ptr(),
                1.5,
                0,
                idx.as_mut_ptr(),
                idx.len() as u32,
            )
        };
        let mut pure_hits = alloc::vec::Vec::new();
        pure.radius_search([1.0, 1.0, 1.0], 1.5, 0, &mut pure_hits);
        assert_eq!(n as usize, pure_hits.len());
        assert_eq!(n, 1);

        let (mut mean, mut icov) = ([0.0_f64; 3], [0.0_f64; 9]);
        assert!(unsafe {
            autoware_ndt_scan_matcher_rs_voxel_grid_map_leaf(
                map,
                idx[0],
                mean.as_mut_ptr(),
                icov.as_mut_ptr(),
            )
        });
        let pure_leaf = pure.leaf(pure_hits[0]).expect("pure leaf");
        assert_eq!(mean, pure_leaf.mean);
        assert_eq!(icov, pure_leaf.icov);

        // cap == 0: nothing written, but the true total is still returned.
        let total = unsafe {
            autoware_ndt_scan_matcher_rs_voxel_grid_map_radius_search(
                map,
                q.as_ptr(),
                1.5,
                0,
                idx.as_mut_ptr(),
                0,
            )
        };
        assert_eq!(total, 1, "returns total found even when cap is 0");

        // null contracts.
        assert_eq!(
            unsafe {
                autoware_ndt_scan_matcher_rs_voxel_grid_map_radius_search(
                    core::ptr::null(),
                    q.as_ptr(),
                    1.5,
                    0,
                    idx.as_mut_ptr(),
                    idx.len() as u32,
                )
            },
            0
        );
        assert!(!unsafe {
            autoware_ndt_scan_matcher_rs_voxel_grid_map_leaf(
                map,
                9999,
                mean.as_mut_ptr(),
                icov.as_mut_ptr(),
            )
        });
        unsafe { autoware_ndt_scan_matcher_rs_voxel_grid_map_free(map) };
        unsafe { autoware_ndt_scan_matcher_rs_voxel_grid_map_free(core::ptr::null_mut()) }; // no-op
    }

    // NDT eigenvalue regularization (compute_icov clamp branch): for a strongly anisotropic
    // (near-planar) voxel the smallest covariance eigenvalue is clamped UP to eig_mult*largest.
    // Oracle is the *invariant* this enforces, not a re-implementation of the clamp: after
    // regularization the largest icov eigenvalue == 1 / (eig_mult * largest_cov_eigenvalue).
    // A bug that skipped clamping would leave a far larger (1/tiny) eigenvalue -> test fails.
    #[cfg_attr(miri, ignore)] // eigendecomposition is heavy under Miri
    #[test]
    fn icov_eigenvalue_regularization_clamps_smallest() {
        // Near-planar cluster inside one voxel: wide spread in x, less in y, zero in z.
        let mut pts: alloc::vec::Vec<[f32; 3]> = alloc::vec::Vec::new();
        for i in 0..60_i32 {
            let f = i as f32;
            pts.push([
                50.0 + 4.0 * (f * 0.7).sin(),
                50.0 + 1.0 * (f * 1.3).cos(),
                50.0,
            ]);
        }
        let eig_mult = 0.01_f64;
        let grid = VoxelGrid::build(&pts, [100.0, 100.0, 100.0], 6, eig_mult);
        let leaf = grid.leaf_at([50.0, 50.0, 50.0]).expect("leaf");

        // Independent brute-force covariance (Identity-init like C++: +I then /(n-1)).
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
        cov += Matrix3::identity();
        cov /= n - 1.0;

        let mut evals: alloc::vec::Vec<f64> =
            cov.symmetric_eigen().eigenvalues.iter().copied().collect();
        evals.sort_by(f64::total_cmp);
        let (smallest, largest) = (evals[0], evals[2]);
        // Precondition: clamping must actually fire (else the test is vacuous).
        assert!(
            smallest < eig_mult * largest,
            "cluster must trigger clamping: {smallest} !< {}",
            eig_mult * largest
        );

        let icov = Matrix3::new(
            leaf.icov[0],
            leaf.icov[1],
            leaf.icov[2],
            leaf.icov[3],
            leaf.icov[4],
            leaf.icov[5],
            leaf.icov[6],
            leaf.icov[7],
            leaf.icov[8],
        );
        let max_icov_eig = icov
            .symmetric_eigen()
            .eigenvalues
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);

        let expected = 1.0 / (eig_mult * largest);
        assert!(
            (max_icov_eig - expected).abs() <= 1e-6 * expected,
            "regularized icov max eigenvalue {max_icov_eig} != 1/(eig_mult*largest) {expected}"
        );
        // And it is far below the un-regularized 1/smallest, proving the clamp happened.
        assert!(
            max_icov_eig < 0.5 / smallest,
            "icov eigenvalue was not clamped down"
        );
    }

    // FFI map: null handles are no-ops (must not deref); remove_target via the shim drops a grid;
    // a small cap truncates the written indices while the return value stays the true total.
    #[test]
    fn ffi_map_null_remove_and_cap_contracts() {
        let ls = [1.0_f64, 1.0, 1.0];

        // null-handle contracts: must not crash, and `new(null)` reports null.
        assert!(
            unsafe { autoware_ndt_scan_matcher_rs_voxel_grid_map_new(core::ptr::null(), 6, 0.01) }
                .is_null()
        );
        unsafe {
            autoware_ndt_scan_matcher_rs_voxel_grid_map_add_target(
                core::ptr::null_mut(),
                core::ptr::null(),
                0,
                0,
            );
            autoware_ndt_scan_matcher_rs_voxel_grid_map_remove_target(core::ptr::null_mut(), 0);
            autoware_ndt_scan_matcher_rs_voxel_grid_map_create_kdtree(core::ptr::null_mut());
        }

        // Four clusters in four adjacent voxels (leaf_size 1.0), all within the query radius.
        let centers = [(0.4_f32, 0.4_f32), (1.4, 0.4), (2.4, 0.4), (3.4, 0.4)];
        let map = unsafe { autoware_ndt_scan_matcher_rs_voxel_grid_map_new(ls.as_ptr(), 6, 0.01) };
        assert!(!map.is_null());
        for (id, &(cx, cy)) in centers.iter().enumerate() {
            let c = dense_cluster(cx, cy, 0.4);
            let flat: alloc::vec::Vec<f32> = c.iter().flat_map(|p| p.iter().copied()).collect();
            unsafe {
                autoware_ndt_scan_matcher_rs_voxel_grid_map_add_target(
                    map,
                    flat.as_ptr(),
                    c.len(),
                    id as u64,
                );
            }
        }
        // Remove the first cluster via the FFI shim (the dispatch path under test).
        unsafe { autoware_ndt_scan_matcher_rs_voxel_grid_map_remove_target(map, 0) };
        unsafe { autoware_ndt_scan_matcher_rs_voxel_grid_map_create_kdtree(map) };

        let q = [1.9_f32, 0.4, 0.4];
        // cap smaller than the number found: only `cap` indices written, rest stay sentinel,
        // but the return is the true total (3 remaining clusters within radius).
        let mut idx = [u32::MAX; 8];
        let total = unsafe {
            autoware_ndt_scan_matcher_rs_voxel_grid_map_radius_search(
                map,
                q.as_ptr(),
                3.0,
                0,
                idx.as_mut_ptr(),
                2,
            )
        };
        assert_eq!(
            total, 3,
            "cluster 0 removed via FFI; 3 remain within radius"
        );
        assert!(
            idx[0] != u32::MAX && idx[1] != u32::MAX,
            "cap entries written"
        );
        assert_eq!(idx[2], u32::MAX, "entries beyond cap left untouched");

        unsafe { autoware_ndt_scan_matcher_rs_voxel_grid_map_free(map) };
    }
}
