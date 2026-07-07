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

//! Persistent NDT engine handle. Wraps the target map + params over a stable C ABI (an opaque
//! `AwNdtEngine*`), so C++ callers can drive incremental map updates, alignment, and scoring.
//!
//! Concurrency (engine concurrency refactor): the engine exposes **`&self`-only** methods, and its
//! mutable state lives behind interior mutability in one of three configs:
//!
//! - **std** (default): target map + params in an `ArcSwap<EngineState>` (the read/align path loads
//!   an immutable snapshot lock-free; map-update publishes a fresh state with an atomic store), the
//!   optional regularization in a tiny `ArcSwap<Option<Regularization>>`, and the per-align scratch
//!   (workspace + last result) in a **thread-local** (at most one align per thread, no shared
//!   mutable scratch). `Sync`: a shared `&NdtEngine` is sound across concurrent ROS callbacks
//!   **without** an external mutex, and every FFI forms only `&*engine` (never `&mut *engine`).
//! - **`no_std` single-core** (no features): `RefCell` cells instead of `ArcSwap`, scratch in the
//!   engine. Intentionally `!Sync` — the compiler rejects sharing across threads.
//! - **`no_std` `mt`** (multi-core kernel): the cells are `awkernel_sync` mutexes whose guards
//!   disable interrupts, so every critical section is a few instructions — an `Arc` refcount bump
//!   (load) or a pointer swap (store); rcu builds the next state OUTSIDE the lock and publishes
//!   with an optimistic `Arc::ptr_eq` retry; old snapshots drop outside the lock. There is **no
//!   engine-owned scratch**: callers own a [`MatchScratch`] per task/thread and use the `_with`
//!   methods (the implicit-scratch stateful API — `align`/`result`/`score_arrays`/… — is compiled
//!   out, so a cross-call scratch dependency cannot even compile). `Sync`, like std.
//!
//! Control-plane: the WCET-bounded path is the inner [`crate::ndt::align`], not this wrapper.

use alloc::sync::Arc;

use nalgebra::{Matrix4, Matrix6};

use crate::ffi_ptr::{self, ffi_mut, ffi_mut_slice, ffi_ref, ffi_slice};

#[cfg(any(feature = "std", not(feature = "mt")))]
use crate::ndt::AwNdtAlignOutput;
use crate::ndt::{
    AlignResult, AlignWorkspace, NdtParams, Regularization, align, nearest_voxel_score_each_point,
    nearest_voxel_transformation_likelihood, transformation_probability,
};
use crate::transform::gauss_constants;
use crate::voxel_grid::VoxelGridMap;

// ---- interior-mutability cell (std: lock-free arc-swap; no_std single-core: RefCell<Arc<…>>;
// no_std `mt`: awkernel_sync mutex around the Arc) ----
// `Swap<T>` aliases the backing cell; the `swap_*` free fns give a uniform load/store/rcu API. `load`
// returns an owned `Arc<T>` snapshot (lock-free under std), so callers hold a stable view while a
// concurrent `store`/`rcu` publishes a new version (the old `Arc` lives until its last reader drops).
// Under `mt` the cell is an interrupt-disabling kernel mutex, so every critical section is a few
// instructions (an `Arc` refcount bump or a pointer swap) — never an align, never a deep clone, and
// never the drop of an old snapshot (a last-reference drop deallocates a whole map; all `mt` paths
// drop old `Arc`s after the guard is released, with interrupts re-enabled).

#[cfg(feature = "std")]
type Swap<T> = arc_swap::ArcSwap<T>;
#[cfg(all(not(feature = "std"), not(feature = "mt")))]
type Swap<T> = core::cell::RefCell<Arc<T>>;
#[cfg(all(feature = "mt", not(feature = "std")))]
type Swap<T> = awkernel_sync::mutex::Mutex<Arc<T>>;

#[cfg(feature = "std")]
fn swap_new<T>(v: T) -> Swap<T> {
    arc_swap::ArcSwap::from_pointee(v)
}
#[cfg(all(not(feature = "std"), not(feature = "mt")))]
fn swap_new<T>(v: T) -> Swap<T> {
    core::cell::RefCell::new(Arc::new(v))
}
#[cfg(all(feature = "mt", not(feature = "std")))]
fn swap_new<T: Send + Sync>(v: T) -> Swap<T> {
    awkernel_sync::mutex::Mutex::new(Arc::new(v))
}

#[cfg(feature = "std")]
fn swap_load<T>(c: &Swap<T>) -> Arc<T> {
    c.load_full()
}
#[cfg(all(not(feature = "std"), not(feature = "mt")))]
fn swap_load<T>(c: &Swap<T>) -> Arc<T> {
    c.borrow().clone()
}
#[cfg(all(feature = "mt", not(feature = "std")))]
fn swap_load<T: Send + Sync>(c: &Swap<T>) -> Arc<T> {
    let mut node = awkernel_sync::mcs::MCSNode::new();
    // Critical section: one Arc refcount increment.
    Arc::clone(&c.lock(&mut node))
}

#[cfg(feature = "std")]
fn swap_store<T>(c: &Swap<T>, v: T) {
    c.store(Arc::new(v));
}
#[cfg(all(not(feature = "std"), not(feature = "mt")))]
fn swap_store<T>(c: &Swap<T>, v: T) {
    *c.borrow_mut() = Arc::new(v);
}
#[cfg(all(feature = "mt", not(feature = "std")))]
fn swap_store<T: Send + Sync>(c: &Swap<T>, v: T) {
    swap_store_arc(c, Arc::new(v));
}

/// Read-copy-update: build the next value from the current one and publish it atomically. The
/// closure may run more than once under contention (std/arc-swap CAS retry and the `mt` optimistic
/// retry), so it must be pure.
#[cfg(feature = "std")]
fn swap_rcu<T>(c: &Swap<T>, f: impl Fn(&T) -> T) {
    c.rcu(|cur| Arc::new(f(cur)));
}
#[cfg(all(not(feature = "std"), not(feature = "mt")))]
fn swap_rcu<T>(c: &Swap<T>, f: impl Fn(&T) -> T) {
    let next = {
        let cur = c.borrow();
        Arc::new(f(&cur))
    };
    *c.borrow_mut() = next;
}
/// `mt`: optimistic retry — `f` runs OUTSIDE the lock (it may deep-clone a whole map, which must
/// never happen with interrupts disabled); the short re-lock publishes only if the snapshot is
/// still current (`Arc::ptr_eq`), else it retries against the fresh one. Writers here are rare
/// (params setup / map-update commit), so the loop converges immediately in practice.
#[cfg(all(feature = "mt", not(feature = "std")))]
fn swap_rcu<T: Send + Sync>(c: &Swap<T>, f: impl Fn(&T) -> T) {
    let mut cur = swap_load(c);
    loop {
        let next = Arc::new(f(&cur));
        let published = {
            // The MCS node lives one iteration — the guard borrows it.
            let mut node = awkernel_sync::mcs::MCSNode::new();
            let mut guard = c.lock(&mut node);
            if Arc::ptr_eq(&guard, &cur) {
                // Critical section: one pointer swap; the previous Arc is handed out first.
                Some(core::mem::replace(&mut *guard, next))
            } else {
                cur = Arc::clone(&guard);
                None
            }
        };
        if let Some(old) = published {
            drop(old); // outside the lock — may deallocate the previous state
            return;
        }
        // A stale `next` also drops here, outside the lock.
    }
}

/// Publish an already-built `Arc<T>` snapshot (no deep copy) — the map-update commit shares the
/// staging engine's final state `Arc` into the live engine in one atomic store.
#[cfg(feature = "std")]
fn swap_store_arc<T>(c: &Swap<T>, v: Arc<T>) {
    c.store(v);
}
#[cfg(all(not(feature = "std"), not(feature = "mt")))]
fn swap_store_arc<T>(c: &Swap<T>, v: Arc<T>) {
    *c.borrow_mut() = v;
}
#[cfg(all(feature = "mt", not(feature = "std")))]
fn swap_store_arc<T: Send + Sync>(c: &Swap<T>, v: Arc<T>) {
    let old = {
        let mut node = awkernel_sync::mcs::MCSNode::new();
        let mut guard = c.lock(&mut node);
        // Critical section: one pointer swap; the old Arc drops after the guard is released.
        core::mem::replace(&mut *guard, v)
    };
    drop(old);
}

/// The `covariance` hyper-params that drive [`NdtEngine::estimate_covariance`]: the estimation mode +
/// scaling + the configured 6x6 + the candidate search offsets. Holds owned `Vec` offsets (so it is
/// `Clone`, not `Copy`); set off the hot path via [`NdtEngine::set_covariance_config`].
#[derive(Clone, Debug)]
struct CovarianceConfig {
    estimation_type: i32,
    scale_factor: f64,
    temperature: f64,
    output_pose_covariance: [f64; 36],
    offset_x: alloc::vec::Vec<f64>,
    offset_y: alloc::vec::Vec<f64>,
}

impl CovarianceConfig {
    /// Default: `FIXED_VALUE` (the configured 6x6 only), unit scale, no candidates — the node/example
    /// always sets the real config before estimating.
    fn new() -> Self {
        Self {
            estimation_type: 0,
            scale_factor: 1.0,
            temperature: 1.0,
            output_pose_covariance: [0.0; 36],
            offset_x: alloc::vec::Vec::new(),
            offset_y: alloc::vec::Vec::new(),
        }
    }
}

/// The atomically-swappable engine state: the target map, the active params, and the cell-id → tile
/// mapping. Immutable once published — map-update builds a fresh `EngineState` and stores it.
#[derive(Clone)]
struct EngineState {
    map: VoxelGridMap,
    params: NdtParams,
    /// The `score_estimation` scalars that gate the convergence verdict (read on `align_outcome`).
    /// Part of the swapped state so `set_convergence_params` is lock-free like `set_params`.
    conv: ConvergenceParams,
    /// The `covariance` hyper-params read on `estimate_covariance` (swapped like `conv`).
    cov_config: CovarianceConfig,
    /// Cell-id bytes → tile `u64` (the engine owns the legacy string-id mapping; keys are the
    /// raw `std::string` cell-id bytes — not validated UTF-8).
    id_map: alloc::collections::BTreeMap<alloc::vec::Vec<u8>, u64>,
    next_id: u64,
}

/// Per-align scratch: the reused workspace + the last align result. Never shared mutable state
/// across threads — std keeps one per executor thread (the `SCRATCH` thread-local), the plain
/// `no_std` single-core build keeps one in the engine, and the `mt` build has each task/thread own
/// one and pass it to the engine's `_with` methods explicitly (reused across frames so the align
/// stays allocation-free after warmup).
///
/// # Examples
///
/// Own one scratch per task/thread and reuse it across frames (the only align path under `mt`):
///
/// ```
/// use autoware_ndt_scan_matcher_rs::engine::{MatchScratch, NdtEngine};
/// use autoware_ndt_scan_matcher_rs::nalgebra::Matrix4;
///
/// let engine = NdtEngine::new(2.0, 6, 0.01);
/// let target: Vec<[f32; 3]> = (0u8..64).map(|i| [f32::from(i) * 0.05, 0.0, 0.0]).collect();
/// engine.add_target(&target, 0);
/// engine.create_kdtree();
///
/// let mut scratch = MatchScratch::new();
/// // Frame 1, then frame 2 — the same scratch is reused (allocation-free after warmup).
/// engine.align_with(&Matrix4::identity(), &target, &mut scratch);
/// engine.align_with(&scratch.result().pose, &target, &mut scratch);
/// assert!(scratch.result().iteration_num >= 0);
/// ```
pub struct MatchScratch {
    workspace: AlignWorkspace,
    last: AlignResult,
}

impl MatchScratch {
    /// Fresh scratch (buffers warm up on the first align).
    #[must_use]
    pub fn new() -> Self {
        Self {
            workspace: AlignWorkspace::new(),
            last: AlignResult::default(),
        }
    }

    /// Owned copy of the last align result run through this scratch (the C++ `getResult`).
    #[must_use]
    pub fn result(&self) -> AlignResult {
        self.last.clone()
    }

    /// The per-iteration score traces of the last align run through this scratch
    /// (`transform_probability_array` / `nearest_voxel_likelihood_array` of the C++ `NdtResult`).
    #[must_use]
    pub fn score_arrays(&self) -> (alloc::vec::Vec<f32>, alloc::vec::Vec<f32>) {
        (
            self.last.transform_probability_array.clone(),
            self.last.nearest_voxel_likelihood_array.clone(),
        )
    }
}

impl Default for MatchScratch {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "std")]
std::thread_local! {
    /// One reused scratch per executor thread. At most one align runs per thread at a time, so this
    /// is exclusive without a lock; reused buffers keep the align frame allocation-free after warmup.
    static SCRATCH: core::cell::RefCell<MatchScratch> = core::cell::RefCell::new(MatchScratch::new());
}

/// Persistent NDT engine: a `Swap` cell of the target map + params (+ id mapping) and the
/// optional regularization. `&self`-only + `Sync` (std and `mt`; the plain `no_std` single-core
/// build is `!Sync` and additionally keeps the align scratch here) — see the module docs.
///
/// # Examples
///
/// Configure, load a target map, and align via a caller-owned [`MatchScratch`] (the universal path,
/// available in every build config):
///
/// ```
/// use autoware_ndt_scan_matcher_rs::engine::{MatchScratch, NdtEngine};
/// use autoware_ndt_scan_matcher_rs::nalgebra::Matrix4;
///
/// let engine = NdtEngine::new(2.0, 6, 0.01);
/// // trans_epsilon, step_size, resolution, max_iterations, outlier_ratio, num_threads
/// engine.set_params(0.01, 0.1, 2.0, 30, 0.55, 1);
///
/// let target: Vec<[f32; 3]> = (0u8..64).map(|i| [f32::from(i) * 0.05, 0.0, 0.0]).collect();
/// engine.add_target(&target, 0);
/// engine.create_kdtree();
///
/// let mut scratch = MatchScratch::new();
/// engine.align_with(&Matrix4::identity(), &target, &mut scratch);
/// assert!(scratch.result().iteration_num >= 0);
/// ```
pub struct NdtEngine {
    state: Swap<EngineState>,
    /// Optional longitudinal regularization, swapped lock-free (set per sensor frame before align,
    /// read at align time on the same thread). A dedicated tiny cell so setting it never clones the
    /// map (it is not part of `EngineState`).
    reg: Swap<Option<Regularization>>,
    min_points: i32,
    eig_mult: f64,
    /// `no_std` (single-core) keeps the scratch here; std uses the thread-local `SCRATCH`.
    #[cfg(all(not(feature = "std"), not(feature = "mt")))]
    scratch: core::cell::RefCell<MatchScratch>,
}

/// Compile-time proof that the engine is shareable across threads in the two concurrent configs
/// (std: arc-swap + thread-local scratch; `mt`: `awkernel_sync` cells + caller-owned scratch). The
/// plain `no_std` single-core engine stays intentionally `!Sync` (`RefCell`), so it is excluded.
#[cfg(any(feature = "std", feature = "mt"))]
const fn assert_send_sync<T: Send + Sync>() {}
#[cfg(any(feature = "std", feature = "mt"))]
const _: () = assert_send_sync::<NdtEngine>();

impl Clone for NdtEngine {
    fn clone(&self) -> Self {
        // Deep-copy the published state + regularization (the map-update double-buffer clones the
        // whole engine). The scratch is not state — a clone starts fresh (re-warms on first align).
        let st = swap_load(&self.state);
        Self {
            state: swap_new((*st).clone()),
            reg: swap_new(*swap_load(&self.reg)),
            min_points: self.min_points,
            eig_mult: self.eig_mult,
            #[cfg(all(not(feature = "std"), not(feature = "mt")))]
            scratch: core::cell::RefCell::new(MatchScratch::new()),
        }
    }
}

impl NdtEngine {
    /// Clone the engine's configuration (params, convergence + covariance config, regularization) but
    /// with an **empty** map (no tiles, fresh id mapping). Used by the map-update rebuild path to start
    /// a staging engine from scratch (mirrors the C++ `need_rebuild`: fresh `NdtType` + `setParams`),
    /// as opposed to [`Clone`] which deep-copies the current map.
    #[must_use]
    pub fn clone_empty(&self) -> Self {
        let st = self.load_state();
        Self {
            state: swap_new(EngineState {
                map: VoxelGridMap::new([st.params.resolution; 3], self.min_points, self.eig_mult),
                params: st.params,
                conv: st.conv,
                cov_config: st.cov_config.clone(),
                id_map: alloc::collections::BTreeMap::new(),
                next_id: 0,
            }),
            reg: swap_new(*swap_load(&self.reg)),
            min_points: self.min_points,
            eig_mult: self.eig_mult,
            #[cfg(all(not(feature = "std"), not(feature = "mt")))]
            scratch: core::cell::RefCell::new(MatchScratch::new()),
        }
    }

    /// New engine with an empty map.
    ///
    /// # Arguments
    /// * `resolution` — voxel/leaf size in metres; also the neighbor search radius.
    /// * `min_points` — minimum points per voxel for it to contribute a Gaussian (C++
    ///   `MultiVoxelGridCovariance` default: 6).
    /// * `eig_mult` — eigenvalue-inflation multiplier conditioning each voxel covariance (C++
    ///   default: 0.01).
    #[must_use]
    pub fn new(resolution: f64, min_points: i32, eig_mult: f64) -> Self {
        Self {
            state: swap_new(EngineState {
                map: VoxelGridMap::new([resolution; 3], min_points, eig_mult),
                params: NdtParams {
                    resolution,
                    ..NdtParams::default()
                },
                conv: ConvergenceParams::default(),
                cov_config: CovarianceConfig::new(),
                id_map: alloc::collections::BTreeMap::new(),
                next_id: 0,
            }),
            reg: swap_new(None),
            min_points,
            eig_mult,
            #[cfg(all(not(feature = "std"), not(feature = "mt")))]
            scratch: core::cell::RefCell::new(MatchScratch::new()),
        }
    }

    /// The current published state snapshot (lock-free under std).
    fn load_state(&self) -> Arc<EngineState> {
        swap_load(&self.state)
    }

    /// Run `f` with the per-align scratch (thread-local under std; engine-owned under `no_std`).
    #[cfg(feature = "std")]
    #[expect(
        clippy::unused_self,
        reason = "the std scratch is a thread-local; &self is unused here but keeps a uniform API with the no_std variant (which borrows self.scratch)"
    )]
    pub(crate) fn with_scratch<R>(&self, f: impl FnOnce(&mut MatchScratch) -> R) -> R {
        SCRATCH.with(|s| f(&mut s.borrow_mut()))
    }
    #[cfg(all(not(feature = "std"), not(feature = "mt")))]
    pub(crate) fn with_scratch<R>(&self, f: impl FnOnce(&mut MatchScratch) -> R) -> R {
        f(&mut self.scratch.borrow_mut())
    }

    /// Update the alignment params (the C++ `setParams`). The regularization is preserved — it is
    /// set separately via [`Self::set_regularization`], mirroring `setRegularizationPose`. Mirroring
    /// the C++ (which applies `resolution` as the leaf size at `addTarget`), the empty map is rebuilt
    /// at the new resolution — the node always calls `set_params` before `add_target`.
    ///
    /// # Arguments
    /// * `trans_epsilon` — translation convergence tolerance (`setTransformationEpsilon`).
    /// * `step_size` — More-Thuente line-search step size (`setStepSize`).
    /// * `resolution` — voxel/leaf size in metres (`setResolution`); rebuilds the map if still empty.
    /// * `max_iterations` — optimizer iteration cap (`setMaximumIterations`).
    /// * `outlier_ratio` — Gaussian mixture outlier ratio (default 0.55).
    /// * `num_threads` — derivative-reduction worker count; `> 1` uses the rayon backend when the
    ///   `parallel` feature is on (bit-identical to serial).
    pub fn set_params(
        &self,
        trans_epsilon: f64,
        step_size: f64,
        resolution: f64,
        max_iterations: i32,
        outlier_ratio: f64,
        num_threads: usize,
    ) {
        swap_rcu(&self.state, |s| {
            let mut n = s.clone();
            n.params.trans_epsilon = trans_epsilon;
            n.params.step_size = step_size;
            n.params.resolution = resolution;
            n.params.max_iterations = max_iterations;
            n.params.outlier_ratio = outlier_ratio;
            n.params.num_threads = num_threads;
            if n.map.is_empty() {
                n.map = VoxelGridMap::new([resolution; 3], self.min_points, self.eig_mult);
            }
            n
        });
    }

    /// Set the `score_estimation` scalars that gate the convergence verdict (the C++
    /// `converged_param_*`). Lock-free (swapped into the published state like [`Self::set_params`]).
    pub fn set_convergence_params(
        &self,
        converged_param_type: i32,
        converged_param_transform_probability: f64,
        converged_param_nearest_voxel_transformation_likelihood: f64,
    ) {
        swap_rcu(&self.state, |s| {
            let mut n = s.clone();
            n.conv = ConvergenceParams {
                converged_param_type,
                converged_param_transform_probability,
                converged_param_nearest_voxel_transformation_likelihood,
            };
            n
        });
    }

    /// Align from `guess` and derive the full [`AlignOutcome`] (oscillation + convergence verdict),
    /// reading the engine's own [`ConvergenceParams`]. The self-contained counterpart of [`run_align`]
    /// (which takes the params explicitly for the C++ FFI). Uses the implicit per-thread/engine
    /// scratch; absent under `mt`, where callers use [`Self::align_outcome_with`].
    #[cfg(any(feature = "std", not(feature = "mt")))]
    #[must_use]
    pub fn align_outcome(&self, guess: &Matrix4<f32>, source: &[[f32; 3]]) -> AlignOutcome {
        self.with_scratch(|scr| self.align_outcome_with(guess, source, scr))
    }

    /// [`Self::align_outcome`] with an explicit caller-owned scratch — what the portable
    /// `ScanMatcher` calls; the only align entry point under `mt`.
    #[must_use]
    pub fn align_outcome_with(
        &self,
        guess: &Matrix4<f32>,
        source: &[[f32; 3]],
        scratch: &mut MatchScratch,
    ) -> AlignOutcome {
        let conv = self.load_state().conv;
        run_align_with(self, guess, source, &conv, scratch)
    }

    /// Set the `covariance` hyper-params read on [`Self::estimate_covariance`] (estimation mode +
    /// scaling + the configured 6x6 + the candidate search offsets). Lock-free (swapped into the
    /// published state like [`Self::set_params`]).
    pub fn set_covariance_config(
        &self,
        estimation_type: i32,
        scale_factor: f64,
        temperature: f64,
        output_pose_covariance: [f64; 36],
        offset_x: &[f64],
        offset_y: &[f64],
    ) {
        swap_rcu(&self.state, |s| {
            let mut n = s.clone();
            n.cov_config = CovarianceConfig {
                estimation_type,
                scale_factor,
                temperature,
                output_pose_covariance,
                offset_x: offset_x.to_vec(),
                offset_y: offset_y.to_vec(),
            };
            n
        });
    }

    /// Estimate the 6x6 pose covariance from an align result, reading the engine's own
    /// `CovarianceConfig`. The self-contained counterpart of [`estimate_pose_covariance`] (which takes
    /// the params explicitly for the C++ FFI) — what the portable `ScanMatcher` calls after a match.
    /// `map_to_base_link_rot3x3` is derived from `result_pose`'s rotation block (the C++ builds the same
    /// from the result quaternion). `scratch` backs the `MULTI_NDT*` candidate re-aligns/scores.
    ///
    /// # Arguments
    /// * `result_pose` — the converged align pose; its rotation block seeds the covariance rotation.
    /// * `hessian` — the align's 6×6 Hessian (from [`MatchScratch::result`]'s `AlignResult`).
    /// * `initial_pose` — the initial guess the align started from (the candidate-search origin).
    /// * `source` — the same sensor cloud that was aligned (`base_link` frame).
    /// * `main_nvtl` — the main result's nearest-voxel transformation likelihood (softmax reference).
    /// * `scratch` — caller-owned workspace reused for the covariance candidate re-aligns/scores.
    #[must_use]
    pub fn estimate_covariance(
        &self,
        result_pose: &Matrix4<f32>,
        hessian: &Matrix6<f64>,
        initial_pose: &Matrix4<f32>,
        source: &[[f32; 3]],
        main_nvtl: f32,
        scratch: &mut MatchScratch,
    ) -> CovEstimationResult {
        let cfg = self.load_state().cov_config.clone();
        // Hessian → row-major [f64; 36]. nalgebra stores column-major, so the transpose's slice is the
        // row-major layout (lengths are both 36 — `copy_from_slice` cannot mismatch here).
        let mut hess = [0.0_f64; 36];
        hess.copy_from_slice(hessian.transpose().as_slice());
        // map→base_link 3x3 rotation (row-major) from the result pose's rotation block (explicit literal
        // — fixed nalgebra indices, no computed index/arithmetic).
        let rot3x3 = [
            f64::from(result_pose[(0, 0)]),
            f64::from(result_pose[(0, 1)]),
            f64::from(result_pose[(0, 2)]),
            f64::from(result_pose[(1, 0)]),
            f64::from(result_pose[(1, 1)]),
            f64::from(result_pose[(1, 2)]),
            f64::from(result_pose[(2, 0)]),
            f64::from(result_pose[(2, 1)]),
            f64::from(result_pose[(2, 2)]),
        ];
        let params = CovEstimationParams {
            estimation_type: cfg.estimation_type,
            scale_factor: cfg.scale_factor,
            temperature: cfg.temperature,
            main_nvtl,
            output_pose_covariance: cfg.output_pose_covariance,
            map_to_base_link_rot3x3: rot3x3,
        };
        estimate_pose_covariance(
            self,
            result_pose,
            &hess,
            initial_pose,
            source,
            &cfg.offset_x,
            &cfg.offset_y,
            &params,
            scratch,
        )
    }

    /// Set (or, when `scale == 0`, clear) the longitudinal regularization toward `(x, y)`.
    #[allow(
        clippy::float_cmp,
        clippy::allow_attributes,
        reason = "exact == 0 is the C++ disable sentinel (setRegularizationPose vs unset)"
    )]
    pub fn set_regularization(&self, x: f32, y: f32, scale: f32) {
        let reg = if scale == 0.0 {
            None
        } else {
            Some(Regularization {
                pose_xy: [x, y],
                scale_factor: scale,
            })
        };
        swap_store(&self.reg, reg);
    }

    /// Add a target map tile keyed by `id` (C++ callers may also register a string `cell_id`,
    /// which this engine maps to a `u64`). Needs a following [`Self::create_kdtree`].
    ///
    /// # Arguments
    /// * `points` — the tile's map points in the map frame (`[x, y, z]`, metres).
    /// * `id` — tile key; re-adding the same `id` replaces that tile.
    pub fn add_target(&self, points: &[[f32; 3]], id: u64) {
        swap_rcu(&self.state, |s| {
            let mut n = s.clone();
            n.map.add_target(points, id);
            n
        });
    }

    /// Remove the map tile registered under `id` (the C++ `removeTarget`).
    pub fn remove_target(&self, id: u64) {
        swap_rcu(&self.state, |s| {
            let mut n = s.clone();
            n.map.remove_target(id);
            n
        });
    }

    /// Add a target tile keyed by the cell-id bytes (the C++ `addTarget(cloud, cell_id)`); the engine
    /// owns the cell-id → `u64` mapping, assigning a fresh id on first use. Needs a following
    /// [`Self::create_kdtree`].
    pub fn add_target_bytes(&self, points: &[[f32; 3]], id: &[u8]) {
        swap_rcu(&self.state, |s| {
            let mut n = s.clone();
            let u = if let Some(&u) = n.id_map.get(id) {
                u
            } else {
                let u = n.next_id;
                n.next_id = n.next_id.saturating_add(1);
                n.id_map.insert(id.to_vec(), u);
                u
            };
            n.map.add_target(points, u);
            n
        });
    }

    /// Remove the tile registered under the cell-id bytes (the C++ `removeTarget(cell_id)`); no-op if
    /// the id is unknown.
    pub fn remove_target_bytes(&self, id: &[u8]) {
        swap_rcu(&self.state, |s| {
            let mut n = s.clone();
            if let Some(u) = n.id_map.remove(id) {
                n.map.remove_target(u);
            }
            n
        });
    }

    /// Rebuild the kd-tree over the current tiles' centroids (the C++ `createVoxelKdtree`).
    pub fn create_kdtree(&self) {
        swap_rcu(&self.state, |s| {
            let mut n = s.clone();
            n.map.create_kdtree();
            n
        });
    }

    /// Atomically publish `src`'s map/params/id state into this engine in **one** store — the
    /// map-update commit. The node's timer builds a fresh, fully-finalized map (tiles added/removed +
    /// kd-tree built) on a private staging engine, then commits it here so concurrent aligns switch to
    /// the complete new map in a single atomic step (never a partially-built / kd-tree-less map). The
    /// state `Arc` is shared, not deep-copied; the old snapshot lives until its last reader drops it.
    pub fn commit_from(&self, src: &NdtEngine) {
        swap_store_arc(&self.state, swap_load(&src.state));
    }

    /// Whether any target tile is loaded (the C++ `hasTarget`).
    #[must_use]
    pub fn has_target(&self) -> bool {
        !self.load_state().map.is_empty()
    }

    /// Align `source` from `guess` into the caller-owned `scratch` (result lands in
    /// [`MatchScratch::result`]) — the core align entry point, and the only one under `mt`.
    ///
    /// # Arguments
    /// * `guess` — initial pose estimate as a 4×4 homogeneous transform (the C++ `Matrix4f` guess).
    /// * `source` — the sensor cloud to align, in the `base_link` frame (`[x, y, z]`, metres).
    /// * `scratch` — caller-owned per-align workspace + result slot; reuse it across frames.
    pub fn align_with(
        &self,
        guess: &Matrix4<f32>,
        source: &[[f32; 3]],
        scratch: &mut MatchScratch,
    ) {
        let st = self.load_state();
        // Fold the per-call regularization into a local params copy (the free `align` reads
        // `params.regularization`); the engine's stored params keep `regularization: None`.
        let params = NdtParams {
            regularization: *swap_load(&self.reg),
            ..st.params
        };
        align(
            &st.map,
            source,
            guess,
            &params,
            &mut scratch.workspace,
            &mut scratch.last,
        );
    }

    /// Align `source` from `guess`, storing the result in the implicit per-thread/engine scratch
    /// (the C++ `align(out, guess, source)`; retrieve via [`Self::result`]). Absent under `mt` —
    /// use [`Self::align_with`].
    ///
    /// # Arguments
    /// * `guess` — initial pose estimate as a 4×4 homogeneous transform.
    /// * `source` — the sensor cloud to align, in the `base_link` frame (`[x, y, z]`, metres).
    #[cfg(any(feature = "std", not(feature = "mt")))]
    pub fn align(&self, guess: &Matrix4<f32>, source: &[[f32; 3]]) {
        self.with_scratch(|scr| self.align_with(guess, source, scr));
    }

    /// The most recent align result (the C++ `getResult`); an owned copy of the implicit scratch.
    /// Absent under `mt` — read [`MatchScratch::result`] from the scratch you aligned with.
    #[cfg(any(feature = "std", not(feature = "mt")))]
    #[must_use]
    pub fn result(&self) -> AlignResult {
        self.with_scratch(|scr| scr.result())
    }

    /// Score `cloud` (already in the target frame) without aligning, using the caller-owned
    /// `scratch` workspace — the C++ `calculateTransformationProbability`.
    pub fn calc_transformation_probability_with(
        &self,
        cloud: &[[f32; 3]],
        scratch: &mut MatchScratch,
    ) -> f64 {
        let st = self.load_state();
        let gauss = gauss_constants(st.params.outlier_ratio, st.params.resolution);
        transformation_probability(
            &st.map,
            cloud,
            st.params.resolution,
            &gauss,
            &mut scratch.workspace,
        )
    }

    /// [`Self::calc_transformation_probability_with`] on the implicit scratch (absent under `mt`).
    #[cfg(any(feature = "std", not(feature = "mt")))]
    pub fn calc_transformation_probability(&self, cloud: &[[f32; 3]]) -> f64 {
        self.with_scratch(|scr| self.calc_transformation_probability_with(cloud, scr))
    }

    /// Nearest-voxel likelihood of `cloud` without aligning, using the caller-owned `scratch`
    /// workspace — the C++ `calculateNearestVoxelTransformationLikelihood`.
    pub fn calc_nearest_voxel_likelihood_with(
        &self,
        cloud: &[[f32; 3]],
        scratch: &mut MatchScratch,
    ) -> f64 {
        let st = self.load_state();
        let gauss = gauss_constants(st.params.outlier_ratio, st.params.resolution);
        nearest_voxel_transformation_likelihood(
            &st.map,
            cloud,
            st.params.resolution,
            &gauss,
            &mut scratch.workspace,
        )
    }

    /// [`Self::calc_nearest_voxel_likelihood_with`] on the implicit scratch (absent under `mt`).
    #[cfg(any(feature = "std", not(feature = "mt")))]
    pub fn calc_nearest_voxel_likelihood(&self, cloud: &[[f32; 3]]) -> f64 {
        self.with_scratch(|scr| self.calc_nearest_voxel_likelihood_with(cloud, scr))
    }

    /// Per-point nearest-voxel score (the C++ `calculateNearestVoxelScoreEachPoint`) using the
    /// caller-owned `scratch` workspace; `out[i] > 0` iff point `i` found a neighbor. `out` is
    /// filled to `cloud.len()`.
    pub fn nearest_voxel_score_each_point_with(
        &self,
        cloud: &[[f32; 3]],
        out: &mut alloc::vec::Vec<f32>,
        scratch: &mut MatchScratch,
    ) {
        let st = self.load_state();
        let gauss = gauss_constants(st.params.outlier_ratio, st.params.resolution);
        nearest_voxel_score_each_point(
            &st.map,
            cloud,
            st.params.resolution,
            &gauss,
            &mut scratch.workspace,
            out,
        );
    }

    /// [`Self::nearest_voxel_score_each_point_with`] on the implicit scratch (absent under `mt`).
    #[cfg(any(feature = "std", not(feature = "mt")))]
    pub fn nearest_voxel_score_each_point(
        &self,
        cloud: &[[f32; 3]],
        out: &mut alloc::vec::Vec<f32>,
    ) {
        self.with_scratch(|scr| self.nearest_voxel_score_each_point_with(cloud, out, scr));
    }

    /// The configured iteration cap (the C++ `getMaximumIterations`).
    #[must_use]
    pub fn max_iterations(&self) -> i32 {
        self.load_state().params.max_iterations
    }

    /// The per-iteration score traces from the last align (`transform_probability_array` /
    /// `nearest_voxel_likelihood_array` of the C++ `NdtResult`); owned copies of the implicit
    /// scratch. Absent under `mt` — read [`MatchScratch::score_arrays`] instead.
    #[cfg(any(feature = "std", not(feature = "mt")))]
    #[must_use]
    pub fn score_arrays(&self) -> (alloc::vec::Vec<f32>, alloc::vec::Vec<f32>) {
        self.with_scratch(|scr| scr.score_arrays())
    }
}

// ---- C ABI shims (opaque `*const NdtEngine` handle; pointers validated per rust-c-ffi-safety) ----
// The engine is `Sync` (std/`mt`; the plain no_std single-core engine is `!Sync`) and exposes
// `&self`-only methods, so every shim forms only `&*engine` (never `&mut *engine`): concurrent
// calls on a shared `const AwNdtEngine*` are sound without an external lock. The lifecycle shims
// (`new`/`free`/`clone`) own/reclaim the `Box`. The scratch-dependent shims (`align`/`get_result`/
// `calc_*`/`get_score_arrays`) need the implicit scratch, so they are compiled out under `mt` —
// the `mt` consumer is Rust (kernel) code using the `_with` methods + `MatchScratch` directly.

#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::allow_attributes,
    reason = "r/c in 0..4 index a fixed 16-element slice and a fixed-size Matrix4"
)]
#[cfg(any(feature = "std", not(feature = "mt")))]
fn matrix4_from_row_major(buf: &[f32]) -> Matrix4<f32> {
    let mut m = Matrix4::<f32>::zeros();
    for r in 0..4 {
        for c in 0..4 {
            m[(r, c)] = buf[(r * 4) + c];
        }
    }
    m
}

/// Create a new engine handle (empty map). Free with `..._ndt_engine_free`.
#[expect(unsafe_code, reason = "C ABI boundary; returns an owned handle")]
#[unsafe(no_mangle)]
pub extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_new(
    resolution: f64,
    min_points: i32,
    eig_mult: f64,
) -> *mut NdtEngine {
    ffi_ptr::into_handle(NdtEngine::new(resolution, min_points, eig_mult))
}

/// # Safety
/// `engine` is a handle from `..._ndt_engine_new`/`_clone` (or null → no-op); not used afterwards.
#[expect(unsafe_code, reason = "C ABI boundary; reclaims an owned handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_free(engine: *mut NdtEngine) {
    // SAFETY: `engine` is a handle from `_new`/`_clone` (or null → no-op); reclaimed once in ffi_ptr.
    unsafe { ffi_ptr::free_handle(engine) };
}

/// Deep-copy an engine (the node's map-update double-buffer). Null → null.
/// # Safety
/// `engine` is a valid handle or null.
#[expect(unsafe_code, reason = "C ABI boundary; clones an owned handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_clone(
    engine: *const NdtEngine,
) -> *mut NdtEngine {
    // SAFETY: `engine` is a valid handle or null; deep-copied once in ffi_ptr.
    unsafe { ffi_ptr::clone_handle(engine) }
}

/// # Safety
/// `engine` is a valid handle (or null → no-op).
#[expect(unsafe_code, reason = "C ABI boundary; dereferences a shared handle")]
#[allow(
    clippy::cast_sign_loss,
    clippy::as_conversions,
    clippy::allow_attributes,
    reason = "num_threads is a non-negative count crossing the C ABI as i32"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_set_params(
    engine: *const NdtEngine,
    trans_epsilon: f64,
    step_size: f64,
    resolution: f64,
    max_iterations: i32,
    outlier_ratio: f64,
    num_threads: i32,
) {
    let e = ffi_ref!(engine, else return);
    e.set_params(
        trans_epsilon,
        step_size,
        resolution,
        max_iterations,
        outlier_ratio,
        num_threads.max(0) as usize,
    );
}

/// # Safety
/// `engine` is a valid handle (or null → no-op).
#[expect(unsafe_code, reason = "C ABI boundary; dereferences a shared handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_set_regularization(
    engine: *const NdtEngine,
    pose_x: f32,
    pose_y: f32,
    scale: f32,
) {
    let e = ffi_ref!(engine, else return);
    e.set_regularization(pose_x, pose_y, scale);
}

/// Add a map tile of `n` xyz `f32` triples (`3 * n` floats) under `id`.
/// # Safety
/// `engine` is a valid handle (or null → no-op); `points` addresses `3 * n` readable `f32`.
#[expect(unsafe_code, reason = "C ABI boundary; reads a caller-owned cloud")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_add_target(
    engine: *const NdtEngine,
    points: *const f32,
    n: usize,
    id: u64,
) {
    let e = ffi_ref!(engine, else return);
    let pts = ffi_slice!(points, n, [f32; 3], else return);
    e.add_target(pts, id);
}

/// # Safety
/// `engine` is a valid handle (or null → no-op).
#[expect(unsafe_code, reason = "C ABI boundary; dereferences a shared handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_remove_target(
    engine: *const NdtEngine,
    id: u64,
) {
    ffi_ref!(engine, else return).remove_target(id);
}

/// Add a target tile keyed by the cell-id bytes (`points` is `3 * n` f32; `id` is `id_len` bytes).
/// The engine owns the cell-id → tile mapping. No-op if `engine`/`points` is null.
/// # Safety
/// `engine` is a valid handle (or null → no-op); `points` addresses `3 * n` f32; `id` addresses
/// `id_len` readable bytes (or null with `id_len` 0).
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads a caller-owned cloud + id bytes"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_add_target_str(
    engine: *const NdtEngine,
    points: *const f32,
    n: usize,
    id: *const u8,
    id_len: usize,
) {
    let e = ffi_ref!(engine, else return);
    let pts = ffi_slice!(points, n, [f32; 3], else return);
    // SAFETY: caller guarantees `id_len` readable bytes (or null/0 → empty), audited in ffi_ptr.
    let id_bytes = unsafe { ffi_ptr::slice_or_empty(id, id_len) };
    e.add_target_bytes(pts, id_bytes);
}

/// Remove the tile registered under the cell-id bytes (`id` is `id_len` bytes). No-op if `engine` is
/// null or the id is unknown.
/// # Safety
/// `engine` is a valid handle (or null → no-op); `id` addresses `id_len` readable bytes (or null/0).
#[expect(unsafe_code, reason = "C ABI boundary; reads caller-owned id bytes")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_remove_target_str(
    engine: *const NdtEngine,
    id: *const u8,
    id_len: usize,
) {
    let e = ffi_ref!(engine, else return);
    // SAFETY: caller guarantees `id_len` readable bytes (or null/0 → empty), audited in ffi_ptr.
    let id_bytes = unsafe { ffi_ptr::slice_or_empty(id, id_len) };
    e.remove_target_bytes(id_bytes);
}

/// Write the current tile cell-ids (`BTreeMap`-sorted). Always writes `*out_count` (number of ids)
/// and `*out_total_len` (sum of id byte lengths). If `out_lengths`/`out_bytes` are non-null and
/// `lengths_cap`/`bytes_cap` are large enough, fills `out_lengths[0..count]` (per-id byte lengths) and
/// `out_bytes[0..total_len]` (the ids concatenated, no separators). Two-pass: call with caps 0 to
/// size, allocate, call again. No-op if `engine`/`out_count`/`out_total_len` is null.
/// # Safety
/// `engine` is a valid handle (or null → no-op). `out_count`/`out_total_len` are writable `u32` (or
/// null → no-op). `out_lengths` addresses `lengths_cap` `u32` and `out_bytes` `bytes_cap` bytes (or
/// either null → that buffer is skipped).
#[expect(
    unsafe_code,
    reason = "C ABI boundary; writes caller-owned bounded buffers"
)]
#[allow(
    clippy::arithmetic_side_effects,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::indexing_slicing,
    clippy::allow_attributes,
    reason = "bounded id-buffer marshaling: offsets sum to total_len, writes capped"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_get_current_map_ids(
    engine: *const NdtEngine,
    out_lengths: *mut u32,
    lengths_cap: u32,
    out_bytes: *mut u8,
    bytes_cap: u32,
    out_count: *mut u32,
    out_total_len: *mut u32,
) {
    // All-or-nothing: engine + both counters must be non-null before anything is written.
    let e = ffi_ref!(engine, else return);
    let out_count = ffi_mut!(out_count, else return);
    let out_total_len = ffi_mut!(out_total_len, else return);
    let st = e.load_state();
    let ids: alloc::vec::Vec<&[u8]> = st.id_map.keys().map(alloc::vec::Vec::as_slice).collect();
    let total: usize = ids.iter().map(|s| s.len()).sum();
    *out_count = ids.len() as u32;
    *out_total_len = total as u32;
    let fill_lengths = !out_lengths.is_null() && lengths_cap as usize >= ids.len();
    let fill_bytes = !out_bytes.is_null() && bytes_cap as usize >= total;
    if !fill_lengths || !fill_bytes {
        return;
    }
    // Caps verified ≥ the required sizes above; both buffers non-null.
    let lens = ffi_mut_slice!(out_lengths, ids.len(), else return);
    let bytes = ffi_mut_slice!(out_bytes, total, else return);
    let mut off = 0usize;
    for (k, id) in ids.iter().enumerate() {
        lens[k] = id.len() as u32;
        bytes[off..off + id.len()].copy_from_slice(id);
        off += id.len();
    }
}

/// # Safety
/// `engine` is a valid handle (or null → no-op).
#[expect(unsafe_code, reason = "C ABI boundary; dereferences a shared handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_create_kdtree(
    engine: *const NdtEngine,
) {
    ffi_ref!(engine, else return).create_kdtree();
}

/// Atomically publish `src`'s map state into `dst` (the map-update commit). No-op if either is null.
/// # Safety
/// `dst`/`src` are valid handles (or either null → no-op); both reborrowed as `&*ptr` only.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; dereferences two shared handles"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_commit_from(
    dst: *const NdtEngine,
    src: *const NdtEngine,
) {
    let d = ffi_ref!(dst, else return);
    let s = ffi_ref!(src, else return);
    d.commit_from(s);
}

/// # Safety
/// `engine` is a valid handle (or null → returns false).
#[expect(unsafe_code, reason = "C ABI boundary; dereferences a shared handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_has_target(
    engine: *const NdtEngine,
) -> bool {
    ffi_ref!(engine, else return false).has_target()
}

/// # Safety
/// `engine` is a valid handle (or null → returns 0).
#[expect(unsafe_code, reason = "C ABI boundary; dereferences a shared handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_max_iterations(
    engine: *const NdtEngine,
) -> i32 {
    ffi_ref!(engine, else return 0).max_iterations()
}

/// Align `source` (`3 * n` `f32`) from `guess` (16 row-major `f32`), storing the result in the
/// thread-local scratch (retrieve with `..._get_result`).
/// # Safety
/// `engine` is a valid handle (or null → no-op); `guess` addresses 16 `f32`, `source` `3 * n` `f32`.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads caller-owned guess + cloud"
)]
#[cfg(any(feature = "std", not(feature = "mt")))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_align(
    engine: *const NdtEngine,
    guess: *const f32,
    source: *const f32,
    n: usize,
) {
    let e = ffi_ref!(engine, else return);
    let guess_buf = ffi_slice!(guess, 16, else return);
    let src = ffi_slice!(source, n, [f32; 3], else return);
    e.align(&matrix4_from_row_major(guess_buf), src);
}

/// Score `cloud` (`3 * n` `f32`) without aligning.
/// # Safety
/// `engine` is a valid handle (or null → returns 0); `cloud` addresses `3 * n` readable `f32`.
#[expect(unsafe_code, reason = "C ABI boundary; reads a caller-owned cloud")]
#[cfg(any(feature = "std", not(feature = "mt")))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_calc_transformation_probability(
    engine: *const NdtEngine,
    cloud: *const f32,
    n: usize,
) -> f64 {
    let e = ffi_ref!(engine, else return 0.0);
    let c = ffi_slice!(cloud, n, [f32; 3], else return 0.0);
    e.calc_transformation_probability(c)
}

/// Nearest-voxel likelihood of `cloud` (`3 * n` `f32`) without aligning.
/// # Safety
/// `engine` is a valid handle (or null → returns 0); `cloud` addresses `3 * n` readable `f32`.
#[expect(unsafe_code, reason = "C ABI boundary; reads a caller-owned cloud")]
#[cfg(any(feature = "std", not(feature = "mt")))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_calc_nearest_voxel_likelihood(
    engine: *const NdtEngine,
    cloud: *const f32,
    n: usize,
) -> f64 {
    let e = ffi_ref!(engine, else return 0.0);
    let c = ffi_slice!(cloud, n, [f32; 3], else return 0.0);
    e.calc_nearest_voxel_likelihood(c)
}

/// Write the last align result into the C output buffers (same shape/contract as the `ndt_align`
/// FFI; each field null-skippable).
/// # Safety
/// `engine` is a valid handle (or null → no-op); `output` is a valid `AwNdtAlignOutput` whose
/// non-null pointers address their documented lengths.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; writes caller-owned output buffers"
)]
#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::allow_attributes,
    reason = "fixed-size matrix marshaling (r/c in 0..4 / 0..6); bounded transform-array copy"
)]
#[cfg(any(feature = "std", not(feature = "mt")))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_get_result(
    engine: *const NdtEngine,
    output: *const AwNdtAlignOutput,
) {
    let e = ffi_ref!(engine, else return);
    let out = ffi_ref!(output, else return);
    let result = e.result();
    // SAFETY: each non-null output pointer addresses its documented length; the derefs are audited
    // in ffi_ptr (null → skipped: `opt_slice_mut` → None, `write_out` → false without writing).
    unsafe {
        if let Some(pose) = ffi_ptr::opt_slice_mut(out.pose, 16) {
            for r in 0..4 {
                for c in 0..4 {
                    pose[(r * 4) + c] = result.pose[(r, c)];
                }
            }
        }
        ffi_ptr::write_out(out.iteration_num, result.iteration_num);
        ffi_ptr::write_out(out.transform_probability, result.transform_probability);
        ffi_ptr::write_out(
            out.nearest_voxel_likelihood,
            result.nearest_voxel_likelihood,
        );
        if let Some(h) = ffi_ptr::opt_slice_mut(out.hessian, 36) {
            for r in 0..6 {
                for c in 0..6 {
                    h[(r * 6) + c] = result.hessian[(r, c)];
                }
            }
        }
        if !out.transformation_array.is_null() && !out.transforms_count.is_null() {
            let cap = out.transforms_cap as usize;
            if let Some(buf) = ffi_ptr::opt_slice_mut(out.transformation_array, cap * 16) {
                for (k, m) in result.transformation_array.iter().take(cap).enumerate() {
                    for r in 0..4 {
                        for c in 0..4 {
                            buf[(k * 16) + (r * 4) + c] = m[(r, c)];
                        }
                    }
                }
            }
            ffi_ptr::write_out(
                out.transforms_count,
                result.transformation_array.len() as u32,
            );
        }
    }
}

/// Per-point nearest-voxel score for `cloud` (`3 * n` `f32`): writes `n` scores to `out_scores`
/// (`out_scores[i] > 0` iff point `i` found a neighbor).
/// # Safety
/// `engine` is a valid handle (or null → no-op); `cloud` addresses `3 * n` `f32`, `out_scores` `n`.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads a cloud, writes per-point scores"
)]
#[cfg(any(feature = "std", not(feature = "mt")))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_calc_nearest_voxel_score_each_point(
    engine: *const NdtEngine,
    cloud: *const f32,
    n: usize,
    out_scores: *mut f32,
) {
    let e = ffi_ref!(engine, else return);
    let c = ffi_slice!(cloud, n, [f32; 3], else return);
    let out = ffi_mut_slice!(out_scores, n, else return);
    let mut scores = alloc::vec::Vec::new();
    e.nearest_voxel_score_each_point(c, &mut scores);
    out.copy_from_slice(&scores);
}

/// Write the last align's per-iteration score traces. `out_tp`/`out_nvl` receive up to `cap` `f32`
/// each; `*out_count` gets the true length (== `iteration_num + 1`).
/// # Safety
/// `engine` is a valid handle (or null → no-op); `out_tp`/`out_nvl` address `cap` `f32`, `out_count`
/// one `u32`.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; writes caller-owned score-trace buffers"
)]
#[allow(
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::indexing_slicing,
    clippy::allow_attributes,
    reason = "trace length crosses the C ABI as u32; k = min(len, cap) bounds the slicing"
)]
#[cfg(any(feature = "std", not(feature = "mt")))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_get_score_arrays(
    engine: *const NdtEngine,
    out_tp: *mut f32,
    out_nvl: *mut f32,
    cap: u32,
    out_count: *mut u32,
) {
    let e = ffi_ref!(engine, else return);
    let tp_out = ffi_mut_slice!(out_tp, cap as usize, else return);
    let nvl_out = ffi_mut_slice!(out_nvl, cap as usize, else return);
    let count = ffi_mut!(out_count, else return);
    let (tp, nvl) = e.score_arrays();
    let k = tp.len().min(cap as usize);
    tp_out[..k].copy_from_slice(&tp[..k]);
    nvl_out[..k].copy_from_slice(&nvl[..k]);
    *count = tp.len() as u32;
}

// --- sensor-callback align orchestrator (std-gated node glue) ---
// Folds align + oscillation count + the convergence verdict into one Rust call against the live
// engine, so the C++ sensor callback no longer drives align through separate C++ glue plus the
// C++ `count_oscillation` and `evaluate_convergence` FFI. Reuses the existing engine align,
// `helper::count_oscillation`, and `convergence::evaluate_convergence` — all no_std, so this is part
// of the no_std rlib (the C-ABI `extern "C"` wrapper below stays `std`-gated).

/// The `score_estimation` scalars that gate convergence (mirrors `AwConvergenceInput`'s param fields).
#[derive(Clone, Copy, Debug, Default)]
pub struct ConvergenceParams {
    pub converged_param_type: i32,
    pub converged_param_transform_probability: f64,
    pub converged_param_nearest_voxel_transformation_likelihood: f64,
}

/// Result of [`run_align`]: the scalars + convergence verdict the C++ sensor callback needs (the
/// variable-length arrays/marker are still read via `get_result`).
#[derive(Clone, Copy, Debug)]
pub struct AlignOutcome {
    pub pose: Matrix4<f32>,
    pub transform_probability: f32,
    pub nearest_voxel_likelihood: f32,
    pub iteration_num: i32,
    pub max_iterations: i32,
    pub oscillation_num: i32,
    pub verdict: crate::convergence::ConvergenceVerdict,
}

/// Align the live engine from `guess`, then derive the oscillation count (from the iteration
/// trajectory translations, matching the C++ `count_oscillation(transformation_msg_array)`) and the
/// convergence verdict. Pure orchestration over existing ports — no new math. Runs on the implicit
/// per-thread/engine scratch; absent under `mt` (use [`run_align_with`]).
#[cfg(any(feature = "std", not(feature = "mt")))]
#[must_use]
pub fn run_align(
    engine: &NdtEngine,
    guess: &Matrix4<f32>,
    source: &[[f32; 3]],
    conv: &ConvergenceParams,
) -> AlignOutcome {
    engine.with_scratch(|scr| run_align_with(engine, guess, source, conv, scr))
}

/// [`run_align`] with an explicit caller-owned scratch — the whole align + verdict reads one
/// scratch session, so it is safe under concurrent aligns in every config (the only variant
/// available under `mt`). The full [`AlignResult`] (e.g. the hessian) stays readable from
/// `scratch` afterwards via [`MatchScratch::result`].
#[must_use]
pub fn run_align_with(
    engine: &NdtEngine,
    guess: &Matrix4<f32>,
    source: &[[f32; 3]],
    conv: &ConvergenceParams,
    scratch: &mut MatchScratch,
) -> AlignOutcome {
    engine.align_with(guess, source, scratch);
    let max_iterations = engine.max_iterations();
    let result = &scratch.last;
    let positions: alloc::vec::Vec<[f64; 3]> = result
        .transformation_array
        .iter()
        .map(|m| {
            [
                f64::from(m[(0, 3)]),
                f64::from(m[(1, 3)]),
                f64::from(m[(2, 3)]),
            ]
        })
        .collect();
    let oscillation_num = crate::helper::count_oscillation(&positions);
    let verdict = crate::convergence::evaluate_convergence(&crate::convergence::ConvergenceInput {
        iteration_num: result.iteration_num,
        max_iterations,
        oscillation_num,
        transform_probability: f64::from(result.transform_probability),
        nearest_voxel_transformation_likelihood: f64::from(result.nearest_voxel_likelihood),
        converged_param_type: conv.converged_param_type,
        converged_param_transform_probability: conv.converged_param_transform_probability,
        converged_param_nearest_voxel_transformation_likelihood: conv
            .converged_param_nearest_voxel_transformation_likelihood,
    });
    AlignOutcome {
        pose: result.pose,
        transform_probability: result.transform_probability,
        nearest_voxel_likelihood: result.nearest_voxel_likelihood,
        iteration_num: result.iteration_num,
        max_iterations,
        oscillation_num,
        verdict,
    }
}

/// C ABI mirror of [`ConvergenceParams`] (same field order/types). Plain scalars.
#[cfg(feature = "std")]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AwAlignParams {
    pub converged_param_type: i32,
    pub converged_param_transform_probability: f64,
    pub converged_param_nearest_voxel_transformation_likelihood: f64,
}

/// C ABI result of [`autoware_ndt_scan_matcher_rs_node_run_align`]. `pose` is row-major 4x4; the
/// embedded verdict is the existing [`crate::node::AwConvergenceVerdict`].
#[cfg(feature = "std")]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AwAlignOutcome {
    pub pose: [f32; 16],
    pub transform_probability: f32,
    pub nearest_voxel_likelihood: f32,
    pub iteration_num: i32,
    pub max_iterations: i32,
    pub oscillation_num: i32,
    pub verdict: crate::node::AwConvergenceVerdict,
}

/// FFI entry: align the live engine + return the scalars/verdict ([`run_align`]). No-op if any
/// pointer is null.
///
/// # Safety
/// `engine` is a valid live handle (or null → no-op). It is reborrowed as `&NdtEngine` (the engine is
/// `Sync` in this std build — as under `mt`; only the plain `no_std` single-core engine is not — so
/// concurrent `&self` calls on a shared `const AwNdtEngine*` are sound — no external lock
/// required). Rust does not retain the pointer past the call. `guess` addresses 16 `f32`, `source`
/// `3 * n` `f32`, `params` a valid [`AwAlignParams`], `out` a writable [`AwAlignOutcome`].
#[cfg(feature = "std")]
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads caller-owned guess/cloud/params, validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_run_align(
    engine: *const NdtEngine,
    guess: *const f32,
    source: *const f32,
    n: usize,
    params: *const AwAlignParams,
    out: *mut AwAlignOutcome,
) {
    if engine.is_null() || guess.is_null() || source.is_null() || params.is_null() || out.is_null()
    {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees the documented lengths + valid structs.
    let (eng, guess_buf, src, prm) = unsafe {
        (
            &*engine,
            core::slice::from_raw_parts(guess, 16),
            core::slice::from_raw_parts(source.cast::<[f32; 3]>(), n),
            &*params,
        )
    };
    let outcome = run_align(
        eng,
        &matrix4_from_row_major(guess_buf),
        src,
        &ConvergenceParams {
            converged_param_type: prm.converged_param_type,
            converged_param_transform_probability: prm.converged_param_transform_probability,
            converged_param_nearest_voxel_transformation_likelihood: prm
                .converged_param_nearest_voxel_transformation_likelihood,
        },
    );
    let mat = &outcome.pose;
    // Row-major 4x4 as an explicit literal (nalgebra `(r,c)` indexing; no array computed-index).
    let pose = [
        mat[(0, 0)],
        mat[(0, 1)],
        mat[(0, 2)],
        mat[(0, 3)],
        mat[(1, 0)],
        mat[(1, 1)],
        mat[(1, 2)],
        mat[(1, 3)],
        mat[(2, 0)],
        mat[(2, 1)],
        mat[(2, 2)],
        mat[(2, 3)],
        mat[(3, 0)],
        mat[(3, 1)],
        mat[(3, 2)],
        mat[(3, 3)],
    ];
    let vd = &outcome.verdict;
    // SAFETY: `out` is non-null per the check and a valid, writable AwAlignOutcome per the contract.
    unsafe {
        *out = AwAlignOutcome {
            pose,
            transform_probability: outcome.transform_probability,
            nearest_voxel_likelihood: outcome.nearest_voxel_likelihood,
            iteration_num: outcome.iteration_num,
            max_iterations: outcome.max_iterations,
            oscillation_num: outcome.oscillation_num,
            verdict: crate::node::AwConvergenceVerdict {
                valid_param_type: vd.valid_param_type,
                is_ok_iteration_num: vd.is_ok_iteration_num,
                is_local_optimal_solution_oscillation: vd.is_local_optimal_solution_oscillation,
                is_ok_score: vd.is_ok_score,
                is_converged: vd.is_converged,
                score: vd.score,
                score_threshold: vd.score_threshold,
            },
        };
    }
}

// --- sensor-callback covariance orchestrator ---
// Folds the whole covariance block of callback_sensor_points_main (rotate the configured 6x6 cov,
// dispatch on the estimation type against the LIVE engine map, scale, and adjust) into one Rust call,
// so the C++ node no longer calls the templated estimate_xy_covariance_by_multi_ndt[_score],
// adjust_diagonal_covariance, propose_poses_to_search, or the rotate_covariance helper twin. Reuses
// crate::{helper,covariance,cov_estimate} — all no_std, so this orchestrator is part of the no_std rlib
// (only the C-ABI `Aw*` structs + the `extern "C"` wrapper below stay `std`-gated).

/// Params for [`estimate_pose_covariance`] (the `covariance` hyper-params + the result-pose rotation).
#[derive(Clone, Copy, Debug)]
pub struct CovEstimationParams {
    /// `0` = `FIXED_VALUE`, `1` = `LAPLACE_APPROXIMATION`, `2` = `MULTI_NDT`, `3` = `MULTI_NDT_SCORE`.
    pub estimation_type: i32,
    pub scale_factor: f64,
    pub temperature: f64,
    /// The main result's nearest-voxel likelihood (used by the `MULTI_NDT_SCORE` softmax).
    pub main_nvtl: f32,
    pub output_pose_covariance: [f64; 36],
    /// Row-major 3x3 map-from-`base_link` rotation, built C++-side from the result-pose quaternion.
    pub map_to_base_link_rot3x3: [f64; 9],
}

/// Result of [`estimate_pose_covariance`]: the full 6x6 output covariance + the debug pose arrays the
/// node publishes (`publish_kind`: `0` none, `1` `MULTI_NDT` publishes both, `2` `MULTI_NDT_SCORE`
/// publishes only the initial poses). Each pose vec is `[main, then per-candidate]`.
#[derive(Clone, Debug)]
pub struct CovEstimationResult {
    pub ndt_covariance: [f64; 36],
    pub publish_kind: i32,
    pub multi_ndt_result_poses: alloc::vec::Vec<Matrix4<f32>>,
    pub multi_initial_poses: alloc::vec::Vec<Matrix4<f32>>,
}

/// Pure covariance orchestrator, ported verbatim from `callback_sensor_points_main`'s covariance block
/// and the `estimate_covariance` method. Runs the `MULTI_NDT`/`MULTI_NDT_SCORE` re-align/score against
/// the live engine map (a loaded snapshot) — never a rebuilt one-tile map. No new math (wires ports).
/// `scratch` backs the candidate re-aligns/scores (caller-owned; the std FFI supplies its
/// thread-local one).
#[expect(
    clippy::too_many_arguments,
    reason = "covariance orchestrator wires the engine + result/initial poses + hessian + source + offsets + params + scratch; grouping would only relocate the same inputs"
)]
#[must_use]
pub fn estimate_pose_covariance(
    engine: &NdtEngine,
    result_pose: &Matrix4<f32>,
    hessian: &[f64; 36],
    initial_pose: &Matrix4<f32>,
    source: &[[f32; 3]],
    offset_x: &[f64],
    offset_y: &[f64],
    params: &CovEstimationParams,
    scratch: &mut MatchScratch,
) -> CovEstimationResult {
    use alloc::vec::Vec;

    let mut ndt_covariance = crate::helper::rotate_covariance(
        &params.output_pose_covariance,
        &params.map_to_base_link_rot3x3,
    );

    // FIXED_VALUE: the rotated configured covariance only (no 2x2 estimate/adjust — C++ short-circuit).
    if params.estimation_type == 0 {
        return CovEstimationResult {
            ndt_covariance,
            publish_kind: 0,
            multi_ndt_result_poses: Vec::new(),
            multi_initial_poses: Vec::new(),
        };
    }

    let st = engine.load_state();
    let main_ndt = AlignResult {
        pose: *result_pose,
        nearest_voxel_likelihood: params.main_nvtl,
        ..AlignResult::default()
    };
    let mut publish_kind = 0_i32;
    let mut multi_ndt_result_poses: Vec<Matrix4<f32>> = Vec::new();
    let mut multi_initial_poses: Vec<Matrix4<f32>> = Vec::new();

    let est: [f64; 4] = match params.estimation_type {
        // LAPLACE_APPROXIMATION
        1 => crate::covariance::laplace_xy_covariance(hessian),
        // MULTI_NDT: re-align each candidate against the live map; publish result + initial poses.
        2 => {
            let poses =
                crate::cov_estimate::propose_poses_to_search(result_pose, offset_x, offset_y);
            let r = crate::cov_estimate::estimate_xy_covariance_by_multi_ndt(
                &main_ndt,
                &poses,
                &st.map,
                source,
                &st.params,
                &mut scratch.workspace,
            );
            publish_kind = 1;
            multi_ndt_result_poses.push(*result_pose);
            multi_ndt_result_poses.extend_from_slice(&r.candidate_result_poses);
            multi_initial_poses.push(*initial_pose);
            multi_initial_poses.extend_from_slice(&poses);
            r.covariance
        }
        // MULTI_NDT_SCORE: score each candidate (no re-align); publish only the initial poses.
        3 => {
            let poses =
                crate::cov_estimate::propose_poses_to_search(result_pose, offset_x, offset_y);
            let r = crate::cov_estimate::estimate_xy_covariance_by_multi_ndt_score(
                &main_ndt,
                &poses,
                &st.map,
                source,
                &st.params,
                params.temperature,
                &mut scratch.workspace,
            );
            publish_kind = 2;
            multi_initial_poses.push(*initial_pose);
            multi_initial_poses.extend_from_slice(&poses);
            r.covariance
        }
        // Unknown type: C++ returns Identity * output_pose_covariance[0] (still scaled+adjusted below).
        _ => {
            let c0 = params.output_pose_covariance[0];
            [c0, 0.0, 0.0, c0]
        }
    };

    // scale (C++ `* scale_factor`) then adjust_diagonal_covariance with the result-pose 2x2 rotation.
    let scaled = [
        est[0] * params.scale_factor,
        est[1] * params.scale_factor,
        est[2] * params.scale_factor,
        est[3] * params.scale_factor,
    ];
    let rot2 = [
        f64::from(result_pose[(0, 0)]),
        f64::from(result_pose[(0, 1)]),
        f64::from(result_pose[(1, 0)]),
        f64::from(result_pose[(1, 1)]),
    ];
    let adj = crate::covariance::adjust_diagonal_covariance(
        &scaled,
        &rot2,
        params.output_pose_covariance[0],
        params.output_pose_covariance[7],
    );
    // C++: ndt_covariance[0+6*0]=adj(0,0), [1+6*1]=adj(1,1), [1+6*0]=adj(1,0), [0+6*1]=adj(0,1).
    ndt_covariance[0] = adj[0];
    ndt_covariance[7] = adj[3];
    ndt_covariance[1] = adj[2];
    ndt_covariance[6] = adj[1];

    CovEstimationResult {
        ndt_covariance,
        publish_kind,
        multi_ndt_result_poses,
        multi_initial_poses,
    }
}

/// C ABI mirror of [`CovEstimationParams`] + the align inputs. `result_pose`/`initial_pose` are
/// row-major 4x4; `hessian` row-major 6x6; `output_pose_covariance` row-major 6x6; `rot3x3` row-major.
#[cfg(feature = "std")]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AwCovEstimationInput {
    pub result_pose: [f32; 16],
    pub hessian: [f64; 36],
    pub initial_pose: [f32; 16],
    pub source: *const f32,
    pub n_source: usize,
    pub estimation_type: i32,
    pub offset_x: *const f64,
    pub offset_y: *const f64,
    pub n_offsets: usize,
    pub scale_factor: f64,
    pub temperature: f64,
    pub main_nvtl: f32,
    pub output_pose_covariance: [f64; 36],
    pub map_to_base_link_rot3x3: [f64; 9],
}

/// C ABI output: `ndt_covariance` (36, written) + `publish_kind` (written) + two caller-owned pose
/// buffers (`pose_cap` poses of 16 row-major floats each) filled up to cap, with the true counts
/// written back (cap+count contract, like `get_score_arrays`).
#[cfg(feature = "std")]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AwCovEstimationOutput {
    pub ndt_covariance: [f64; 36],
    pub publish_kind: i32,
    pub multi_ndt_result_poses: *mut f32,
    pub multi_initial_poses: *mut f32,
    pub pose_cap: u32,
    pub multi_ndt_result_count: u32,
    pub multi_initial_count: u32,
}

/// FFI entry for the covariance orchestrator ([`estimate_pose_covariance`]). No-op if any pointer is
/// null.
///
/// # Safety
/// `engine` is a valid live handle (or null → no-op), reborrowed `&NdtEngine` for the call (the engine
/// is `Sync` in this std build — as under `mt`; only the plain `no_std` single-core engine is not —
/// so concurrent `&self` calls are sound, no external lock required). `input` is a valid
/// [`AwCovEstimationInput`] whose `source` (`3*n_source` f32) and `offset_x`/`offset_y` (`n_offsets`
/// f64) are readable. `output` is a writable [`AwCovEstimationOutput`] whose pose buffers each address
/// `pose_cap * 16` writable f32 (or are null to skip). Nothing retained.
#[cfg(feature = "std")]
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads caller-owned cloud/offsets, writes caller-owned buffers"
)]
#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::allow_attributes,
    reason = "fixed-size 4x4 pose marshaling into the caller's bounded pose buffers"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_estimate_pose_covariance(
    engine: *const NdtEngine,
    input: *const AwCovEstimationInput,
    output: *mut AwCovEstimationOutput,
) {
    if engine.is_null() || input.is_null() || output.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid live handle + input struct.
    let (eng, inp) = unsafe { (&*engine, &*input) };
    if inp.source.is_null() || inp.offset_x.is_null() || inp.offset_y.is_null() {
        return;
    }
    // SAFETY: caller guarantees the documented lengths.
    let (source, ox, oy) = unsafe {
        (
            core::slice::from_raw_parts(inp.source.cast::<[f32; 3]>(), inp.n_source),
            core::slice::from_raw_parts(inp.offset_x, inp.n_offsets),
            core::slice::from_raw_parts(inp.offset_y, inp.n_offsets),
        )
    };
    let result = eng.with_scratch(|scr| {
        estimate_pose_covariance(
            eng,
            &matrix4_from_row_major(&inp.result_pose),
            &inp.hessian,
            &matrix4_from_row_major(&inp.initial_pose),
            source,
            ox,
            oy,
            &CovEstimationParams {
                estimation_type: inp.estimation_type,
                scale_factor: inp.scale_factor,
                temperature: inp.temperature,
                main_nvtl: inp.main_nvtl,
                output_pose_covariance: inp.output_pose_covariance,
                map_to_base_link_rot3x3: inp.map_to_base_link_rot3x3,
            },
            scr,
        )
    });
    // SAFETY: `output` is non-null per the check and a writable AwCovEstimationOutput per the contract.
    let out = unsafe { &mut *output };
    out.ndt_covariance = result.ndt_covariance;
    out.publish_kind = result.publish_kind;
    let cap = out.pose_cap as usize;
    // SAFETY: each non-null pose buffer addresses `pose_cap * 16` writable f32 per the contract.
    unsafe {
        out.multi_ndt_result_count = fill_pose_buffer(
            out.multi_ndt_result_poses,
            cap,
            &result.multi_ndt_result_poses,
        );
        out.multi_initial_count =
            fill_pose_buffer(out.multi_initial_poses, cap, &result.multi_initial_poses);
    }
}

/// Write up to `cap` poses (16 row-major f32 each) into `buf`; return the TRUE pose count.
///
/// # Safety
/// `buf` is null (→ no write) or addresses `cap * 16` writable f32.
#[cfg(feature = "std")]
#[expect(
    unsafe_code,
    reason = "C ABI boundary; writes a caller-owned bounded f32 buffer"
)]
#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::allow_attributes,
    reason = "fixed-size 4x4 pose marshaling into a bounded buffer"
)]
pub(crate) unsafe fn fill_pose_buffer(buf: *mut f32, cap: usize, poses: &[Matrix4<f32>]) -> u32 {
    let count = poses.len() as u32;
    if !buf.is_null() && cap > 0 {
        // SAFETY: per the contract, `buf` addresses `cap * 16` writable f32.
        let slice = unsafe { core::slice::from_raw_parts_mut(buf, cap * 16) };
        for (k, m) in poses.iter().take(cap).enumerate() {
            for r in 0..4 {
                for c in 0..4 {
                    slice[(k * 16) + (r * 4) + c] = m[(r, c)];
                }
            }
        }
    }
    count
}

#[cfg(test)]
#[allow(
    unsafe_code,
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::arithmetic_side_effects,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::unreadable_literal,
    clippy::borrow_as_ptr,
    clippy::too_many_lines,
    clippy::allow_attributes,
    reason = "test code"
)]
mod tests {
    use super::*;
    use crate::ndt::{NdtParams, align};
    use alloc::vec::Vec;

    fn dense_cluster(cx: f32, cy: f32, cz: f32) -> Vec<[f32; 3]> {
        (0..8)
            .map(|i| {
                let f = i as f32 * 0.02;
                [cx + f, cy - f, cz + 0.5 * f]
            })
            .collect()
    }

    fn configured(engine: &mut NdtEngine) {
        engine.set_params(0.01, 0.1, 1.0, 30, 0.55, 1);
    }

    // Building the map incrementally through the engine + aligning equals a one-shot ndt::align on a
    // map with the same two tiles (the engine is a thin stateful wrapper — bit-identical).
    #[test]
    fn engine_align_matches_one_shot() {
        let tile_a = dense_cluster(0.5, 0.5, 0.5);
        let tile_b = dense_cluster(4.5, 0.5, 0.5);
        let source: Vec<[f32; 3]> = tile_a
            .iter()
            .chain(tile_b.iter())
            .map(|p| [p[0] + 0.1, p[1] - 0.05, p[2]])
            .collect();
        let guess = Matrix4::<f32>::identity();

        let mut engine = NdtEngine::new(1.0, 6, 0.01);
        configured(&mut engine);
        engine.add_target(&tile_a, 0);
        engine.add_target(&tile_b, 1);
        engine.create_kdtree();
        assert!(engine.has_target());
        let mut scratch = MatchScratch::new();
        engine.align_with(&guess, &source, &mut scratch);
        let got = scratch.result();

        let mut map = VoxelGridMap::new([1.0; 3], 6, 0.01);
        map.add_target(&tile_a, 0);
        map.add_target(&tile_b, 1);
        map.create_kdtree();
        let params = NdtParams {
            trans_epsilon: 0.01,
            step_size: 0.1,
            resolution: 1.0,
            max_iterations: 30,
            outlier_ratio: 0.55,
            regularization: None,
            num_threads: 1,
        };
        let mut ws = AlignWorkspace::new();
        let mut want = AlignResult::default();
        align(&map, &source, &guess, &params, &mut ws, &mut want);

        assert_eq!(got.iteration_num, want.iteration_num);
        assert_eq!(got.pose, want.pose);
        assert_eq!(got.hessian, want.hessian);
        assert_eq!(got.transform_probability, want.transform_probability);
        assert_eq!(got.nearest_voxel_likelihood, want.nearest_voxel_likelihood);
    }

    fn two_tile_engine() -> (NdtEngine, Vec<[f32; 3]>) {
        let tile_a = dense_cluster(0.5, 0.5, 0.5);
        let tile_b = dense_cluster(4.5, 0.5, 0.5);
        let source: Vec<[f32; 3]> = tile_a
            .iter()
            .chain(tile_b.iter())
            .map(|p| [p[0] + 0.1, p[1] - 0.05, p[2]])
            .collect();
        let mut engine = NdtEngine::new(1.0, 6, 0.01);
        configured(&mut engine);
        engine.add_target(&tile_a, 0);
        engine.add_target(&tile_b, 1);
        engine.create_kdtree();
        (engine, source)
    }

    const TP_PARAMS: ConvergenceParams = ConvergenceParams {
        converged_param_type: 0, // TRANSFORM_PROBABILITY
        converged_param_transform_probability: 0.0,
        converged_param_nearest_voxel_transformation_likelihood: 0.0,
    };

    // run_align == composing engine.align + result + helper::count_oscillation + evaluate_convergence
    // by hand (the orchestrator adds no new math, just folds the existing ports).
    #[test]
    fn run_align_matches_manual_composition() {
        let (engine, source) = two_tile_engine();
        let guess = Matrix4::<f32>::identity();
        let outcome = run_align_with(
            &engine,
            &guess,
            &source,
            &TP_PARAMS,
            &mut MatchScratch::new(),
        );

        // Manual reference on a fresh, identically-built engine.
        let (ref_engine, ref_source) = two_tile_engine();
        let mut ref_scratch = MatchScratch::new();
        ref_engine.align_with(&guess, &ref_source, &mut ref_scratch);
        let max_it = ref_engine.max_iterations();
        let r = ref_scratch.result();
        let positions: Vec<[f64; 3]> = r
            .transformation_array
            .iter()
            .map(|m| {
                [
                    f64::from(m[(0, 3)]),
                    f64::from(m[(1, 3)]),
                    f64::from(m[(2, 3)]),
                ]
            })
            .collect();
        let osc = crate::helper::count_oscillation(&positions);
        let verdict =
            crate::convergence::evaluate_convergence(&crate::convergence::ConvergenceInput {
                iteration_num: r.iteration_num,
                max_iterations: max_it,
                oscillation_num: osc,
                transform_probability: f64::from(r.transform_probability),
                nearest_voxel_transformation_likelihood: f64::from(r.nearest_voxel_likelihood),
                converged_param_type: 0,
                converged_param_transform_probability: 0.0,
                converged_param_nearest_voxel_transformation_likelihood: 0.0,
            });

        assert_eq!(outcome.pose, r.pose);
        assert_eq!(outcome.transform_probability, r.transform_probability);
        assert_eq!(outcome.nearest_voxel_likelihood, r.nearest_voxel_likelihood);
        assert_eq!(outcome.iteration_num, r.iteration_num);
        assert_eq!(outcome.max_iterations, max_it);
        assert_eq!(outcome.oscillation_num, osc);
        assert_eq!(outcome.verdict, verdict);
    }

    // The FFI shim writes exactly what the pure run_align computes.
    #[cfg(feature = "std")]
    #[test]
    fn ffi_run_align_matches_pure() {
        let (engine, source) = two_tile_engine();
        let guess = Matrix4::<f32>::identity();
        let pure = run_align(&engine, &guess, &source, &TP_PARAMS);

        let (ffi_engine, ffi_source) = two_tile_engine();
        let guess16 = [
            1.0f32, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let flat: Vec<f32> = ffi_source.iter().flat_map(|p| *p).collect();
        let params = AwAlignParams {
            converged_param_type: 0,
            converged_param_transform_probability: 0.0,
            converged_param_nearest_voxel_transformation_likelihood: 0.0,
        };
        let mut out = AwAlignOutcome::default();
        // SAFETY: engine valid; guess 16 f32; flat 3*n f32; params/out valid.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_run_align(
                &ffi_engine,
                guess16.as_ptr(),
                flat.as_ptr(),
                ffi_source.len(),
                &params,
                &mut out,
            );
        }
        assert_eq!(out.transform_probability, pure.transform_probability);
        assert_eq!(out.nearest_voxel_likelihood, pure.nearest_voxel_likelihood);
        assert_eq!(out.iteration_num, pure.iteration_num);
        assert_eq!(out.max_iterations, pure.max_iterations);
        assert_eq!(out.oscillation_num, pure.oscillation_num);
        assert_eq!(out.verdict.is_converged, pure.verdict.is_converged);
        assert_eq!(out.pose[0], pure.pose[(0, 0)]);
        assert_eq!(out.pose[3], pure.pose[(0, 3)]);
        assert_eq!(out.pose[15], pure.pose[(3, 3)]);
    }

    // Null pointers are a no-op (no write, no panic).
    #[cfg(feature = "std")]
    #[test]
    fn ffi_run_align_null_is_noop() {
        let mut out = AwAlignOutcome {
            iteration_num: 123,
            ..AwAlignOutcome::default()
        };
        let params = AwAlignParams {
            converged_param_type: 0,
            converged_param_transform_probability: 0.0,
            converged_param_nearest_voxel_transformation_likelihood: 0.0,
        };
        let guess16 = [0.0f32; 16];
        // SAFETY: a null engine must leave `out` untouched.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_run_align(
                core::ptr::null_mut(),
                guess16.as_ptr(),
                guess16.as_ptr(),
                0,
                &params,
                &mut out,
            );
        }
        assert_eq!(out.iteration_num, 123);
    }

    // --- covariance orchestrator tests ---

    const ROT3X3_ID: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    const COV_OX: [f64; 4] = [0.3, -0.3, 0.0, 0.0];
    const COV_OY: [f64; 4] = [0.0, 0.0, 0.3, -0.3];

    fn out_cov() -> [f64; 36] {
        let mut c = [0.0_f64; 36];
        c[0] = 0.25;
        c[7] = 0.36;
        c[14] = 0.01;
        c[21] = 0.01;
        c[28] = 0.01;
        c[35] = 0.01;
        c
    }

    fn cov_params(estimation_type: i32, scale_factor: f64) -> CovEstimationParams {
        CovEstimationParams {
            estimation_type,
            scale_factor,
            temperature: 0.1,
            main_nvtl: 1.0,
            output_pose_covariance: out_cov(),
            map_to_base_link_rot3x3: ROT3X3_ID,
        }
    }

    // Align an engine once and return (engine, source, result_pose) for the cov tests.
    fn aligned_engine() -> (NdtEngine, Vec<[f32; 3]>, Matrix4<f32>) {
        let (engine, source) = two_tile_engine();
        let mut scratch = MatchScratch::new();
        engine.align_with(&Matrix4::<f32>::identity(), &source, &mut scratch);
        let pose = scratch.result().pose;
        (engine, source, pose)
    }

    fn rot2_of(pose: &Matrix4<f32>) -> [f64; 4] {
        [
            f64::from(pose[(0, 0)]),
            f64::from(pose[(0, 1)]),
            f64::from(pose[(1, 0)]),
            f64::from(pose[(1, 1)]),
        ]
    }

    #[test]
    fn estimate_pose_covariance_fixed_just_rotates() {
        let (engine, source, result_pose) = aligned_engine();
        let r = estimate_pose_covariance(
            &engine,
            &result_pose,
            &[0.0; 36],
            &Matrix4::identity(),
            &source,
            &COV_OX,
            &COV_OY,
            &cov_params(0, 1.0),
            &mut MatchScratch::new(),
        );
        assert_eq!(r.publish_kind, 0);
        assert!(r.multi_ndt_result_poses.is_empty() && r.multi_initial_poses.is_empty());
        // FIXED → just the rotated configured covariance (identity rotation → unchanged).
        assert_eq!(
            r.ndt_covariance,
            crate::helper::rotate_covariance(&out_cov(), &ROT3X3_ID)
        );
    }

    #[test]
    fn estimate_pose_covariance_multi_ndt_matches_composition() {
        let (engine, source, result_pose) = aligned_engine();
        let r = estimate_pose_covariance(
            &engine,
            &result_pose,
            &[0.0; 36],
            &Matrix4::identity(),
            &source,
            &COV_OX,
            &COV_OY,
            &cov_params(2, 2.0),
            &mut MatchScratch::new(),
        );

        // Manual reference on a fresh identical engine.
        let (ref_engine, ref_source) = two_tile_engine();
        let ref_state = ref_engine.load_state();
        let poses = crate::cov_estimate::propose_poses_to_search(&result_pose, &COV_OX, &COV_OY);
        let main_ndt = AlignResult {
            pose: result_pose,
            ..AlignResult::default()
        };
        let mut ws = AlignWorkspace::new();
        let mr = crate::cov_estimate::estimate_xy_covariance_by_multi_ndt(
            &main_ndt,
            &poses,
            &ref_state.map,
            &ref_source,
            &ref_state.params,
            &mut ws,
        );
        let scaled = [
            mr.covariance[0] * 2.0,
            mr.covariance[1] * 2.0,
            mr.covariance[2] * 2.0,
            mr.covariance[3] * 2.0,
        ];
        let adj = crate::covariance::adjust_diagonal_covariance(
            &scaled,
            &rot2_of(&result_pose),
            out_cov()[0],
            out_cov()[7],
        );
        let mut expected = crate::helper::rotate_covariance(&out_cov(), &ROT3X3_ID);
        expected[0] = adj[0];
        expected[7] = adj[3];
        expected[1] = adj[2];
        expected[6] = adj[1];

        assert_eq!(r.ndt_covariance, expected);
        assert_eq!(r.publish_kind, 1);
        assert_eq!(r.multi_ndt_result_poses.len(), poses.len() + 1);
        assert_eq!(r.multi_initial_poses.len(), poses.len() + 1);
        // The first published pose is the main result / initial pose.
        assert_eq!(r.multi_ndt_result_poses[0], result_pose);
        assert_eq!(r.multi_initial_poses[0], Matrix4::<f32>::identity());
    }

    #[test]
    fn estimate_pose_covariance_score_publishes_initial_only() {
        let (engine, source, result_pose) = aligned_engine();
        let r = estimate_pose_covariance(
            &engine,
            &result_pose,
            &[0.0; 36],
            &Matrix4::identity(),
            &source,
            &COV_OX,
            &COV_OY,
            &cov_params(3, 1.0),
            &mut MatchScratch::new(),
        );
        assert_eq!(r.publish_kind, 2);
        assert!(r.multi_ndt_result_poses.is_empty());
        assert_eq!(r.multi_initial_poses.len(), COV_OX.len() + 1);
    }

    #[cfg(feature = "std")]
    #[test]
    fn ffi_estimate_cov_matches_pure() {
        let (engine, source, result_pose) = aligned_engine();
        let pure = estimate_pose_covariance(
            &engine,
            &result_pose,
            &[0.0; 36],
            &Matrix4::identity(),
            &source,
            &COV_OX,
            &COV_OY,
            &cov_params(2, 2.0),
            &mut MatchScratch::new(),
        );

        let (ffi_engine, ffi_source) = two_tile_engine();
        let mut pose16 = [0.0_f32; 16];
        for r in 0..4 {
            for c in 0..4 {
                pose16[(r * 4) + c] = result_pose[(r, c)];
            }
        }
        let id16 = [
            1.0f32, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let flat: Vec<f32> = ffi_source.iter().flat_map(|p| *p).collect();
        let input = AwCovEstimationInput {
            result_pose: pose16,
            hessian: [0.0; 36],
            initial_pose: id16,
            source: flat.as_ptr(),
            n_source: ffi_source.len(),
            estimation_type: 2,
            offset_x: COV_OX.as_ptr(),
            offset_y: COV_OY.as_ptr(),
            n_offsets: COV_OX.len(),
            scale_factor: 2.0,
            temperature: 0.1,
            main_nvtl: 1.0,
            output_pose_covariance: out_cov(),
            map_to_base_link_rot3x3: ROT3X3_ID,
        };
        let cap = (COV_OX.len() + 1) as u32;
        let mut res_buf = vec![0.0_f32; (cap as usize) * 16];
        let mut init_buf = vec![0.0_f32; (cap as usize) * 16];
        let mut out = AwCovEstimationOutput {
            ndt_covariance: [0.0; 36],
            publish_kind: -1,
            multi_ndt_result_poses: res_buf.as_mut_ptr(),
            multi_initial_poses: init_buf.as_mut_ptr(),
            pose_cap: cap,
            multi_ndt_result_count: 0,
            multi_initial_count: 0,
        };
        // SAFETY: engine valid; input pointers valid for their lengths; out buffers sized cap*16.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_estimate_pose_covariance(
                &ffi_engine,
                &input,
                &mut out,
            );
        }
        assert_eq!(out.ndt_covariance, pure.ndt_covariance);
        assert_eq!(out.publish_kind, pure.publish_kind);
        assert_eq!(
            out.multi_ndt_result_count as usize,
            pure.multi_ndt_result_poses.len()
        );
        assert_eq!(
            out.multi_initial_count as usize,
            pure.multi_initial_poses.len()
        );
        // First marshaled result pose == pure's first result pose (row-major).
        assert_eq!(res_buf[0], pure.multi_ndt_result_poses[0][(0, 0)]);
        assert_eq!(res_buf[3], pure.multi_ndt_result_poses[0][(0, 3)]);
    }

    #[cfg(feature = "std")]
    #[test]
    fn ffi_estimate_cov_null_is_noop() {
        let mut out = AwCovEstimationOutput {
            ndt_covariance: [0.0; 36],
            publish_kind: 42,
            multi_ndt_result_poses: core::ptr::null_mut(),
            multi_initial_poses: core::ptr::null_mut(),
            pose_cap: 0,
            multi_ndt_result_count: 0,
            multi_initial_count: 0,
        };
        // SAFETY: a null engine must leave `out` untouched.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_estimate_pose_covariance(
                core::ptr::null_mut(),
                core::ptr::null(),
                &mut out,
            );
        }
        assert_eq!(out.publish_kind, 42);
    }

    // clone() is an independent deep copy: mutating the original's map after cloning must not affect
    // the clone's alignment.
    #[test]
    fn engine_clone_is_independent() {
        let tile_a = dense_cluster(0.5, 0.5, 0.5);
        let tile_b = dense_cluster(4.5, 0.5, 0.5);
        let source: Vec<[f32; 3]> = tile_a.iter().map(|p| [p[0] + 0.1, p[1], p[2]]).collect();
        let guess = Matrix4::<f32>::identity();

        let mut original = NdtEngine::new(1.0, 6, 0.01);
        configured(&mut original);
        original.add_target(&tile_a, 0);
        original.add_target(&tile_b, 1);
        original.create_kdtree();

        let mut scratch = MatchScratch::new();
        let clone = original.clone();
        clone.align_with(&guess, &source, &mut scratch);
        let clone_before = scratch.result().pose;

        // Mutate the original after the clone; the clone must be unaffected.
        original.remove_target(1);
        original.create_kdtree();
        original.align_with(&guess, &source, &mut scratch);

        clone.align_with(&guess, &source, &mut scratch);
        assert_eq!(
            scratch.result().pose,
            clone_before,
            "clone shares state with original"
        );
    }

    #[test]
    fn engine_remove_target_and_has_target() {
        let engine = NdtEngine::new(1.0, 6, 0.01);
        assert!(!engine.has_target());
        engine.add_target(&dense_cluster(0.5, 0.5, 0.5), 0);
        engine.add_target(&dense_cluster(4.5, 0.5, 0.5), 1);
        assert!(engine.has_target());
        engine.remove_target(0);
        engine.remove_target(1);
        assert!(!engine.has_target());
    }

    // FFI handle round-trip == the pure engine API (marshaling check).
    #[cfg(any(feature = "std", not(feature = "mt")))]
    #[test]
    fn ffi_engine_matches_pure() {
        let tile = dense_cluster(0.5, 0.5, 0.5);
        let flat: Vec<f32> = tile.iter().flatten().copied().collect();
        let source: Vec<[f32; 3]> = tile.iter().map(|p| [p[0] + 0.1, p[1], p[2]]).collect();
        let src_flat: Vec<f32> = source.iter().flatten().copied().collect();
        let guess16 = [
            1.0_f32, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];

        // pure
        let mut pure = NdtEngine::new(1.0, 6, 0.01);
        configured(&mut pure);
        pure.add_target(&tile, 0);
        pure.create_kdtree();
        pure.align(&matrix4_from_row_major(&guess16), &source);
        let want = pure.result().clone();
        let want_tp_score = pure.calc_transformation_probability(&source);
        let want_nvl_score = pure.calc_nearest_voxel_likelihood(&source);
        let want_maxit = pure.max_iterations();
        let mut want_scores = Vec::new();
        pure.nearest_voxel_score_each_point(&source, &mut want_scores);
        let (want_tp_arr, want_nvl_arr) = {
            let (a, b) = pure.score_arrays();
            (a.clone(), b.clone())
        };

        // FFI
        // SAFETY: every pointer below addresses its documented length.
        unsafe {
            let e = autoware_ndt_scan_matcher_rs_ndt_engine_new(1.0, 6, 0.01);
            autoware_ndt_scan_matcher_rs_ndt_engine_set_params(e, 0.01, 0.1, 1.0, 30, 0.55, 1);
            autoware_ndt_scan_matcher_rs_ndt_engine_add_target(e, flat.as_ptr(), tile.len(), 0);
            autoware_ndt_scan_matcher_rs_ndt_engine_create_kdtree(e);
            assert!(autoware_ndt_scan_matcher_rs_ndt_engine_has_target(e));
            autoware_ndt_scan_matcher_rs_ndt_engine_align(
                e,
                guess16.as_ptr(),
                src_flat.as_ptr(),
                source.len(),
            );

            let mut pose = [0.0_f32; 16];
            let mut iter = 0_i32;
            let mut tp = 0.0_f32;
            let mut nvl = 0.0_f32;
            let mut hessian = [0.0_f64; 36];
            let mut count = 0_u32;
            let out = AwNdtAlignOutput {
                pose: pose.as_mut_ptr(),
                iteration_num: &mut iter,
                transform_probability: &mut tp,
                nearest_voxel_likelihood: &mut nvl,
                hessian: hessian.as_mut_ptr(),
                transformation_array: core::ptr::null_mut(),
                transforms_cap: 0,
                transforms_count: &mut count,
            };
            autoware_ndt_scan_matcher_rs_ndt_engine_get_result(e, &out);

            assert_eq!(iter, want.iteration_num);
            assert_eq!(tp, want.transform_probability);
            assert_eq!(nvl, want.nearest_voxel_likelihood);
            for r in 0..4 {
                for c in 0..4 {
                    assert_eq!(pose[(r * 4) + c], want.pose[(r, c)]);
                }
            }

            // Remaining shims marshal to the same values as the pure API.
            assert_eq!(
                autoware_ndt_scan_matcher_rs_ndt_engine_max_iterations(e),
                want_maxit
            );
            assert_eq!(
                autoware_ndt_scan_matcher_rs_ndt_engine_calc_transformation_probability(
                    e,
                    src_flat.as_ptr(),
                    source.len()
                ),
                want_tp_score
            );
            assert_eq!(
                autoware_ndt_scan_matcher_rs_ndt_engine_calc_nearest_voxel_likelihood(
                    e,
                    src_flat.as_ptr(),
                    source.len()
                ),
                want_nvl_score
            );
            // set_regularization shim (no-op with scale 0); clone is an independent live handle.
            autoware_ndt_scan_matcher_rs_ndt_engine_set_regularization(e, 0.0, 0.0, 0.0);
            let e2 = autoware_ndt_scan_matcher_rs_ndt_engine_clone(e);
            assert!(autoware_ndt_scan_matcher_rs_ndt_engine_has_target(e2));
            autoware_ndt_scan_matcher_rs_ndt_engine_free(e2);
            // remove the only tile -> no target left.
            // per-point score + per-iteration score arrays marshal to the pure values.
            let mut ffi_scores = vec![0.0_f32; source.len()];
            autoware_ndt_scan_matcher_rs_ndt_engine_calc_nearest_voxel_score_each_point(
                e,
                src_flat.as_ptr(),
                source.len(),
                ffi_scores.as_mut_ptr(),
            );
            assert_eq!(ffi_scores, want_scores);

            let cap = (want_tp_arr.len() as u32) + 4;
            let mut tp_arr = vec![0.0_f32; cap as usize];
            let mut nvl_arr = vec![0.0_f32; cap as usize];
            let mut scount = 0_u32;
            autoware_ndt_scan_matcher_rs_ndt_engine_get_score_arrays(
                e,
                tp_arr.as_mut_ptr(),
                nvl_arr.as_mut_ptr(),
                cap,
                &mut scount,
            );
            assert_eq!(scount as usize, want_tp_arr.len());
            assert_eq!(&tp_arr[..scount as usize], want_tp_arr.as_slice());
            assert_eq!(&nvl_arr[..scount as usize], want_nvl_arr.as_slice());

            autoware_ndt_scan_matcher_rs_ndt_engine_remove_target(e, 0);
            assert!(!autoware_ndt_scan_matcher_rs_ndt_engine_has_target(e));
            autoware_ndt_scan_matcher_rs_ndt_engine_free(e);
        }
    }

    // --- engine-owned cell-id map ---

    fn ids_of(engine: &NdtEngine) -> Vec<Vec<u8>> {
        engine.load_state().id_map.keys().cloned().collect()
    }

    #[test]
    fn id_map_add_remove_and_sorted_ids() {
        let tile = dense_cluster(0.5, 0.5, 0.5);
        let mut engine = NdtEngine::new(1.0, 6, 0.01);
        configured(&mut engine);
        engine.add_target_bytes(&tile, b"beta");
        engine.add_target_bytes(&tile, b"alpha");
        // BTreeMap-sorted, deterministic.
        assert_eq!(ids_of(&engine), vec![b"alpha".to_vec(), b"beta".to_vec()]);
        assert!(engine.has_target());

        engine.remove_target_bytes(b"alpha");
        assert_eq!(ids_of(&engine), vec![b"beta".to_vec()]);
        // Removing an unknown id is a no-op.
        engine.remove_target_bytes(b"missing");
        assert_eq!(ids_of(&engine), vec![b"beta".to_vec()]);
    }

    #[test]
    fn id_map_reuses_u64_for_existing_cell_id() {
        let tile = dense_cluster(0.5, 0.5, 0.5);
        let mut engine = NdtEngine::new(1.0, 6, 0.01);
        configured(&mut engine);
        engine.add_target_bytes(&tile, b"0");
        engine.add_target_bytes(&tile, b"0"); // same cell-id → same tile, no new id
        assert_eq!(ids_of(&engine), vec![b"0".to_vec()]);
    }

    #[test]
    fn clone_carries_id_map() {
        let tile = dense_cluster(0.5, 0.5, 0.5);
        let mut engine = NdtEngine::new(1.0, 6, 0.01);
        configured(&mut engine);
        engine.add_target_bytes(&tile, b"0");
        engine.add_target_bytes(&tile, b"1");
        let clone = engine.clone();
        // Mutating the original must not affect the clone's id-map.
        engine.remove_target_bytes(b"0");
        assert_eq!(ids_of(&engine), vec![b"1".to_vec()]);
        assert_eq!(ids_of(&clone), vec![b"0".to_vec(), b"1".to_vec()]);
    }

    #[test]
    fn ffi_string_id_map_roundtrip() {
        let tile = dense_cluster(0.5, 0.5, 0.5);
        let flat: Vec<f32> = tile.iter().flat_map(|p| *p).collect();
        let mut engine = NdtEngine::new(1.0, 6, 0.01);
        configured(&mut engine);
        // SAFETY: valid engine; flat is 3*n f32; ids are valid byte slices.
        unsafe {
            autoware_ndt_scan_matcher_rs_ndt_engine_add_target_str(
                &engine,
                flat.as_ptr(),
                tile.len(),
                b"aa".as_ptr(),
                2,
            );
            autoware_ndt_scan_matcher_rs_ndt_engine_add_target_str(
                &engine,
                flat.as_ptr(),
                tile.len(),
                b"bbb".as_ptr(),
                3,
            );
        }
        // Pass 1: sizes.
        let mut count = 0u32;
        let mut total = 0u32;
        // SAFETY: count/total are writable; the buffers are null (size-only pass).
        unsafe {
            autoware_ndt_scan_matcher_rs_ndt_engine_get_current_map_ids(
                &engine,
                core::ptr::null_mut(),
                0,
                core::ptr::null_mut(),
                0,
                &mut count,
                &mut total,
            );
        }
        assert_eq!(count, 2);
        assert_eq!(total, 5); // "aa" + "bbb"

        // Pass 2: fill.
        let mut lengths = vec![0u32; count as usize];
        let mut bytes = vec![0u8; total as usize];
        // SAFETY: buffers sized to count/total per pass 1.
        unsafe {
            autoware_ndt_scan_matcher_rs_ndt_engine_get_current_map_ids(
                &engine,
                lengths.as_mut_ptr(),
                count,
                bytes.as_mut_ptr(),
                total,
                &mut count,
                &mut total,
            );
        }
        assert_eq!(lengths, vec![2, 3]); // sorted: "aa", "bbb"
        assert_eq!(&bytes[0..2], b"aa");
        assert_eq!(&bytes[2..5], b"bbb");

        // SAFETY: null engine is a no-op (count untouched).
        let mut c2 = 99u32;
        let mut t2 = 99u32;
        unsafe {
            autoware_ndt_scan_matcher_rs_ndt_engine_get_current_map_ids(
                core::ptr::null(),
                core::ptr::null_mut(),
                0,
                core::ptr::null_mut(),
                0,
                &mut c2,
                &mut t2,
            );
        }
        assert_eq!(c2, 99);
    }
}
