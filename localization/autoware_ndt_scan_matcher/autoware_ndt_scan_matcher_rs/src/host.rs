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

//! Portable host-interface ports for the NDT scan-matcher node logic.
//!
//! The orchestration in [`crate::scan_matcher`] is written **generically over `H: Host`** so the same
//! `no_std` Rust runs on any environment, which only differs *below* these ports:
//! - the ROS node implements them via rclcpp (a Rust `FfiHost` adapter over the C-ABI vtables);
//! - a bare-metal / `no_std` async kernel implements them with its own async runtime + map source;
//! - `examples/tokio_ndt.rs` implements them with Tokio + synthetic data (the async reference).
//!
//! The **I/O ports are `async`** (map loading is inherently async — a ROS service future, a kernel DMA
//! read, …) declared with return-position `impl Future` so no boxing/`Send` bound is forced (clean in
//! `no_std`, static dispatch). The engine **align hot path stays synchronous** (WCET) — it is called
//! between awaits, never `async`.

use alloc::vec::Vec;

use nalgebra::Matrix4;

/// A map tile to add: its cell id (raw bytes — the PCD `cell_id`, not assumed UTF-8) + its points.
pub struct MapTile {
    pub id: Vec<u8>,
    pub points: Vec<[f32; 3]>,
}

/// A differential map update around a position (mirrors the ROS `pcd_loader` add/remove delta).
pub struct MapDelta {
    pub add: Vec<MapTile>,
    /// Cell ids to remove.
    pub remove: Vec<Vec<u8>>,
}

/// The result of matching one scan (the subset the node publishes / acts on).
#[derive(Clone, Debug)]
pub struct MatchResult {
    /// Estimated `map→base_link` pose (4x4).
    pub pose: Matrix4<f32>,
    pub transform_probability: f32,
    pub nearest_voxel_likelihood: f32,
    pub iteration_num: i32,
    /// The convergence verdict (`is_converged`) the node gates on — see
    /// [`crate::convergence::evaluate_convergence`].
    pub converged: bool,
    /// Consecutive direction-inversion count over the iteration trajectory (the C++
    /// `count_oscillation`); `> 10` lets a local-minimum stop still count as converged.
    pub oscillation_num: i32,
}

/// The estimated pose covariance (the row-major 6x6 the node publishes). Produced alongside a
/// [`MatchResult`] by `ScanMatcher::match_scan_with_covariance`.
#[derive(Clone, Debug)]
pub struct CovarianceResult {
    /// Row-major 6x6 pose covariance (`map` frame), after the configured estimation mode + scaling.
    pub covariance: [f64; 36],
}

/// Source of map tiles around a position. ROS: the `pcd_loader` service; kernel: flash/DMA; example:
/// synthetic. Async because the load is inherently I/O.
pub trait MapSource {
    /// Load the differential map update for the area centered at `center` (x, y) with `radius`.
    fn load(&self, center: [f64; 2], radius: f64) -> impl core::future::Future<Output = MapDelta>;
}

/// Sink for a match result. ROS: publishers; kernel/example: their own output.
pub trait OutputSink {
    fn publish_result(&self, result: &MatchResult);
}

/// Wall/sim clock (nanoseconds). ROS: `node->now()`; example: a counter.
pub trait Clock {
    fn now_ns(&self) -> i64;
}

/// The full host interface a migrated node callback drives. Blanket-implemented for any type that
/// provides all the ports, so node logic takes `host: &H` with `H: Host`.
pub trait Host: MapSource + OutputSink + Clock {}
impl<T: MapSource + OutputSink + Clock> Host for T {}
