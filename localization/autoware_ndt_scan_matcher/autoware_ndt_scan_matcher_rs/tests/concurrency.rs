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

//! Concurrency stress test for the engine refactor (slice 2): the `NdtEngine` is `Sync`, so the ROS
//! node shares one `&NdtEngine` across callbacks WITHOUT a giant lock. This reproduces that exact
//! contention — many reader threads aligning while a writer thread repeatedly publishes freshly-built
//! maps via `commit_from` (the map-update path) — and asserts every align observes a complete,
//! valid map (a finite pose), never a partially-built / kd-tree-less one. Run under
//! `ThreadSanitizer` for a data-race proof:
//!   RUSTFLAGS="-Zsanitizer=thread" cargo +nightly test --target x86_64-unknown-linux-gnu \
//!     --test concurrency
//! (a plain `cargo test` still exercises it and catches gross races / panics / aborts.)
//!
//! Also runs against the `mt` (multi-core `no_std`) lock backend on host:
//!   `cargo test --no-default-features --features mt,awkernel_sync/std --test concurrency`
//! The plain `no_std` single-core build (`--no-default-features` alone) keeps the engine `!Sync`
//! by design, so this test is gated to the two `Sync` configs.

#![cfg(any(feature = "std", feature = "mt"))]
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::as_conversions,
    clippy::cast_precision_loss,
    clippy::allow_attributes,
    reason = "test code"
)]

use std::sync::Arc;
use std::thread;

use autoware_ndt_scan_matcher_rs::engine::{MatchScratch, NdtEngine};
use nalgebra::Matrix4;

fn dense_cluster(cx: f32, cy: f32, cz: f32) -> Vec<[f32; 3]> {
    (0..8)
        .map(|i| {
            let f = i as f32 * 0.02;
            [cx + f, cy - f, cz + 0.5 * f]
        })
        .collect()
}

fn configured_two_tile_engine() -> (NdtEngine, Vec<[f32; 3]>) {
    let tile_a = dense_cluster(0.5, 0.5, 0.5);
    let tile_b = dense_cluster(4.5, 0.5, 0.5);
    let source: Vec<[f32; 3]> = tile_a
        .iter()
        .chain(tile_b.iter())
        .map(|p| [p[0] + 0.1, p[1] - 0.05, p[2]])
        .collect();
    let engine = NdtEngine::new(1.0, 6, 0.01);
    engine.set_params(0.01, 0.1, 1.0, 30, 0.55, 1);
    engine.add_target(&tile_a, 0);
    engine.add_target(&tile_b, 1);
    engine.create_kdtree();
    (engine, source)
}

// 3 readers align continuously while 1 writer commits a freshly-built map each iteration. With the
// giant lock gone, the only synchronization is the engine's internal ArcSwap; a reader must always
// load a complete published map (every align yields a finite pose), and nothing races / aborts.
#[test]
fn concurrent_aligns_and_map_commits_stay_consistent() {
    const READERS: usize = 3;
    const READER_ITERS: usize = 4000;
    const WRITER_ITERS: usize = 2000;

    let (engine, source) = configured_two_tile_engine();
    let engine = Arc::new(engine);
    let guess = Matrix4::<f32>::identity();

    let mut handles = Vec::new();

    for _ in 0..READERS {
        let e = Arc::clone(&engine);
        let src = source.clone();
        handles.push(thread::spawn(move || {
            // One caller-owned scratch per reader thread (the `mt` usage model; equivalent to the
            // std thread-local).
            let mut scratch = MatchScratch::new();
            for _ in 0..READER_ITERS {
                e.align_with(&guess, &src, &mut scratch);
                let r = scratch.result();
                // A complete map always yields a finite pose; a torn/kd-tree-less map would not.
                assert!(r.pose[(0, 3)].is_finite(), "align saw an inconsistent map");
                assert!(e.has_target(), "map vanished mid-run");
            }
        }));
    }

    {
        let e = Arc::clone(&engine);
        let extra = dense_cluster(8.5, 0.5, 0.5);
        handles.push(thread::spawn(move || {
            for i in 0..WRITER_ITERS {
                // Build the next map on a PRIVATE staging engine (deep clone), then publish it with a
                // single atomic commit — exactly the map-update module's ON path.
                let staging = (*e).clone();
                if i % 2 == 0 {
                    staging.add_target(&extra, 2);
                } else {
                    staging.remove_target(2);
                }
                staging.create_kdtree();
                e.commit_from(&staging);
            }
        }));
    }

    for h in handles {
        h.join()
            .expect("worker thread panicked (likely a data race or torn map)");
    }

    assert!(engine.has_target());
}
