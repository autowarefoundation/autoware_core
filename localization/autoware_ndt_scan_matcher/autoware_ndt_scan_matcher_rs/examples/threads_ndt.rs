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

//! NDT scan matcher driven by **threads + a primitive poll loop, with NO async runtime** — modeling a
//! bare-metal / no_std kernel that schedules concurrent tasks itself.
//!
//! Caveat: `examples/` are std programs (`cargo run`), and `std::thread` is std — a *literal* no_std
//! bare-metal binary needs its own `#![no_std] #![no_main]` target/crate, not `examples/`. So here
//! `std::thread` **stands in for the kernel's tasks**, and a hand-rolled `block_on` (just polls a
//! future to completion with a no-op waker — exactly what a no_std executor does) drives the **async**
//! `MapSource` port **without Tokio**. It also exercises the `Sync`, lock-free `ScanMatcher` under real
//! concurrent threads: one map-update thread publishing fresh maps while align workers run.
//!
//! Run: `cargo run --example threads_ndt`

#![allow(
    unsafe_code,
    clippy::as_conversions,
    clippy::cast_precision_loss,
    clippy::indexing_slicing,
    clippy::arithmetic_side_effects,
    clippy::float_cmp,
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::allow_attributes,
    reason = "example code: a minimal no-runtime block_on (core::task) + synthetic data + threads"
)]

use std::sync::Arc;

use autoware_ndt_scan_matcher_rs::host::{
    Clock, MapDelta, MapSource, MapTile, MatchResult, OutputSink,
};
use autoware_ndt_scan_matcher_rs::scan_matcher::ScanMatcher;
use nalgebra::Matrix4;

/// Poll a future to completion with a no-op waker — the minimal executor a no_std scheduler provides.
/// Suitable for self-contained futures (no external I/O wait); our `MapSource::load` is immediately
/// ready, so this returns after the first poll.
fn block_on<F: core::future::Future>(future: F) -> F::Output {
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    fn noop(_: *const ()) {}
    fn clone(_: *const ()) -> RawWaker {
        RawWaker::new(core::ptr::null(), &VTABLE)
    }
    static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);

    // SAFETY: the vtable's clone/wake/drop are all no-ops over a null data pointer — trivially sound.
    let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
    let mut cx = Context::from_waker(&waker);
    let mut future = future;
    // SAFETY: `future` is owned here and never moved again before being dropped.
    let mut future = unsafe { core::pin::Pin::new_unchecked(&mut future) };
    loop {
        match future.as_mut().poll(&mut cx) {
            Poll::Ready(value) => return value,
            Poll::Pending => core::hint::spin_loop(),
        }
    }
}

fn dense_tile(cx: f32, cy: f32, cz: f32, id: &[u8]) -> MapTile {
    let mut points = Vec::new();
    for k in 0..60 {
        let f = k as f32;
        points.push([
            cx + (f * 0.7).sin() * 0.3,
            cy + (f * 1.1).cos() * 0.3,
            cz + (f * 0.3).sin() * 0.15,
        ]);
    }
    MapTile {
        id: id.to_vec(),
        points,
    }
}

/// Synthetic host. `extra` toggles whether a third tile is present, so the map-update thread can
/// publish genuinely different maps each round.
struct ThreadHost {
    extra: bool,
}

impl MapSource for ThreadHost {
    async fn load(&self, _center: [f64; 2], _radius: f64) -> MapDelta {
        let mut add = vec![
            dense_tile(0.0, 0.0, 0.0, b"0"),
            dense_tile(8.0, 4.0, 0.0, b"1"),
        ];
        let mut remove = Vec::new();
        if self.extra {
            add.push(dense_tile(-6.0, 2.0, 0.0, b"2"));
        } else {
            remove.push(b"2".to_vec());
        }
        MapDelta { add, remove }
    }
}

impl OutputSink for ThreadHost {
    fn publish_result(&self, _r: &MatchResult) {}
}

impl Clock for ThreadHost {
    fn now_ns(&self) -> i64 {
        0
    }
}

fn synthetic_scan() -> Vec<[f32; 3]> {
    let offset = [0.2_f32, -0.15, 0.1];
    let mut scan = Vec::new();
    for tile in [
        dense_tile(0.0, 0.0, 0.0, b"0"),
        dense_tile(8.0, 4.0, 0.0, b"1"),
    ] {
        for p in tile.points {
            scan.push([p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]]);
        }
    }
    scan
}

fn main() {
    const ALIGN_WORKERS: usize = 3;
    const ALIGNS_PER_WORKER: usize = 2000;
    const MAP_UPDATES: usize = 1000;

    let matcher = Arc::new(ScanMatcher::new(2.0, 6, 0.01));
    matcher.set_params(0.01, 0.1, 2.0, 30, 0.55, 1);
    // Gate convergence on the transform-probability score (type 0); 0.0 is the defensive threshold.
    matcher.set_convergence_params(0, 0.0, 0.0);
    // Initial map (drive the async port with the hand-rolled block_on — no runtime).
    block_on(matcher.update_map(&ThreadHost { extra: true }, [0.0, 0.0], 50.0));
    assert!(matcher.has_target(), "map should be loaded");

    let mut handles = Vec::new();

    // Map-update thread: publish fresh maps continuously (std::thread = a kernel task stand-in).
    {
        let matcher = Arc::clone(&matcher);
        handles.push(std::thread::spawn(move || {
            for i in 0..MAP_UPDATES {
                let host = ThreadHost { extra: i % 2 == 0 };
                block_on(matcher.update_map(&host, [0.0, 0.0], 50.0));
            }
        }));
    }

    // Align workers: match the synthetic scan against whatever map is currently published.
    let scan = synthetic_scan();
    let guess = Matrix4::<f32>::identity();
    for _ in 0..ALIGN_WORKERS {
        let matcher = Arc::clone(&matcher);
        let scan = scan.clone();
        handles.push(std::thread::spawn(move || {
            for _ in 0..ALIGNS_PER_WORKER {
                let r = matcher.match_scan(&guess, &scan);
                assert!(r.pose[(0, 3)].is_finite(), "align saw an inconsistent map");
            }
        }));
    }

    for h in handles {
        h.join()
            .expect("a worker thread panicked (data race / torn map?)");
    }

    // Final deterministic check on the quiesced matcher.
    let r = matcher.match_scan(&guess, &scan);
    assert!(matcher.has_target() && r.pose[(0, 3)].is_finite());
    assert!(
        r.converged,
        "the quiesced synthetic match should report converged"
    );
    println!(
        "OK: {ALIGN_WORKERS} threads x {ALIGNS_PER_WORKER} aligns concurrent with {MAP_UPDATES} map \
         updates, no async runtime (hand-rolled block_on). final translation=({:.4}, {:.4}, {:.4})  \
         converged={}  oscillation={}",
        r.pose[(0, 3)],
        r.pose[(1, 3)],
        r.pose[(2, 3)],
        r.converged,
        r.oscillation_num,
    );
}
