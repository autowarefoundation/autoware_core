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

//! ROS map-update glue: drive the portable [`apply_map_update`] orchestration (the `Host`
//! [`MapSource`] port) from the C++ node, so the staging-engine + `commit_from` double-buffer lives in
//! Rust and the C++ side only supplies the tiles.
//!
//! The C++ map-update timer calls [`autoware_ndt_scan_matcher_rs_ndt_engine_update_map`] with an
//! [`AwMapSource`] vtable; `FfiHost` implements [`MapSource`] by calling back into C++ (`fill`), which
//! runs the pcd-loader service (already synchronous on the C++ side) and **pushes** the resulting tiles
//! into a Rust-owned `MapDelta` via [`autoware_ndt_scan_matcher_rs_map_delta_add`] /
//! `..._map_delta_remove`. The `async` `load` is therefore ready after one poll, so the FFI drives it
//! with a trivial noop-waker `block_on` — "callbacks `block_on` the async node fns" with no real suspend.

use core::ffi::c_void;
use core::future::Future;

use alloc::vec::Vec;

use crate::engine::NdtEngine;
use crate::ffi_ptr::{self, ffi_mut, ffi_ref, ffi_slice};
use crate::host::{MapDelta, MapSource, MapTile};
use crate::scan_matcher::apply_map_update;

/// Poll a ready future to completion with a noop waker — the C++ `fill` callback resolves the map load
/// synchronously, so the future never actually suspends (one poll). Mirrors the bare-metal executor in
/// `examples/threads_ndt.rs`.
#[expect(
    unsafe_code,
    reason = "noop RawWaker over a null data pointer (all ops no-ops) + Pin of an owned, never-moved future — trivially sound"
)]
fn block_on<F: Future>(future: F) -> F::Output {
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

/// C-ABI map source: an opaque C++ context + a `fill` callback that, for the requested area, runs the
/// pcd-loader and pushes the add-tiles + remove-ids into the Rust `MapDelta` builder (an opaque
/// `*mut MapDelta`) via [`autoware_ndt_scan_matcher_rs_map_delta_add`] /
/// `autoware_ndt_scan_matcher_rs_map_delta_remove`. Field order must match the C `AwMapSource` struct.
#[repr(C)]
pub struct AwMapSource {
    /// Opaque C++ context (the map-update module); passed back to `fill`.
    pub ctx: *mut c_void,
    /// Build the delta for the disc at `(cx, cy)` radius `radius` by pushing into `builder`.
    pub fill: extern "C" fn(ctx: *mut c_void, cx: f64, cy: f64, radius: f64, builder: *mut c_void),
}

/// [`MapSource`] over an [`AwMapSource`] vtable. `load` runs `fill` synchronously (the C++ pcd wait is
/// itself synchronous) and yields the populated delta — a ready future.
struct FfiHost<'a> {
    src: &'a AwMapSource,
}

impl MapSource for FfiHost<'_> {
    fn load(&self, center: [f64; 2], radius: f64) -> impl Future<Output = MapDelta> {
        let src = self.src;
        async move {
            let mut delta = MapDelta {
                add: Vec::new(),
                remove: Vec::new(),
            };
            // The builder is an opaque `*mut MapDelta`; `fill` pushes into it via the add/remove FFIs.
            let builder = core::ptr::from_mut(&mut delta).cast::<c_void>();
            (src.fill)(src.ctx, center[0], center[1], radius, builder);
            delta
        }
    }
}

/// Push one add-tile (cell-id bytes + `n_pts` interleaved `xyz` `f32`) into the delta builder. No-op if
/// `builder` is null. A null/empty id or cloud yields an empty `Vec` (a degenerate but harmless tile).
///
/// # Safety
/// `builder` must be the `*mut MapDelta` passed to `AwMapSource::fill` (or null → no-op). When
/// `id_len > 0`, `id_ptr` must point to `id_len` readable bytes; when `n_pts > 0`, `pts_ptr` must point
/// to `3 * n_pts` readable, aligned `f32`. The pointers are only read during the call.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads caller-owned id/cloud + the Rust-owned builder, validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_map_delta_add(
    builder: *mut c_void,
    id_ptr: *const u8,
    id_len: usize,
    pts_ptr: *const f32,
    n_pts: usize,
) {
    let delta = ffi_mut!(builder.cast::<MapDelta>(), else return);
    // SAFETY: caller guarantees the id/cloud lengths (or null/0 → empty); audited in ffi_ptr.
    let id = unsafe { ffi_ptr::slice_or_empty(id_ptr, id_len) }.to_vec();
    let points = unsafe { ffi_ptr::slice_or_empty(pts_ptr.cast::<[f32; 3]>(), n_pts) }.to_vec();
    delta.add.push(MapTile { id, points });
}

/// Push one remove-id (cell-id bytes) into the delta builder. No-op if `builder`/`id_ptr` is null or
/// `id_len` is 0.
///
/// # Safety
/// `builder` must be the `*mut MapDelta` passed to `AwMapSource::fill`; when `id_len > 0`, `id_ptr` must
/// point to `id_len` readable bytes. Both are only read during the call.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads caller-owned id + the Rust-owned builder, validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_map_delta_remove(
    builder: *mut c_void,
    id_ptr: *const u8,
    id_len: usize,
) {
    let delta = ffi_mut!(builder.cast::<MapDelta>(), else return);
    if id_len == 0 {
        return;
    }
    let id = ffi_slice!(id_ptr, id_len, else return).to_vec();
    delta.remove.push(id);
}

/// Load the map around `(cx, cy)` (radius `radius`) from `source` and publish it into `engine` via the
/// portable [`apply_map_update`] (staging engine + `commit_from`). `rebuild != 0` starts staging empty
/// (drops the current tiles — the C++ `need_rebuild`); otherwise it is incremental. No-op if any pointer
/// is null.
///
/// # Safety
/// `engine` is a valid, live `NdtEngine` handle (or null → no-op); it is reborrowed as `&NdtEngine` (the
/// engine is `Sync`). `source` is a valid [`AwMapSource`] whose `fill` + `ctx` outlive the call (or null
/// → no-op). Rust does not retain either pointer past the call.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reborrows the live engine + map-source vtable, validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_update_map(
    engine: *const NdtEngine,
    source: *const AwMapSource,
    cx: f64,
    cy: f64,
    radius: f64,
    rebuild: bool,
) {
    let eng = ffi_ref!(engine, else return);
    let src = ffi_ref!(source, else return);
    let host = FfiHost { src };
    block_on(apply_map_update(eng, &host, [cx, cy], radius, rebuild));
}
