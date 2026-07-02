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

//! The ROS **side-effects host vtable** (`AwHost`) — the C-ABI adapter that realizes the portable
//! [`crate::host`] `Host` seam for the rclcpp node. A migrated callback body drives ROS side effects
//! (clock, logging, TF lookup, and — added as later sub-slices land — publishing) through this vtable
//! of C function pointers over an opaque `ctx` (the `NDTScanMatcher *`), instead of touching rclcpp
//! directly. Side-effects only; no node state (that lives Rust-side on the handle). Field order must
//! match the C `AwHost` struct.
//!
//! This is deliberately minimal for Phase 5 sub-slice 1 (the sensor-callback prologue needs only
//! `now_ns` / `log` / `lookup_transform`); publish ops are added with the sub-slices that move the
//! align + publish tail.

use core::ffi::c_void;

/// A borrowed UTF-8 (or raw) byte string crossing the C ABI: `(ptr, len)`, not NUL-terminated.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct AwStr {
    pub ptr: *const u8,
    pub len: usize,
}

impl AwStr {
    /// Borrow `self` as a byte slice for the duration of the call (null/zero-length → empty).
    ///
    /// # Safety
    /// When `len > 0`, `ptr` must point to `len` readable bytes that outlive the borrow.
    #[must_use]
    #[expect(
        unsafe_code,
        reason = "C ABI boundary; pointer validated per rust-c-ffi-safety"
    )]
    pub unsafe fn as_bytes<'a>(self) -> &'a [u8] {
        if self.ptr.is_null() || self.len == 0 {
            return &[];
        }
        // SAFETY: non-null with len > 0 per the check; caller guarantees `len` readable bytes.
        unsafe { core::slice::from_raw_parts(self.ptr, self.len) }
    }
}

/// Log severity byte for [`AwHost::log`] (mirrors the ROS logger levels used by the node).
pub const LOG_WARN: i32 = 1;
pub const LOG_ERROR: i32 = 2;

/// The ROS side-effects vtable. Built + owned C++-side (`make_host`); the trampolines cast `ctx` back
/// to `NDTScanMatcher *`. Every fn-pointer + `ctx` must stay valid for the duration of the call.
#[repr(C)]
pub struct AwHost {
    ctx: *mut c_void,
    /// `this->now().nanoseconds()`.
    now_ns: extern "C" fn(*mut c_void) -> i64,
    /// Emit a ROS log line at `level` (`LOG_WARN`/`LOG_ERROR`); `msg` is `(ptr, len)` UTF-8.
    log: extern "C" fn(*mut c_void, level: i32, msg: *const u8, msg_len: usize),
    /// Look up the `target←source` transform and write it as a row-major 4x4 `f32` into
    /// `out_matrix4x4_row_major` (16 floats). Returns `false` on a TF failure (out left untouched).
    lookup_transform:
        extern "C" fn(*mut c_void, target: AwStr, source: AwStr, out_matrix4x4_row_major: *mut f32)
            -> bool,
}

impl AwHost {
    /// The host clock in nanoseconds.
    #[must_use]
    pub fn now_ns(&self) -> i64 {
        (self.now_ns)(self.ctx)
    }

    /// Emit a ROS log line.
    pub fn log(&self, level: i32, msg: &str) {
        (self.log)(self.ctx, level, msg.as_ptr(), msg.len());
    }

    /// Look up the `target←source` transform as a row-major 4x4. `None` on TF failure.
    #[must_use]
    pub fn lookup_transform(&self, target: &[u8], source: &[u8]) -> Option<[f32; 16]> {
        let mut out = [0.0_f32; 16];
        let ok = (self.lookup_transform)(
            self.ctx,
            AwStr {
                ptr: target.as_ptr(),
                len: target.len(),
            },
            AwStr {
                ptr: source.as_ptr(),
                len: source.len(),
            },
            out.as_mut_ptr(),
        );
        ok.then_some(out)
    }
}
