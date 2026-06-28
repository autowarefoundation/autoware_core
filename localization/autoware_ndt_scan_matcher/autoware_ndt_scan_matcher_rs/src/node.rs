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

//! Phase N0 — node callback logic in Rust (proof of pattern). The C++ rclcpp shell owns the node
//! state + ROS I/O and exposes them to Rust via a **host interface**: a `#[repr(C)]` vtable of C
//! function pointers + an opaque context (the `NDTScanMatcher*`). A migrated callback's body lives
//! here and drives the node purely through that vtable. ROS-node glue only (not the `no_std` engine
//! path) — this module is `std`-gated so the `no_std` rlib excludes it.

use core::ffi::c_void;

/// The ROS I/O / node-state operations a migrated Rust callback needs, as C function pointers over an
/// opaque context. Built and owned C++-side (the trampolines cast `ctx` back to `NDTScanMatcher*`);
/// every pointer + `ctx` must stay valid for the duration of the call. Field order must match the C
/// `AwNdtHost` struct.
#[repr(C)]
pub struct NdtHost {
    ctx: *mut c_void,
    /// Set the node's activation flag (`is_activated_`).
    set_activated: extern "C" fn(*mut c_void, bool),
    /// Clear the initial-pose interpolation buffer (`initial_pose_buffer_`).
    clear_initial_pose_buffer: extern "C" fn(*mut c_void),
}

/// Migrated body of `service_trigger_node`: set the activation flag, and on enable clear the
/// initial-pose buffer (so stale poses don't survive a re-activation). The C++ wrapper keeps the
/// diagnostics around this core.
///
/// # Safety
/// `host` is a valid [`NdtHost`] (or null → no-op) whose function pointers and `ctx` outlive the call.
#[expect(
    unsafe_code,
    reason = "C ABI host-interface boundary; validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_on_trigger(
    host: *const NdtHost,
    activate: bool,
) {
    if host.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid host whose ctx/fn-pointers are live.
    let h = unsafe { &*host };
    (h.set_activated)(h.ctx, activate);
    if activate {
        (h.clear_initial_pose_buffer)(h.ctx);
    }
}

#[cfg(test)]
#[allow(
    unsafe_code,
    clippy::borrow_as_ptr,
    clippy::allow_attributes,
    reason = "test code: invoking the C ABI entry with a mock host"
)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

    // The mock host records calls in process-global state, so the two trigger sequences below run in
    // ONE sequential test (parallel tests would race the flags).
    static ACTIVATED: AtomicBool = AtomicBool::new(false);
    static CLEARS: AtomicU32 = AtomicU32::new(0);

    extern "C" fn rec_set_activated(_ctx: *mut c_void, activate: bool) {
        ACTIVATED.store(activate, Ordering::SeqCst);
    }
    extern "C" fn rec_clear(_ctx: *mut c_void) {
        CLEARS.fetch_add(1, Ordering::SeqCst);
    }

    #[test]
    fn on_trigger_sets_flag_and_clears_only_on_activate() {
        let host = NdtHost {
            ctx: core::ptr::null_mut(),
            set_activated: rec_set_activated,
            clear_initial_pose_buffer: rec_clear,
        };

        // activate: flag set true, buffer cleared once.
        // SAFETY: `host` is a valid NdtHost living for the call.
        unsafe { autoware_ndt_scan_matcher_rs_node_on_trigger(&host, true) };
        assert!(ACTIVATED.load(Ordering::SeqCst));
        assert_eq!(CLEARS.load(Ordering::SeqCst), 1);

        // deactivate: flag set false, buffer NOT cleared again.
        // SAFETY: as above.
        unsafe { autoware_ndt_scan_matcher_rs_node_on_trigger(&host, false) };
        assert!(!ACTIVATED.load(Ordering::SeqCst));
        assert_eq!(CLEARS.load(Ordering::SeqCst), 1);

        // null host is a no-op (no panic).
        // SAFETY: passing null is explicitly handled.
        unsafe { autoware_ndt_scan_matcher_rs_node_on_trigger(core::ptr::null(), true) };
        assert_eq!(CLEARS.load(Ordering::SeqCst), 1);
    }
}
