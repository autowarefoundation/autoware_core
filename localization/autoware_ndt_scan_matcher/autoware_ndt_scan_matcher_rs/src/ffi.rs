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

//! Panic-safe FFI boundary helpers + the status/error types the object-level entry points return.
//!
//! A Rust panic that unwinds into C++ is undefined behavior. The crate already *prevents* panics via
//! the rust-hardening lint gates (no `unwrap`/`expect`/`panic`/indexing in production), but a
//! defective dependency or an `overflow-checks` integer overflow could still panic. These helpers
//! *contain* it: a caught unwind becomes [`AwStatus::Panic`] / a null pointer instead of crossing the
//! boundary.
//!
//! std-only: `catch_unwind` needs `std`, and these wrap the ROS-node C ABI — the `no_std` kernel
//! build drives the engine directly and crosses no C ABI here.

use std::panic::{AssertUnwindSafe, catch_unwind};

/// Status code returned across the C ABI by the object-level (`NdtScanMatcherRs`) FFI entry points.
/// `0` is success; each non-zero value is a specific failure the C++ shell logs. `#[repr(i32)]` fixes
/// the ABI (a plain C `int`).
#[repr(i32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AwStatus {
    /// The call succeeded.
    Ok = 0,
    /// A required pointer argument was null.
    NullPtr = 1,
    /// An argument was structurally invalid (e.g. mismatched offset-model lengths).
    InvalidParam = 2,
    /// The Rust side panicked; the unwind was caught at the boundary (no UB), the call did nothing.
    Panic = 3,
}

/// A recoverable failure inside an FFI entry point, mapped to an [`AwStatus`] at the boundary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    /// A required pointer argument was null.
    NullPtr,
    /// A validated argument was structurally invalid.
    InvalidParam,
}

impl Error {
    /// The [`AwStatus`] this error maps to at the C ABI boundary.
    #[must_use]
    pub fn into_status(self) -> AwStatus {
        match self {
            Error::NullPtr => AwStatus::NullPtr,
            Error::InvalidParam => AwStatus::InvalidParam,
        }
    }
}

/// Run `f` at an FFI boundary, mapping its result to an [`AwStatus`] and containing any panic.
///
/// `AssertUnwindSafe` is sound here: on an unwind we return [`AwStatus::Panic`] and touch no shared
/// state through a possibly-broken invariant — `f` owns whatever it mutates, and nothing observable
/// escapes the caught panic.
#[must_use]
pub fn ffi_boundary<F>(f: F) -> AwStatus
where
    F: FnOnce() -> Result<(), Error>,
{
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(Ok(())) => AwStatus::Ok,
        Ok(Err(e)) => e.into_status(),
        Err(_) => AwStatus::Panic,
    }
}

/// Run `f` at an FFI boundary that produces an owning pointer; an error or panic yields a null
/// pointer (the C++ RAII wrapper treats null as construction failure). See [`ffi_boundary`] for why
/// `AssertUnwindSafe` is sound.
#[must_use]
pub fn ffi_boundary_ptr<T, F>(f: F) -> *mut T
where
    F: FnOnce() -> Result<*mut T, Error>,
{
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(Ok(ptr)) => ptr,
        _ => core::ptr::null_mut(),
    }
}

#[cfg(test)]
#[allow(
    clippy::panic,
    clippy::allow_attributes,
    reason = "test code may freely panic to exercise the boundary"
)]
mod tests {
    use super::*;

    #[test]
    fn boundary_maps_ok_err_and_panic() {
        assert_eq!(ffi_boundary(|| Ok(())), AwStatus::Ok);
        assert_eq!(ffi_boundary(|| Err(Error::NullPtr)), AwStatus::NullPtr);
        assert_eq!(
            ffi_boundary(|| Err(Error::InvalidParam)),
            AwStatus::InvalidParam
        );
        assert_eq!(ffi_boundary(|| panic!("boom")), AwStatus::Panic);
    }

    #[test]
    fn boundary_ptr_returns_value_or_null() {
        let mut value: i32 = 7;
        let p = ffi_boundary_ptr::<i32, _>(|| Ok(&raw mut value));
        assert!(!p.is_null());

        let n = ffi_boundary_ptr::<i32, _>(|| Err(Error::NullPtr));
        assert!(n.is_null());

        let panicked = ffi_boundary_ptr::<i32, _>(|| panic!("boom"));
        assert!(panicked.is_null());
    }

    #[test]
    fn error_into_status_matches() {
        assert_eq!(Error::NullPtr.into_status(), AwStatus::NullPtr);
        assert_eq!(Error::InvalidParam.into_status(), AwStatus::InvalidParam);
    }
}
