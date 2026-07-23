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

//! Audited C-ABI pointer helpers + guard macros — the *one* place raw-pointer dereferences live.
//!
//! Every `#[unsafe(no_mangle)] extern "C"` entry point in this crate receives raw pointers from
//! C++. The recurring shape is "check a condition (almost always null), then dereference". Instead
//! of hand-writing that `unsafe` deref (and re-deriving its `// SAFETY:` justification) at ~180
//! call sites, the raw operation is centralized here into a handful of `unsafe fn`s, each audited
//! **once** against the master contract below. Call sites then read as a single, uniform line —
//! but `unsafe` stays lexically present (the guard macros expand to an `unsafe {}` block, and every
//! boundary fn keeps its `#[expect(unsafe_code, reason = …)]` anchor), so this centralizes the
//! *reasoning* about unsafety without hiding it (per the `rust-c-ffi-safety` rules).
//!
//! `core` + `alloc` only (no `std`): the FFI entry points are not all std-gated — many carry
//! `#[cfg(any(feature = "std", not(feature = "mt")))]` and so exist in the `no_std` single-core
//! build. `alloc::boxed::Box` is available in `no_std`, so the handle-lifecycle helpers work there
//! too. (The std-only, panic-catching boundary lives in [`crate::ffi`] instead.)
//!
//! # Safety — master contract for the borrow helpers
//! For [`opt_ref`], [`opt_mut`], [`opt_slice`], [`slice_or_empty`], [`opt_slice_mut`],
//! [`read_copy`]: a null pointer is always accepted (→ `None` / empty / `false`). A **non-null**
//! pointer must, for the returned reference's lifetime `'a`:
//! - point to a single value / an array of `len` values that is **valid, properly aligned, and
//!   fully initialized** for the referent type (for the write helpers, only *allocated + aligned* —
//!   the caller writes, so the target may be uninitialized);
//! - **outlive** `'a` (the caller must not retain the returned reference past the C call), and
//! - not be **aliased** in a conflicting way for `'a` (no other live `&mut` to the same place; for
//!   the mutable helpers, no overlapping mutable regions — e.g. distinct output buffers).
//!
//! A reinterpreting `.cast::<U>()` at the call site (e.g. `*const f32` → `*const [f32; 3]`) is a
//! *safe* pointer operation and stays at the call site; it must never **increase** alignment
//! (`align_of::<U>() <= align_of::<original>()`), which holds for the element-regrouping casts used
//! here (`align_of::<[f32; 3]>() == align_of::<f32>()`).

use alloc::boxed::Box;

/// Borrow `*const T` as `&'a T`; null → `None`. (Shapes: shared-handle / input-struct deref.)
///
/// # Safety
/// See the [module master contract](self). Thin, documented wrapper over `<*const T>::as_ref`.
#[inline]
#[must_use]
#[expect(
    unsafe_code,
    reason = "the crate's single audited shared-pointer dereference"
)]
pub unsafe fn opt_ref<'a, T>(p: *const T) -> Option<&'a T> {
    // SAFETY: caller upholds the module master contract; null is handled by `as_ref`.
    unsafe { p.as_ref() }
}

/// Borrow `*mut T` as `&'a mut T`; null → `None`. (Shape: mutable-handle deref.)
///
/// # Safety
/// See the [module master contract](self); additionally the caller must guarantee **unique**
/// access for `'a` (no other live reference to the same place).
#[inline]
#[must_use]
#[expect(
    unsafe_code,
    reason = "the crate's single audited mutable-pointer dereference"
)]
pub unsafe fn opt_mut<'a, T>(p: *mut T) -> Option<&'a mut T> {
    // SAFETY: caller upholds the master contract + unique access; null is handled by `as_mut`.
    unsafe { p.as_mut() }
}

/// Read a `Copy` value out of `*const T` by value; null → `None`. (Shape: fixed-array read, e.g.
/// `read_copy(leaf_size.cast::<[f64; 3]>())`.)
///
/// # Safety
/// See the [module master contract](self). If non-null, `p` must be readable for one aligned `T`.
#[inline]
#[must_use]
#[expect(
    unsafe_code,
    reason = "the crate's single audited by-value pointer read"
)]
pub unsafe fn read_copy<T: Copy>(p: *const T) -> Option<T> {
    if p.is_null() {
        return None;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, aligned, initialized `T`.
    Some(unsafe { *p })
}

/// Build `&'a [T]` from `p`/`len`; null → `None`. (Shape: input cloud / pose buffer.)
///
/// # Safety
/// See the [module master contract](self). If non-null, `p` must address `len` valid, aligned,
/// initialized `T` (an empty slice for `len == 0`).
#[inline]
#[must_use]
#[expect(
    unsafe_code,
    reason = "the crate's single audited shared slice construction"
)]
pub unsafe fn opt_slice<'a, T>(p: *const T, len: usize) -> Option<&'a [T]> {
    if p.is_null() {
        return None;
    }
    // SAFETY: non-null per the check; caller guarantees `len` valid, aligned, initialized `T`.
    Some(unsafe { core::slice::from_raw_parts(p, len) })
}

/// Like [`opt_slice`] but a null pointer **or** `len == 0` yields an empty slice (never `None`).
/// For genuinely optional/emptyable inputs such as the cell-id byte buffer.
///
/// # Safety
/// See the [module master contract](self). If non-null and `len > 0`, `p` must address `len` valid,
/// aligned, initialized `T`.
#[inline]
#[must_use]
#[expect(
    unsafe_code,
    reason = "the crate's single audited optional-empty slice construction"
)]
pub unsafe fn slice_or_empty<'a, T>(p: *const T, len: usize) -> &'a [T] {
    if p.is_null() || len == 0 {
        return &[];
    }
    // SAFETY: non-null with len > 0 per the check; caller guarantees `len` readable `T`.
    unsafe { core::slice::from_raw_parts(p, len) }
}

/// Build `&'a mut [T]` from `p`/`len`; null → `None`. (Shape: caller-owned output buffer.)
///
/// # Safety
/// See the [module master contract](self); additionally the caller must guarantee **unique,
/// non-overlapping** access to `len` aligned, writable `T` for `'a` (the slots may be
/// uninitialized — the caller writes them).
#[inline]
#[must_use]
#[expect(
    unsafe_code,
    reason = "the crate's single audited mutable slice construction"
)]
pub unsafe fn opt_slice_mut<'a, T>(p: *mut T, len: usize) -> Option<&'a mut [T]> {
    if p.is_null() {
        return None;
    }
    // SAFETY: non-null per the check; caller guarantees `len` unique, aligned, writable `T`.
    Some(unsafe { core::slice::from_raw_parts_mut(p, len) })
}

/// Write `value` through `p` once; returns `false` without writing if `p` is null. Uses
/// [`core::ptr::write`] semantics (does **not** drop any prior value), so it is correct for
/// possibly-uninitialized out-params. Intended for `#[repr(C)]` POD/`Copy` out-structs.
///
/// # Safety
/// If non-null, `p` must be aligned and writable for one `T`.
#[inline]
#[expect(unsafe_code, reason = "the crate's single audited out-pointer write")]
pub unsafe fn write_out<T>(p: *mut T, value: T) -> bool {
    if p.is_null() {
        return false;
    }
    // SAFETY: non-null per the check; caller guarantees an aligned, writable slot for `T`.
    unsafe { p.write(value) };
    true
}

/// Move `value` onto the heap and hand C an owning raw handle (free with [`free_handle`]). Safe:
/// no dereference occurs. (Shape: `..._new`.)
#[inline]
#[must_use]
pub fn into_handle<T>(value: T) -> *mut T {
    Box::into_raw(Box::new(value))
}

/// Reclaim and drop an owning handle; null → no-op. (Shape: `..._free`.)
///
/// # Safety
/// `p` is null, or a handle produced by [`into_handle`] / `Box::into_raw` that is freed **exactly
/// once** and not used afterwards.
#[inline]
#[expect(unsafe_code, reason = "the crate's single audited handle reclaim")]
pub unsafe fn free_handle<T>(p: *mut T) {
    if p.is_null() {
        return;
    }
    // SAFETY: non-null per the check; `p` came from `Box::into_raw` and is dropped exactly once.
    drop(unsafe { Box::from_raw(p) });
}

/// Deep-copy a shared handle into a fresh owning handle; null → null. (Shape: `..._clone`.)
///
/// # Safety
/// `p` is null, or a valid, live handle (see the [module master contract](self)).
#[inline]
#[must_use]
#[expect(unsafe_code, reason = "the crate's single audited handle clone")]
pub unsafe fn clone_handle<T: Clone>(p: *const T) -> *mut T {
    // SAFETY: caller upholds the master contract; null is handled by `opt_ref`.
    match unsafe { opt_ref(p) } {
        Some(r) => into_handle(r.clone()),
        None => core::ptr::null_mut(),
    }
}

/// Borrow a shared handle/struct, else bail out. Two forms:
/// - `ffi_ref!(ptr, else <expr>)` — C-ABI no-op guard: `ptr` null → run `<expr>` (typically
///   `return` / `return <default>`), else bind `&*ptr`.
/// - `ffi_ref!(ptr, or <err>)?` — Result guard (std entry points inside `ffi_boundary`): `ptr`
///   null → `Err(<err>)` via `?`, else `&*ptr`.
///
/// Expands to an `unsafe {}` call to [`opt_ref`]; the enclosing fn must carry
/// `#[expect(unsafe_code, reason = …)]` (every boundary fn already does, via `#[unsafe(no_mangle)]`).
macro_rules! ffi_ref {
    ($p:expr, else $ret:expr) => {{
        // SAFETY: deref audited in `ffi_ptr::opt_ref`; caller contract per this fn's `# Safety`.
        match unsafe { $crate::ffi_ptr::opt_ref($p) } {
            ::core::option::Option::Some(r) => r,
            ::core::option::Option::None => $ret,
        }
    }};
    ($p:expr, or $err:expr) => {
        // SAFETY: deref audited in `ffi_ptr::opt_ref`; caller contract per this fn's `# Safety`.
        (unsafe { $crate::ffi_ptr::opt_ref($p) }).ok_or($err)
    };
}

/// Borrow a mutable handle/struct, else bail out. Forms mirror [`ffi_ref!`]; expands to an
/// `unsafe {}` call to [`opt_mut`].
macro_rules! ffi_mut {
    ($p:expr, else $ret:expr) => {{
        // SAFETY: deref audited in `ffi_ptr::opt_mut`; caller contract per this fn's `# Safety`.
        match unsafe { $crate::ffi_ptr::opt_mut($p) } {
            ::core::option::Option::Some(r) => r,
            ::core::option::Option::None => $ret,
        }
    }};
    ($p:expr, or $err:expr) => {
        // SAFETY: deref audited in `ffi_ptr::opt_mut`; caller contract per this fn's `# Safety`.
        (unsafe { $crate::ffi_ptr::opt_mut($p) }).ok_or($err)
    };
}

/// Build a shared input slice, else bail out. Forms:
/// - `ffi_slice!(ptr, len, else <expr>)` — `&[T]` where `ptr: *const T`.
/// - `ffi_slice!(ptr, len, <Elem>, else <expr>)` — regroups via a safe `.cast::<Elem>()` first
///   (e.g. `ffi_slice!(points, n, [f32; 3], else return)` for `points: *const f32`).
///
/// Expands to an `unsafe {}` call to [`opt_slice`].
macro_rules! ffi_slice {
    ($p:expr, $len:expr, $elem:ty, else $ret:expr) => {
        ffi_slice!($p.cast::<$elem>(), $len, else $ret)
    };
    ($p:expr, $len:expr, else $ret:expr) => {{
        // SAFETY: deref audited in `ffi_ptr::opt_slice`; caller contract per this fn's `# Safety`.
        match unsafe { $crate::ffi_ptr::opt_slice($p, $len) } {
            ::core::option::Option::Some(s) => s,
            ::core::option::Option::None => $ret,
        }
    }};
}

/// Build a mutable output slice, else bail out: `ffi_mut_slice!(ptr, len, else <expr>)`. Expands to
/// an `unsafe {}` call to [`opt_slice_mut`].
macro_rules! ffi_mut_slice {
    ($p:expr, $len:expr, else $ret:expr) => {{
        // SAFETY: deref audited in `ffi_ptr::opt_slice_mut`; caller contract per this fn's `# Safety`.
        match unsafe { $crate::ffi_ptr::opt_slice_mut($p, $len) } {
            ::core::option::Option::Some(s) => s,
            ::core::option::Option::None => $ret,
        }
    }};
}

/// Read a `Copy` value by value, else bail out. Forms:
/// - `ffi_read!(ptr, else <expr>)` — reads `T` where `ptr: *const T`.
/// - `ffi_read!(ptr, <Ty>, else <expr>)` — regroups via a safe `.cast::<Ty>()` first (e.g.
///   `ffi_read!(leaf_size, [f64; 3], else return core::ptr::null_mut())`).
///
/// Expands to an `unsafe {}` call to [`read_copy`].
macro_rules! ffi_read {
    ($p:expr, $ty:ty, else $ret:expr) => {
        ffi_read!($p.cast::<$ty>(), else $ret)
    };
    ($p:expr, else $ret:expr) => {{
        // SAFETY: read audited in `ffi_ptr::read_copy`; caller contract per this fn's `# Safety`.
        match unsafe { $crate::ffi_ptr::read_copy($p) } {
            ::core::option::Option::Some(v) => v,
            ::core::option::Option::None => $ret,
        }
    }};
}

pub(crate) use {ffi_mut, ffi_mut_slice, ffi_read, ffi_ref, ffi_slice};

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::float_cmp,
    unsafe_code,
    clippy::allow_attributes,
    reason = "test code"
)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    #[test]
    fn opt_ref_null_is_none_else_borrows() {
        let v: i32 = 42;
        // SAFETY: local valid i32 / null.
        assert_eq!(unsafe { opt_ref(&raw const v) }, Some(&42));
        assert_eq!(unsafe { opt_ref::<i32>(core::ptr::null()) }, None);
    }

    #[test]
    fn opt_mut_null_is_none_else_borrows_mut() {
        let mut v: i32 = 1;
        // SAFETY: local valid i32, unique access; then null.
        if let Some(r) = unsafe { opt_mut(&raw mut v) } {
            *r = 7;
        }
        assert_eq!(v, 7);
        assert!(unsafe { opt_mut::<i32>(core::ptr::null_mut()) }.is_none());
    }

    #[test]
    fn read_copy_null_is_none_else_reads_value() {
        let arr: [f64; 3] = [1.0, 2.0, 3.0];
        // SAFETY: cast to the same-aligned array type, then read by value / null.
        assert_eq!(
            unsafe { read_copy(arr.as_ptr().cast::<[f64; 3]>()) },
            Some(arr)
        );
        assert_eq!(unsafe { read_copy::<[f64; 3]>(core::ptr::null()) }, None);
    }

    #[test]
    fn opt_slice_null_is_none_zero_len_is_empty_else_borrows() {
        let data: [f32; 6] = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
        // SAFETY: 6 f32 regrouped into 2 xyz triples / valid ptr len 0 / null.
        let triples = unsafe { opt_slice(data.as_ptr().cast::<[f32; 3]>(), 2) };
        assert_eq!(triples, Some(&[[0.0, 1.0, 2.0], [3.0, 4.0, 5.0]][..]));
        assert_eq!(unsafe { opt_slice(data.as_ptr(), 0) }, Some(&[][..]));
        assert_eq!(unsafe { opt_slice::<f32>(core::ptr::null(), 3) }, None);
    }

    #[test]
    fn slice_or_empty_null_or_zero_len_is_empty() {
        let bytes: [u8; 3] = [1, 2, 3];
        // SAFETY: valid ptr+len / null / zero len.
        assert_eq!(unsafe { slice_or_empty(bytes.as_ptr(), 3) }, &[1, 2, 3]);
        assert!(unsafe { slice_or_empty::<u8>(core::ptr::null(), 5) }.is_empty());
        assert!(unsafe { slice_or_empty(bytes.as_ptr(), 0) }.is_empty());
    }

    #[test]
    fn opt_slice_mut_null_is_none_else_writes() {
        let mut buf: [f64; 2] = [0.0, 0.0];
        // SAFETY: unique valid buffer of 2 f64 / null.
        if let Some(s) = unsafe { opt_slice_mut(buf.as_mut_ptr(), 2) } {
            s.copy_from_slice(&[9.0, 8.0]);
        }
        assert_eq!(buf, [9.0, 8.0]);
        assert!(unsafe { opt_slice_mut::<f64>(core::ptr::null_mut(), 2) }.is_none());
    }

    #[test]
    fn write_out_null_returns_false_without_writing() {
        let mut out: i32 = -1;
        // SAFETY: valid, aligned slot / null.
        assert!(unsafe { write_out(&raw mut out, 55) });
        assert_eq!(out, 55);
        assert!(!unsafe { write_out::<i32>(core::ptr::null_mut(), 99) });
    }

    #[test]
    fn into_free_handle_round_trip() {
        let p = into_handle(Vec::from([1_u32, 2, 3]));
        assert!(!p.is_null());
        // SAFETY: `p` from `into_handle`, freed exactly once.
        unsafe { free_handle(p) };
        // Null free is a no-op.
        unsafe { free_handle::<Vec<u32>>(core::ptr::null_mut()) };
    }

    #[test]
    fn clone_handle_null_is_null_else_deep_copies() {
        let orig = into_handle(Vec::from([4_u8, 5, 6]));
        // SAFETY: `orig` is a valid handle / null.
        let copy = unsafe { clone_handle(orig.cast_const()) };
        assert!(!copy.is_null());
        // SAFETY: both handles are live and distinct; free each exactly once.
        unsafe {
            assert_eq!(&*orig, &*copy);
            free_handle(orig);
            free_handle(copy);
        }
        assert!(unsafe { clone_handle::<Vec<u8>>(core::ptr::null()) }.is_null());
    }

    #[test]
    fn ffi_ref_macro_borrows_or_returns_default() {
        fn get(p: *const i32) -> i32 {
            let r = ffi_ref!(p, else return -1);
            *r
        }
        let v = 10_i32;
        assert_eq!(get(&raw const v), 10);
        assert_eq!(get(core::ptr::null()), -1);
    }

    #[test]
    fn ffi_ref_macro_or_err_form() {
        fn get(p: *const i32) -> Result<i32, ()> {
            let r = ffi_ref!(p, or())?;
            Ok(*r)
        }
        let v = 3_i32;
        assert_eq!(get(&raw const v), Ok(3));
        assert_eq!(get(core::ptr::null()), Err(()));
    }

    #[test]
    fn ffi_mut_macro_borrows_mut_or_returns() {
        fn set(p: *mut i32) -> bool {
            let r = ffi_mut!(p, else return false);
            *r = 21;
            true
        }
        let mut v = 0_i32;
        assert!(set(&raw mut v));
        assert_eq!(v, 21);
        assert!(!set(core::ptr::null_mut()));
    }

    #[test]
    fn ffi_slice_macro_regroups_and_guards_null() {
        fn sum(p: *const f32, n: usize) -> f32 {
            let pts = ffi_slice!(p, n, [f32; 3], else return -1.0);
            pts.iter().map(|t| t[0] + t[1] + t[2]).sum()
        }
        let data: [f32; 6] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        assert_eq!(sum(data.as_ptr(), 2), 21.0);
        assert_eq!(sum(core::ptr::null(), 2), -1.0);
    }

    #[test]
    fn ffi_mut_slice_macro_writes_or_returns() {
        fn fill(p: *mut i32, n: usize) -> bool {
            let s = ffi_mut_slice!(p, n, else return false);
            s.fill(7);
            true
        }
        let mut buf = [0_i32; 3];
        assert!(fill(buf.as_mut_ptr(), 3));
        assert_eq!(buf, [7, 7, 7]);
        assert!(!fill(core::ptr::null_mut(), 3));
    }

    #[test]
    fn ffi_read_macro_regroups_and_guards_null() {
        fn first(p: *const f64) -> f64 {
            let arr = ffi_read!(p, [f64; 3], else return f64::NAN);
            arr[0]
        }
        let ls: [f64; 3] = [2.5, 0.0, 0.0];
        assert_eq!(first(ls.as_ptr()), 2.5);
        assert!(first(core::ptr::null()).is_nan());
    }
}
