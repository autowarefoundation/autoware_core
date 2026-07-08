# Panic containment and status codes

A Rust panic unwinding into C++ is undefined behavior, so every fallible FFI entry point contains
unwinds at the boundary and reports a status instead. This lives in `src/ffi.rs` (std-only —
`catch_unwind` needs `std`, and these wrap the ROS-node C ABI; the `no_std` kernel path does not
cross this seam).

## The boundary helpers

```rust,ignore
pub fn ffi_boundary<F: FnOnce() -> Result<(), Error>>(f: F) -> AwStatus {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(Ok(()))  => AwStatus::Ok,
        Ok(Err(e))  => e.into_status(),   // Error::NullPtr -> AwStatus::NullPtr, ...
        Err(_)      => AwStatus::Panic,   // a caught unwind, contained
    }
}
pub fn ffi_boundary_ptr<T, F>(f: F) -> *mut T   // same, but Ok -> ptr, Err/panic -> null
```

`AssertUnwindSafe` is sound here: on an unwind the helper returns `AwStatus::Panic` (or null) and
touches no shared state afterward. `Error` is the internal error enum (`NullPtr`, `InvalidParam`, …)
mapped to `AwStatus` by `into_status`.

## The contract

An FFI function's body is a closure returning `Result<(), Error>`; validation failures become an
error status, a bug that panics becomes `AwStatus::Panic`, and success is `AwStatus::Ok`. The C++
side logs any non-OK status. No panic ever crosses into C++.

> Source: `src/ffi.rs`.
