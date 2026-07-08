# Panic containment and status codes

No Rust panic may unwind into C++. This chapter covers the boundary helpers.

Planned contents:

- `ffi_boundary` — `catch_unwind` → `AwStatus` (`Ok` / error / `Panic`).
- `ffi_boundary_ptr` — `catch_unwind` → pointer or null.
- `AwStatus` / `Error` and `into_status`.
- The rule that C++ logs non-OK statuses.
- Interaction with the `no_std` build (where `catch_unwind`/`std` panic is unavailable) and how
  the `ffi.rs` module is `#[cfg(feature = "std")]`.

> Status: outline (draft to be written).
> Source: `src/ffi.rs`.
