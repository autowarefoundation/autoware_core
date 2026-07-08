# ffi_ptr helpers and guard macros

The single audited home for raw-pointer dereferences at the boundary (`src/ffi_ptr.rs`).

Planned contents:

- Why every deref is centralized: one file holds the soundness argument.
- The guard macros `ffi_ref!`, `ffi_mut!`, `ffi_slice!` — null handling via the `else` arm.
- Helper functions (e.g. `write_out`) and their contracts.
- `core + alloc` only (not `std`-gated) so the `no_std` FFI shares them.
- How this maps to the 26 FFI soundness rules.

> Status: outline (draft to be written).
> Source: `src/ffi_ptr.rs`.
