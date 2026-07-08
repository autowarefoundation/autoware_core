# ffi_ptr helpers and guard macros

Every raw-pointer dereference at the boundary goes through one audited module, `src/ffi_ptr.rs`.
There are no ad-hoc `unsafe { *ptr }` sites elsewhere in the crate — so the soundness argument for
foreign pointers lives in exactly one file.

## Guard macros

The macros borrow a foreign pointer and run an `else` arm on null (no panic), so callers write a
total function:

```rust,ignore
let input = ffi_ref!(input_ptr, else return AwStatus::NullPtr);      // &T   (null -> else)
let out   = ffi_mut!(out_ptr,   else return AwStatus::NullPtr);      // &mut T
let src   = ffi_slice!(ptr, len, [f32; 3], else return STATUS_ERR);  // &[T] from ptr + len
let buf   = ffi_mut_slice!(ptr, len, else return STATUS_ERR);        // &mut [T]
let val   = ffi_read!(ptr, else return);                             // read a Copy value out
```

Under the hood they wrap the documented helpers `as_ref` / `as_mut` (thin, audited wrappers over
`<*const T>::as_ref` / `as_mut`, which already null-check), plus `write_out` (write a value through
an `*mut T`, no-op on null) and `into_handle` (box a value into an owned `*mut T`).

## Why centralize

Each macro carries the module's "master contract" (non-null-or-handled, aligned, initialized, valid
for the borrow's lifetime, unique for `_mut`). Concentrating the derefs means a reviewer audits one
module against the [26 FFI soundness rules](ffi-boundary.md) instead of every call site. The module
is `core + alloc` only (not `std`-gated), because FFI functions exist in the `no_std` build too.

> Source: `src/ffi_ptr.rs`.
