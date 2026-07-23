# Why a Rust port

The motivation, expanded from the [Introduction](../introduction.md).

## Memory safety at the boundary

The C++/Rust seam is the riskiest part of any port. Here every pointer, length, and struct that
crosses the C ABI is validated on the Rust side before use, through one audited module
([`ffi_ptr`](../arch/ffi-ptr.md)) and borrow-only [view types](../arch/ffi-types.md) — Rust never
retains a borrowed C++ pointer. The soundness argument is centralized rather than scattered across
ad-hoc `unsafe` sites.

## Panic-free, WCET-bounded real time

The localization align path is safety-relevant and runs every frame. The Rust align kernel is
**allocation-free after warmup**, cannot panic (the crate bans `unwrap`/`expect`/`panic`/indexing in
non-test code), and carries a documented worst-case execution-time contract. See
[The WCET contract](../rt/wcet.md), [Zero-allocation guarantees](../rt/zero-alloc.md), and
[Panic-free, bounded execution](../rt/panic-free.md).

## A `no_std` / kernel target

Portability is the deeper driver: the same algorithm core builds without `std` so it can run under a
bare-metal kernel, not just ROS. This shaped the two-layer split (portable core vs `std` FFI shell)
and the [Host ports](../arch/host-vtable.md). See [Portability](../arch/portability.md).

## Keep rclcpp; don't rewrite ROS

The node keeps running through `rclcpp` — the port does **not** introduce `rclrs` or wrap ROS 2 in
Rust. C++ owns the ROS runtime; Rust owns the algorithm ([Scope and non-goals](scope.md)).

## Maintainability

The target shape is **one Rust `on_*` forwarder per C++ callback** instead of scattered in-function
`#ifdef NDT_USE_RUST` branches. The two backends are selected at the translation-unit / build-target
level, keeping both the Rust and legacy-C++ builds clean.

> Source: the package `README.md`; `src/lib.rs` (crate overview + module layout).
