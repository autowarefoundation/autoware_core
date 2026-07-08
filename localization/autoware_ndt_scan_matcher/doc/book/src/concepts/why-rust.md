# Why a Rust port

The motivation behind the port, expanded from the [Introduction](../introduction.md).

Planned contents:

- Memory safety at the C ABI boundary (validated views, no borrowed C++ memory retained).
- Panic-free, allocation-free, WCET-bounded real-time execution.
- The `no_std` / bare-metal kernel target as the portability driver.
- What is kept from C++ (rclcpp runtime) and why not `rclrs`.
- The maintainability argument: one Rust `on_*` forwarder per callback instead of scattered
  `#ifdef NDT_USE_RUST` branches.

> Status: outline (draft to be written).
> Source: the package `README.md`; `src/lib.rs` (crate overview).
