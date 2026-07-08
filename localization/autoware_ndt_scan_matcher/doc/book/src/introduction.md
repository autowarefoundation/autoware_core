# Introduction

This book documents the **Rust port of the Autoware NDT scan matcher**
(`autoware_ndt_scan_matcher`). It is written for the people who build on, review, and
maintain the port — not as a user manual for the ROS node (see the package
[`README.md`](https://github.com/autowarefoundation/autoware_core) for node I/O and
parameters).

The port keeps `rclcpp` as the ROS 2 runtime and moves the *algorithmic core* — the NDT
engine, scan-matching state, convergence judgement, covariance estimation, map-update
decisions, and callback bodies — into a Rust crate,
[`autoware_ndt_scan_matcher_rs`](https://github.com/autowarefoundation/autoware_core).
The C++ side becomes a thin shell that owns only the ROS 2 boundary and forwards each
callback to a single Rust entry point.

## Why this exists

- **Memory safety at the boundary.** Every pointer and struct that crosses the C ABI is
  validated on the Rust side before use.
- **Panic-free, WCET-bounded real time.** The align hot path is allocation-free after
  warmup, has a documented worst-case execution-time contract, and cannot panic.
- **A `no_std` / kernel target.** The same algorithm core builds without `std`, so it can
  run under a bare-metal kernel — a portability goal that shaped the whole design.

## What is and isn't in scope

**In scope (Rust):** the NDT engine and voxel-grid map, the align kernel, scores
(transform probability and nearest-voxel likelihood), covariance estimation, the align-service
pose search, pose buffers, convergence, map-update policy, and diagnostics content.

**Out of scope (stays C++ / `rclcpp`):** node construction, publishers/subscribers/services/
timers, parameter declaration, TF lookup, the map-loader service call, and actual message
publication. Rust never links `rclcpp` and never subscribes to topics; it requests ROS side
effects through a host vtable that C++ implements.

## The shape of the system

```mermaid
flowchart TB
    subgraph cpp["C++ / rclcpp shell"]
        node["rclcpp::Node\nsubscriptions · publishers\nservices · timers · TF"]
        host["Host vtable impl\n(ROS side effects)"]
    end
    subgraph rust["Rust / autoware_ndt_scan_matcher_rs"]
        handle["NdtScanMatcherRs\n(std FFI shell: node state)"]
        matcher["ScanMatcher (no_std core)"]
        engine["NdtEngine\n(&self-only, lock-free map)"]
        handle --> matcher --> engine
    end
    node -- "one on_* call per callback\n(C ABI, message views)" --> handle
    handle -- "side effects\n(publish, TF, log, map load)" --> host
    host --> node
```

Read [How to read this book](reader-map.md) next — it routes each kind of reader to the
chapters that matter most for them.
