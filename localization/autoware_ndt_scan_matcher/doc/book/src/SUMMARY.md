# Summary

[Introduction](introduction.md)
[How to read this book](reader-map.md)

# Part I — Concepts

- [NDT scan matching primer](concepts/ndt-primer.md)
- [Scores: TP and NVTL](concepts/scores.md)
- [Why a Rust port](concepts/why-rust.md)
- [Scope and non-goals](concepts/scope.md)

# Part II — Getting Started

- [Build and test](start/build-and-test.md)
- [Feature flags and build configurations](start/features.md)
- [Using the Rust crate](start/using-the-crate.md)
- [Running the ROS node](start/ros-node.md)

# Part III — Architecture

- [System overview](arch/overview.md)
- [The FFI boundary](arch/ffi-boundary.md)
    - [C ABI types and view types](arch/ffi-types.md)
    - [ffi_ptr helpers and guard macros](arch/ffi-ptr.md)
    - [Panic containment and status codes](arch/panic-containment.md)
    - [The Host abstraction and C vtables](arch/host-vtable.md)
- [The NDT engine](arch/engine.md)
    - [Engine state and the config API](arch/engine-state.md)
    - [Concurrency and interior mutability](arch/concurrency.md)
    - [MatchScratch and the align entry points](arch/scratch.md)
- [The align hot path](arch/align.md)
    - [Voxel grid and kd-tree](arch/voxel-grid.md)
    - [Serial and parallel derivatives](arch/derivatives.md)
- [Covariance estimation](arch/covariance.md)
    - [The TPE pose search](arch/tpe.md)
- [Map update](arch/map-update.md)
- [Portability and the Host ports](arch/portability.md)

# Part IV — The C++ to Rust Port

- [Porting strategy and phases](port/strategy.md)
- [Behavior equivalence and verification](port/verification.md)
    - [Differential testing](port/differential.md)
    - [Trace-based state-machine verification](port/trace-verification.md)
    - [Numeric parity](port/numeric-parity.md)
- [Divergences from upstream](port/divergences.md)
- [C++ to Rust map](port/symbol-map.md)

# Part V — Real-Time and no_std

- [The WCET contract](rt/wcet.md)
- [Zero-allocation guarantees](rt/zero-alloc.md)
- [The `mt` multi-core engine](rt/mt.md)
- [Panic-free, bounded execution](rt/panic-free.md)

# Part VI — Quality Gates

- [Lint gates and suppression policy](quality/hardening.md)
- [Test taxonomy](quality/tests.md)
- [Benchmarking](quality/benchmarks.md)
- [Contributing and PR conventions](quality/contributing.md)

# Appendices

- [Glossary](appendix/glossary.md)
- [Parameter reference](appendix/parameters.md)
- [Module index](appendix/modules.md)
- [References](appendix/references.md)
