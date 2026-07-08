# Behavior equivalence and verification

The port's correctness bar is **observable equivalence with the C++ implementation**. The
mechanism is a differential oracle: drive the same inputs through the legacy C++ path (`NDT_USE_RUST=OFF`)
and the Rust path (`ON`) and diff the observables. This follows a trace-based state-machine
verification workflow.

## Layers of verification

- **ON-vs-OFF differential (authoritative).** Unit gtests (helpers, covariance), property tests
  (`voxel_grid` / `kdtree` vs brute force), an align **trace diff**, and the `standard_sequence_*`
  integration tests — all must match between OFF (C++) and ON (Rust). See
  [Differential testing](differential.md) and
  [Trace-based state-machine verification](trace-verification.md).
- **Numeric parity.** The f32 `Matrix4f` pipeline, the Gauss constants, and serial/parallel
  bit-identity are checked so scores and poses match to tight tolerances. See
  [Numeric parity](numeric-parity.md).
- **Bounded WCET.** A zero-allocation-per-frame test (`tests/zero_alloc.rs`) and a worst-case
  frame-time benchmark (`examples/wcet_frame.rs`) on the serial backend, plus a real-time review
  per engine/align patch. See [The WCET contract](../rt/wcet.md).
- **`no_std` gate.** `cargo rustc --no-default-features --lib --target
  {x86_64,aarch64}-unknown-none --crate-type rlib` — proves the portable core stays kernel-buildable.
- **Unsafe FFI under Miri.** `cargo +nightly miri test` with `libm/force-soft-floats`, exercising
  the `ffi_ptr` boundary.
- **Coverage as a map, not a target.** `./coverage.sh` is diagnostic. Every test carries an oracle
  (assertion / invariant / round-trip / reference model / null-edge contract); `extern "C"` shims
  get Rust direct-call tests because `cargo llvm-cov` sees only Rust.

## Tolerances

For the deterministic paths the differential check is effectively exact; for the align result the
accepted tolerances are: pose translation ≤ 1e-3 m, rotation ≤ 1e-3 rad, TP / NVTL ≤ 1e-4,
iteration count **exact**. Where a documented upstream divergence exists (e.g. the pcl Hessian
quirk, see [Divergences from upstream](divergences.md)) the port **mirrors** it rather than
"fixing" it, so the differential test stays green.

Some behaviour is not amenable to exact traces — notably the align-service TPE search, which in
C++ samples with a libstdc++-specific `std::mt19937_64` + distributions. There, verification
splits into exact trace checks for deterministic control flow, tolerance checks for align
outcomes, and property/statistical checks (success rate, p95 error envelope) for search quality.

## Running it

```sh
colcon test --packages-select autoware_ndt_scan_matcher [--ctest-args -R <regex>]
```

For the integration tests, source the ROS + workspace environment first and filter the package's
own results via `colcon test-result --test-result-base build/<pkg>`.

## Sub-chapters

- [Differential testing](differential.md)
- [Trace-based state-machine verification](trace-verification.md)
- [Numeric parity](numeric-parity.md)

> Source: the crate `tests/` + the package's C++ gtests and `standard_sequence_*` launch tests.
