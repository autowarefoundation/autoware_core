# Benchmarking

C++ vs Rust performance comparison methodology.

Planned contents:

- Principle: **capture-once, replay-everywhere** (record real inputs, replay deterministically).
- The three tiers: **L1** node / end-to-end (primary, ships first), **L2** kernel micro-benchmark
  (locate where time goes), **L3** offline differential replay (realistic distributions, zero
  jitter).
- Fidelity controls & pitfalls to keep the comparison fair.
- Metrics & reporting; phasing/milestones; CMake wiring precedent.

## Running the offline replay benchmark

The L3 replay driver (`bench/ndt_bench_replay.cpp`) is opt-in and drives both engines in one
executable, so it requires the Rust backend. Build it with `-DNDT_BUILD_BENCH=ON` (which needs
`-DNDT_USE_RUST=ON`), then run the orchestration script:

```sh
colcon build --packages-select autoware_ndt_scan_matcher \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DNDT_USE_RUST=ON -DNDT_BUILD_BENCH=ON
bench/run.sh          # build -> JSON (bench/ndt_bench.json) -> HTML (bench/report.html)
```

The per-frame WCET micro-benchmark is the crate example `examples/wcet_frame.rs`
(`cargo run --release --example wcet_frame`).

> Status: outline (draft to be written).
> Source: `bench/` (`run.sh`, `ndt_bench_replay.cpp`, `gen_report.py`); `CMakeLists.txt`
> (`NDT_BUILD_BENCH`); `examples/wcet_frame.rs`.
