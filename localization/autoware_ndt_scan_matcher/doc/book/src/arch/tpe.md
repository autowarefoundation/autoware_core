# The TPE pose search

The align-service (initial-pose) search uses a Tree-Structured Parzen Estimator over a 6-D pose
`[x, y, z, roll, pitch, yaw]` — `tpe::TreeStructuredParzenEstimator`.

## The propose/evaluate loop

The search ([`node_align_service::run_align_service_search_impl`](../port/trace-verification.md))
runs: draw a candidate with `get_next_input`, build a pose, `run_align` the engine from it, then feed
the outcome back with `add_trial`. It keeps the best-scoring particle and reports the per-particle
initial/result poses, scores, iteration counts, and marker/cloud publish counts.

## The sampler

Until `n_startup_trials` trials accumulate, TPE samples the prior: the five prior dims
(`x, y, z, roll, pitch`) from Gaussians around the initial pose (standard deviations from the request
covariance diagonal), plus `yaw` uniform over `[-π, π)`. After startup it partitions trials into an
"above" (better) and "below" set and picks candidates by expected improvement from above/below
Gaussian KDEs.

## Determinism

The RNG is a Rust-owned `SplitMix64` + Box-Muller, seeded per request with a fixed seed. This
deliberately does **not** reproduce libstdc++'s `std::normal_distribution` sample sequence — that
sequence is implementation-defined and not portable — which is why exact candidate-trace equivalence
with the C++ search is out of scope (see [Verification](../port/verification.md) and
[Divergences](../port/divergences.md)). It is deterministic for a fixed seed, so the search is
reproducible and unit-testable.

## C ABI

`AwTpe` is the opaque handle; `AW_TPE_STATUS_*` / `AW_TPE_DIRECTION_*` are the C-ABI status and
minimize/maximize codes.

> Source: `src/tpe.rs`, `src/node_align_service.rs`.
