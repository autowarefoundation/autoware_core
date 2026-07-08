# Scores: TP and NVTL

The two scores NDT reports, and how they gate convergence and covariance.

Planned contents:

- **Transform Probability (TP)** — definition, how it is accumulated over the cloud.
- **Nearest-Voxel Transformation Likelihood (NVTL)** — definition, why it is more robust in
  sparse maps.
- `converged_param_type` — which score gates convergence (`0` = TP, `1` = NVTL) and the
  corresponding thresholds.
- The per-iteration score traces (`transform_probability_array`, `nearest_voxel_likelihood_array`).
- The no-ground variants published for diagnostics.
- Link to [convergence](../port/strategy.md) and [covariance estimation](../arch/covariance.md).

> Status: outline (draft to be written).
> Source: `src/convergence.rs`, `src/ndt.rs` (`transformation_probability`,
> `nearest_voxel_transformation_likelihood`), `src/host.rs` (`MatchResult`).
