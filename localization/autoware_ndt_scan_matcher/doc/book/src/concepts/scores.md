# Scores: TP and NVTL

NDT reports two quality scores from a converged pose; both are published, and one of them gates the
convergence decision.

## Transform Probability (TP)

TP is the NDT score accumulated over the whole cloud, normalized by the point count — the average
per-point Gaussian response at the final pose (`red.score / source_len` in `ndt.rs`'s `finalize`,
exposed as `AlignResult::transform_probability`). Higher is better; it reflects how well the source
sits in the map's distributions overall.

## Nearest-Voxel Transformation Likelihood (NVTL)

NVTL scores each transformed point against only its **nearest** voxel and averages over the points
that found one (`nearest_voxel_score / found`). It is more robust than TP on sparse or partially
observed maps, so it is the default convergence gate in Autoware.

## Which score gates convergence

`converged_param_type` selects the gate: `0` = transform probability, `1` = nearest-voxel likelihood
(the config default is `1`). The selected score is compared against its threshold
(`converged_param_transform_probability` or
`converged_param_nearest_voxel_transformation_likelihood`); the pure decision — including the
oscillation override — is [`convergence::evaluate_convergence`](../port/verification.md), whose
inputs/verdict are documented on `convergence::ConvergenceInput` / `ConvergenceVerdict`.

## Traces and no-ground variants

`align` also records the per-iteration score traces (`transform_probability_array`,
`nearest_voxel_likelihood_array`) for diagnostics/plotting. The sensor callback additionally computes
**no-ground** TP/NVTL (scores on the cloud with ground points filtered) and publishes them as debug
topics. Both scores travel out of a match in [`host::MatchResult`](../arch/host-vtable.md).

## How the scores feed downstream

The gate result drives publish/skip decisions; the align result's Hessian and NVTL also seed
[covariance estimation](../arch/covariance.md).

> Source: `src/convergence.rs`, `src/ndt.rs` (`transformation_probability`,
> `nearest_voxel_transformation_likelihood`, `finalize`), `src/host.rs` (`MatchResult`).
