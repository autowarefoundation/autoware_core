# Covariance estimation

Estimating the 6×6 pose covariance published alongside the match result.

Planned contents:

- The four modes: `FIXED_VALUE` (0), `LAPLACE` (1), `MULTI_NDT` (2), `MULTI_NDT_SCORE` (3).
- `CovarianceConfig` (estimation type, scale factor, softmax temperature, configured 6×6, XY
  search offsets) and how `estimate_covariance` reads it.
- The Laplace approximation from the Hessian; the multi-NDT candidate re-align/score and softmax
  weighting.
- `rotate_covariance` (map→base_link rotation of the position block).
- Symmetry/PSD checks (`multi_ndt_covariance_is_symmetric_psd`).

> Status: outline (draft to be written).
> Source: `src/covariance.rs`, `src/cov_estimate.rs`, `src/engine.rs` (`estimate_covariance`),
> `src/helper.rs` (`rotate_covariance`).
