# Covariance estimation

After a match, the 6×6 pose covariance published with the estimate is produced by one of four modes,
selected by `CovarianceConfig.estimation_type` and driven by
[`NdtEngine::estimate_covariance`](engine.md).

## The four modes

- **`FIXED_VALUE` (0)** — the configured `output_pose_covariance`, rotated into the map frame by the
  result pose's rotation (`helper::rotate_covariance`).
- **`LAPLACE` (1)** — the XY covariance from the align **Hessian** (`laplace_xy_covariance`, the
  Laplace approximation `H⁻¹`).
- **`MULTI_NDT` (2)** — re-run the full `align` from each candidate pose around the result
  (`propose_poses_to_search(center, offset_x, offset_y)`), then take the sample XY covariance of the
  converged poses with the unbiased `(n-1)/n` correction (`estimate_xy_covariance_by_multi_ndt`).
- **`MULTI_NDT_SCORE` (3)** — for each candidate, transform the source and score it (no re-align),
  then a temperature-scaled softmax of the scores weights a weighted mean/covariance
  (`estimate_xy_covariance_by_multi_ndt_score`, `calc_weight_vec`).

## Frame handling

The XY covariance is computed in base_link, clamped to floors, and rotated back to map:
`rotate_covariance_to_base_link` / `rotate_covariance_to_map` (`Rᵀ C R` / `R C Rᵀ` on the 2×2 yaw
block) and `adjust_diagonal_covariance`. The engine wrapper builds the 3×3 `rot3x3` from the result
pose's rotation block, so the full 6×6's position block is rotated consistently.

## Configuration and checks

The mode, `scale_factor`, softmax `temperature`, the configured 6×6, and the XY search offsets come
from `set_covariance_config`. The multi-NDT result is symmetric PSD (pinned by
`multi_ndt_covariance_is_symmetric_psd`).

> Source: `src/cov_estimate.rs`, `src/covariance.rs`, `src/engine.rs` (`estimate_covariance`),
> `src/helper.rs` (`rotate_covariance`).
