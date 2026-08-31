# Running the ROS node

`autoware_ndt_scan_matcher` runs as a standard Autoware localization node. This chapter summarizes
the runtime interface; the package `README.md` is the authoritative reference for the full
parameter list.

## Topics

**In:** `ekf_pose_with_covariance` (initial pose), `points_raw` (sensor cloud), and — only when
regularization is enabled — `sensing/gnss/pose_with_covariance`.

**Out:** `ndt_pose` and `ndt_pose_with_covariance` (the estimate), `/diagnostics`, plus debug topics
(`points_aligned`, `points_aligned_no_ground`, `initial_pose_with_covariance`, `multi_ndt_pose`,
`transform_probability`, `no_ground_transform_probability`, `exe_time_ms`, …).

## Services

- `trigger_node` (`std_srvs/srv/SetBool`) — activate/deactivate the matcher.
- `ndt_align_srv` — estimate an initial pose by the align-service search (see
  [The TPE pose search](../arch/tpe.md)).

## Parameters

Grouped under `ros__parameters`: `frame.*` (base/ndt/map frames), `sensor_points.*` (timeout,
required distance), `ndt.*` (`trans_epsilon`, `step_size`, `resolution`, `max_iterations`,
`num_threads`, `regularization.*`), `initial_pose_estimation.*`, and `score_estimation.*`
(`converged_param_type` — default `1` = NVTL — and the two thresholds). See
[Scores: TP and NVTL](../concepts/scores.md) for the convergence gate and
[Parameter reference](../appendix/parameters.md) for the full table.

## Regularization (optional, off by default)

Adds a longitudinal pull toward a GNSS base position; useful on feature-poor roads where GNSS is
reliable, and counter-productive in tunnels/indoors. Enable it via the `ndt.regularization.*`
parameters and remap the regularization pose topic. See the package `README.md` "Regularization"
section.

## Choosing the backend

The node runs the legacy C++ engine or the Rust port depending on how the package was built
(`-DNDT_USE_RUST=ON`); this is a build-time choice, not a runtime parameter. See
[Build and test](build-and-test.md#selecting-the-backend-c-vs-rust).

> Source: the package `README.md`, `config/ndt_scan_matcher.param.yaml`, `launch/`.
