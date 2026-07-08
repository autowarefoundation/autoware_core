# Parameter reference

The node's ROS parameters, from `config/ndt_scan_matcher.param.yaml` (defaults shown). The package
`README.md` and `schema/` are the authoritative, exhaustive reference — this table is the working
subset most relevant to the port.

## `frame`

| Parameter | Default | Meaning |
|---|---|---|
| `base_frame` | `base_link` | vehicle reference frame |
| `ndt_base_frame` | `ndt_base_link` | NDT reference frame |
| `map_frame` | `map` | map frame |

## `sensor_points`

| Parameter | Default | Meaning |
|---|---|---|
| `timeout_sec` | 1.0 | max stamp difference between now and the sensor cloud [s] |
| `required_distance` | 10.0 | min max-range of the input cloud, else skip matching [m] |

## `ndt`

| Parameter | Default | Meaning |
|---|---|---|
| `trans_epsilon` | 0.01 | convergence tolerance between consecutive transforms |
| `step_size` | 0.1 | More-Thuente line-search max step |
| `resolution` | 2.0 | voxel-grid resolution [m] |
| `max_iterations` | 30 | optimizer iteration cap |
| `num_threads` | 4 | derivative-reduction worker count (`>1` ⇒ parallel; sizes the global pool — see [features](../start/features.md#parallelism-and-worker-threads)) |
| `regularization.enable` | false | enable longitudinal regularization |
| `regularization.scale_factor` | 0.01 | regularization strength |

## `initial_pose_estimation` (align service)

| Parameter | Default | Meaning |
|---|---|---|
| `particles_num` | 200 | TPE particles for the initial-pose search |
| `n_startup_trials` | 100 | TPE random startup trials (≤ `particles_num`) |

## `validation`

| Parameter | Default | Meaning |
|---|---|---|
| `initial_pose_timeout_sec` | 1.0 | max stamp diff between initial pose and cloud [s] |
| `initial_pose_distance_tolerance_m` | 10.0 | max distance between the two interpolation poses [m] |
| `initial_to_result_distance_tolerance_m` | 3.0 | max initial→result distance [m] |
| `critical_upper_bound_exe_time_ms` | 100.0 | exec time above which matching is deemed failing [ms] |
| `skipping_publish_num` | 5 | consecutive rejects tolerated |

## `score_estimation`

| Parameter | Default | Meaning |
|---|---|---|
| `converged_param_type` | 1 | gate: 0 = TP, 1 = NVTL |
| `converged_param_transform_probability` | 3.0 | TP threshold (type 0) |
| `converged_param_nearest_voxel_transformation_likelihood` | 2.3 | NVTL threshold (type 1) |
| `no_ground_points.enable` | false | also score on the no-ground cloud |

The `covariance.*` group (estimation mode, scale, softmax temperature, configured 6×6, XY search
offsets) drives [covariance estimation](../arch/covariance.md); see the README/schema for its full
list. Each of these maps to a Rust engine setter (see [Engine state](../arch/engine-state.md)).

> Source: `config/ndt_scan_matcher.param.yaml`, `config/ndt_scan_matcher.schema.json`, the package
> `README.md`.
