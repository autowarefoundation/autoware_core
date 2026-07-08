# Parameter reference

The node parameters, sourced from the package config and schema.

Planned contents:

- A table generated/distilled from `config/ndt_scan_matcher.param.yaml` and
  `ndt_scan_matcher.schema.json` (name, type, default, description).
- Grouping: frame/NDT params (resolution, step size, epsilon, iterations, outlier ratio,
  num_threads), convergence (`converged_param_*`), covariance (estimation mode, scale, temperature,
  offsets), regularization, map-update radius.
- How each maps to a Rust config setter (link to [Engine state](../arch/engine-state.md)).

> Status: outline (draft to be written).
> Source: `config/ndt_scan_matcher.param.yaml`, `config/ndt_scan_matcher.schema.json`, package
> `README.md`.
