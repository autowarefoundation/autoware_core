# Running the ROS node

How to run `autoware_ndt_scan_matcher` as a ROS 2 node. This chapter summarizes and links the
package `README.md` rather than duplicating it.

Planned contents:

- Topics in/out (`ekf_pose_with_covariance`, `points_raw`, `ndt_pose(_with_covariance)`,
  `/diagnostics`, and the debug topics).
- Services: `trigger_node`, `ndt_align`.
- Key parameters (link to [Parameter reference](../appendix/parameters.md)).
- Regularization (optional, off by default).
- How the Rust path is selected at build time (`NDT_USE_RUST`) — see
  [Build and test](build-and-test.md#selecting-the-backend-c-vs-rust).

> Status: outline (draft to be written).
> Source: package `README.md`, `config/ndt_scan_matcher.param.yaml`, `launch/`.
