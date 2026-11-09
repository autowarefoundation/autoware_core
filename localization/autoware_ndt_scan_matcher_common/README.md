# autoware_ndt_scan_matcher_common

## Purpose

This package holds the parts of [autoware_ndt_scan_matcher](../autoware_ndt_scan_matcher/README.md) that an NDT backend needs. An NDT backend is the implementation of the NDT scan matching that the node loads with [pluginlib](https://github.com/ros/pluginlib).

A backend package depends on this package only. It does not depend on the node and its ROS dependencies.

## Contents

- `autoware::ndt_scan_matcher::NdtBackend` in `autoware/ndt_scan_matcher_common/ndt_backend.hpp`: the interface of a backend. The header lists the rules that a backend must obey, for example for thread safety and copies.
- `ndt_omp` in `autoware/ndt_scan_matcher_common/ndt_omp/`: NDT on the CPU with OpenMP. `ndt_struct.hpp` holds the parameter and result types of the interface.
- The `omp` backend: a thin wrapper of ndt_omp, in the library `autoware_ndt_scan_matcher_common_omp_backend`.

## Add a backend

A backend package does these steps:

1. Depend on `autoware_ndt_scan_matcher_common` and `pluginlib`.
2. Implement `autoware::ndt_scan_matcher::NdtBackend`, and export the class with `PLUGINLIB_EXPORT_CLASS`.
3. Register the class with `pluginlib_export_plugin_description_file(autoware_ndt_scan_matcher <file>.xml)`.

Then the parameter `ndt.backend` of autoware_ndt_scan_matcher selects the backend by its class name in `<file>.xml`.

Do not link the backend library into other libraries. pluginlib loads it at runtime.
