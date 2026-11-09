# autoware_ndt_scan_matcher_common

## Purpose

This package holds the types that [autoware_ndt_scan_matcher](../autoware_ndt_scan_matcher/README.md) shares with its NDT implementations. It has headers only.

## Contents

- `pclomp::NdtParams` and `pclomp::NdtResult` in `autoware/ndt_scan_matcher_common/ndt_types.hpp`: the parameters and the result of an NDT alignment.

## Compile options

`pclomp::NdtResult` holds fixed-size `Eigen` types. A package that passes these types to or from autoware_ndt_scan_matcher must compile with the same SIMD options as autoware_ndt_scan_matcher. Otherwise, the types can have a different memory alignment, and the program crashes.
