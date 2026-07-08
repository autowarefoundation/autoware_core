# C ABI types and view types

The concrete `#[repr(C)]` vocabulary that crosses the boundary.

Planned contents:

- Status/result: `AwStatus`. Scalars and fixed arrays allowed across FFI.
- View types: `AwStr`, `AwPose`, `AwPoseWithCovarianceStampedView`, `AwPointCloud2View`,
  `AwPoint3fSlice` — borrow-only, read during the call, never retained.
- Parameters: `AwNdtParams` (scalars + `(ptr, len)` offset models, copied Rust-side).
- The cbindgen-generated header as single source of truth.
- Validation rules per type (e.g. PointCloud2 field datatype/count before decode).

> Status: outline (draft to be written).
> Source: `src/sensor_points.rs`, `src/node_handle.rs` (`AwNdtParams`), `build.rs` (cbindgen).
