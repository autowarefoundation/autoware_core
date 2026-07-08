# C ABI types and view types

The concrete `#[repr(C)]` vocabulary that crosses the boundary. The cbindgen-generated header
(`autoware_ndt_scan_matcher_rs.h`) is the single source of truth — the C++ side is never hand-synced
against the Rust definitions.

## What may cross

Scalars (`i32`/`u32`/`i64`/`u64`/`f32`/`f64`), explicit `bool`, fixed-size numeric arrays (e.g.
`[f64; 36]` for a row-major 6×6), raw pointer + length, opaque handles, and `#[repr(C)]` structs /
enums with explicit integer representation. Nothing else — no STL, Eigen, PCL, `rclcpp`/`tf2`, Rust
`Vec`/`String`, or non-FFI-safe references.

## Result codes

`AwStatus` is the C-ABI result enum returned by the fallible entry points (see
[Panic containment](panic-containment.md)); align-service and search entry points use their own
`NDT_ALIGN_SERVICE_STATUS_*` / status integers.

## Borrowed view types

ROS messages cross as thin borrowed views, read **only during the call**:

```rust,ignore
#[repr(C)] pub struct AwStr { ptr: *const u8, len: usize }        // UTF-8 bytes, not owned
#[repr(C)] pub struct AwPose { position: [f64; 3], orientation: [f64; 4] }  // xyz + quat xyzw
#[repr(C)] pub struct AwPointCloud2View { /* stamp, frame_id, strides, data ptr+len, x/y/z offsets */ }
```

A `PointCloud2` view is untrusted: its field datatype and count are validated (expect `FLOAT32`
x/y/z) *before* the bytes are decoded — never reinterpreted blindly. Any data Rust must retain is
copied into a Rust-owned structure first ([`ffi_ptr`](ffi-ptr.md) does the audited reads).

## Parameters

`AwNdtParams` crosses as a C-ABI struct of scalars plus `(ptr, len)` "offset model" arrays (the
covariance search offsets), all **copied** into a Rust-owned `Params` at handle construction so no
borrowed pointer is retained.

> Source: `src/sensor_points.rs` (`AwPointCloud2View`), `src/ffi_host.rs` (`AwStr`, `AwPose`),
> `src/node_handle.rs` (`AwNdtParams`), `build.rs` (cbindgen).
