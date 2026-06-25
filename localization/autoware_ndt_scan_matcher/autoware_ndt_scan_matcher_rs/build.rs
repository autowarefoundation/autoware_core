// Copyright 2024 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// In a build script, panicking (expect) is the idiomatic way to fail the build.
#![allow(clippy::expect_used)]

// ROS-message bindings are only generated under the `ros` feature (the ROS-node build).
// The no_std / awkernel build leaves `ros` off, so bindgen / libclang / ROS headers are not needed.
#[cfg(feature = "ros")]
fn main() {
    use std::env;
    use std::path::PathBuf;

    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-env-changed=ROS_INCLUDE_DIRS");

    // Colon-separated include dirs supplied by CMake (geometry_msgs et al.).
    let include_dirs = env::var("ROS_INCLUDE_DIRS").unwrap_or_default();

    let mut builder = bindgen::Builder::default()
        .header("wrapper.h")
        .allowlist_type("geometry_msgs__msg__Pose")
        .allowlist_type("geometry_msgs__msg__Point")
        .allowlist_type("geometry_msgs__msg__Quaternion")
        .use_core() // no_std-friendly output (core:: instead of std::)
        .layout_tests(true) // emit size/align/offset tests = layout verification vs the C header
        .derive_default(true);

    for dir in include_dirs.split(':').filter(|s| !s.is_empty()) {
        builder = builder.clang_arg(format!("-I{dir}"));
    }

    let bindings = builder.generate().expect("bindgen failed to generate ROS bindings");
    let out_path = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR not set"));
    bindings
        .write_to_file(out_path.join("ros_msgs.rs"))
        .expect("failed to write ros_msgs.rs");
}

#[cfg(not(feature = "ros"))]
fn main() {}
