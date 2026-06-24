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

// C ABI exported by the `autoware_ndt_scan_matcher_rs` Rust crate.
// Hand-written for now; switch to cbindgen-generated output once the surface grows.

#ifndef AUTOWARE_NDT_SCAN_MATCHER_RS_H_
#define AUTOWARE_NDT_SCAN_MATCHER_RS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint64_t autoware_ndt_scan_matcher_rs_add(uint64_t left, uint64_t right);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AUTOWARE_NDT_SCAN_MATCHER_RS_H_
