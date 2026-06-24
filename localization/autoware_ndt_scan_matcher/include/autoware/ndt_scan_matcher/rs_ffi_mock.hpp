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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__RS_FFI_MOCK_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__RS_FFI_MOCK_HPP_

#include <cstdint>

namespace autoware::ndt_scan_matcher
{

// Scaffolding mock: thin C++ wrapper over the `autoware_ndt_scan_matcher_rs` Rust
// crate's C ABI. Exists to prove the C++ -> Rust FFI link end-to-end before any real
// logic is ported. Replace with the actual ported surface as the port progresses.
std::uint64_t rs_add(std::uint64_t left, std::uint64_t right);

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__RS_FFI_MOCK_HPP_
