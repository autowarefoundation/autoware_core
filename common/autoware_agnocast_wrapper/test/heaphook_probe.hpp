// Copyright 2026 TIER IV, Inc.
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

#pragma once

#include "autoware/agnocast_wrapper/runtime.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

namespace autoware::agnocast_wrapper::test
{

/// agnocast exits the process from inside the endpoint constructor when LD_PRELOAD lacks the
/// heaphook (validate_ld_preload() in agnocast_utils.cpp), which would take the whole test binary
/// down instead of failing one case.
inline bool agnocast_heaphook_loaded()
{
  const char * ld_preload = std::getenv("LD_PRELOAD");
  return ld_preload != nullptr &&
         std::string(ld_preload).find("libagnocast_heaphook.so") != std::string::npos;
}

}  // namespace autoware::agnocast_wrapper::test

/// GTEST_SKIP() returns from the function it appears in, so the guard cannot be a helper the
/// fixtures call.
#define AUTOWARE_SKIP_WITHOUT_AGNOCAST_HEAPHOOK()                                              \
  do {                                                                                         \
    if (                                                                                       \
      ::autoware::agnocast_wrapper::use_agnocast() &&                                          \
      !::autoware::agnocast_wrapper::test::agnocast_heaphook_loaded()) {                       \
      GTEST_SKIP() << "ENABLE_AGNOCAST=1 without the agnocast heaphook: the agnocast backend " \
                      "cannot be exercised in this environment.";                              \
    }                                                                                          \
  } while (false)
