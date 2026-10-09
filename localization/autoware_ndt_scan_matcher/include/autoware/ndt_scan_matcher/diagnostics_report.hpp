// Copyright 2022 Autoware Foundation
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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__DIAGNOSTICS_REPORT_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__DIAGNOSTICS_REPORT_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::ndt_scan_matcher
{

// Severity of a diagnostics update. Mirrors diagnostic_msgs::msg::DiagnosticStatus levels so the
// ROS-free modules that produce a report need no ROS diagnostics dependency.
enum class DiagnosticLevel : int8_t { OK = 0, WARN = 1, ERROR = 2, STALE = 3 };

// A single diagnostic key/value. The value keeps its type so the ROS node can format it exactly
// the way DiagnosticsInterface would (e.g. bool as "True"/"False").
//
// Note on the numeric alternatives: DiagnosticsInterface formats with std::to_string, so widening
// an int to int64_t or a float to double on the way in produces the same string it would have
// produced from the narrower type. Key values therefore survive this round trip unchanged.
struct DiagnosticKeyValue
{
  std::string key;
  std::variant<bool, int64_t, double, std::string> value;
};

// Diagnostics accumulated while a module does its work: key/values plus an overall status (level +
// message). Returned to the ROS node, which forwards it to a DiagnosticsInterface. Plain data,
// kept ROS-free on purpose.
struct DiagnosticsReport
{
  DiagnosticLevel level{DiagnosticLevel::OK};
  std::string message;
  std::vector<DiagnosticKeyValue> key_values;

  void add_key_value(DiagnosticKeyValue key_value) { key_values.push_back(std::move(key_value)); }

  // Accumulates like DiagnosticsInterface: raises the level and appends the message.
  void update_level_and_message(DiagnosticLevel new_level, const std::string & new_message)
  {
    if (static_cast<int8_t>(new_level) > static_cast<int8_t>(DiagnosticLevel::OK)) {
      if (!message.empty()) {
        message += "; ";
      }
      message += new_message;
    }
    if (static_cast<int8_t>(new_level) > static_cast<int8_t>(level)) {
      level = new_level;
    }
  }
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__DIAGNOSTICS_REPORT_HPP_
