// Copyright 2023 TIER IV, Inc.
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

// NOTE: This is an internal (non-installed) header. It is intentionally kept
// out of the public include/ tree so that it is not part of the package's
// exported API/ABI. It exists to make the pure status-mapping helper unit
// testable.

#ifndef OSQP_INTERFACE_STATUS_HPP_
#define OSQP_INTERFACE_STATUS_HPP_

#include <osqp/osqp.h>

#include <string>

namespace autoware::qp_interface
{
/// \brief Map an OSQP solver status value to its canonical string name.
/// \param status_val One of the OSQP_* status codes defined in osqp.h.
/// \return The string name of the status (e.g. "OSQP_SOLVED"), or
///         "OSQP_UNKNOWN" for an unrecognized value.
std::string status_to_string(c_int status_val);
}  // namespace autoware::qp_interface

#endif  // OSQP_INTERFACE_STATUS_HPP_
