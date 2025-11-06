// Copyright 2024 TIER IV, Inc.
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

#ifndef UTILS__LOCALIZATION_CONVERSION_HPP_
#define UTILS__LOCALIZATION_CONVERSION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>
#include <autoware_localization_msgs/srv/initialize_localization.hpp>

#include <string>

namespace autoware::default_adapi::localization_conversion
{

using ExternalInitializeRequest =
  autoware_adapi_v1_msgs::srv::InitializeLocalization::Request::SharedPtr;
using InternalInitializeRequest =
  autoware_localization_msgs::srv::InitializeLocalization::Request::SharedPtr;
InternalInitializeRequest convert_request(const ExternalInitializeRequest & external);

using ExternalResponse = autoware_adapi_v1_msgs::msg::ResponseStatus;
using InternalResponse = autoware_common_msgs::msg::ResponseStatus;
ExternalResponse convert_response(const InternalResponse & internal);

template <class ClientT, class RequestT>
ExternalResponse convert_call(ClientT & client, RequestT & req)
{
  try {
    auto future = client->async_send_request(convert_request(req));

    // Wait with timeout (5 seconds) to prevent indefinite blocking
    const auto timeout = std::chrono::seconds(5);
    const auto status = future.wait_for(timeout);

    if (status == std::future_status::timeout) {
      ExternalResponse response;
      response.success = false;
      response.code = autoware_adapi_v1_msgs::msg::ResponseStatus::SERVICE_TIMEOUT;
      response.message = "Service call timed out after 5 seconds";
      return response;
    }

    if (status == std::future_status::ready) {
      auto result = future.get();
      if (result) {
        return convert_response(result->status);
      } else {
        ExternalResponse response;
        response.success = false;
        response.code = autoware_adapi_v1_msgs::msg::ResponseStatus::SERVICE_UNREADY;
        response.message = "Service returned null response";
        return response;
      }
    }

    // Handle deferred status (should not occur with async_send_request)
    ExternalResponse response;
    response.success = false;
    response.code = autoware_adapi_v1_msgs::msg::ResponseStatus::UNKNOWN;
    response.message = "Unknown future status";
    return response;
  } catch (const std::exception & e) {
    ExternalResponse response;
    response.success = false;
    response.code = autoware_adapi_v1_msgs::msg::ResponseStatus::UNKNOWN;
    response.message = std::string("Exception in service call: ") + e.what();
    return response;
  }
}

}  // namespace autoware::default_adapi::localization_conversion

#endif  // UTILS__LOCALIZATION_CONVERSION_HPP_
