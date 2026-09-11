// Copyright 2022 TIER IV, Inc.
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

#include "localization.hpp"

#include "utils/localization_conversion.hpp"

namespace autoware::default_adapi
{

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions & options)
: autoware::agnocast_wrapper::Node("localization", options), diagnostics_(this)
{
  diagnostics_.setHardwareID("none");
  diagnostics_.add("state", this, &LocalizationNode::diagnose_state);

  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // AD API
  pub_state_ =
    adaptor_.create_publisher<autoware::adapi_specs::localization::InitializationState>();
  srv_initialize_ = adaptor_.create_service<autoware::adapi_specs::localization::Initialize>(
    this, &LocalizationNode::on_initialize, group_cli_);

  // Component Interface
  sub_state_ =
    adaptor_
      .create_subscription<autoware::component_interface_specs::localization::InitializationState>(
        this, &LocalizationNode::on_state);
  cli_initialize_ =
    adaptor_.create_client<autoware::component_interface_specs::localization::Initialize>();

  state_.state = ImplState::Message::UNKNOWN;
}

void LocalizationNode::diagnose_state(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  const auto message = std::to_string(state_.state);

  if (state_.state == ImplState::Message::INITIALIZED) {
    stat.summary(DiagnosticStatus::OK, message);
  } else {
    stat.summary(DiagnosticStatus::ERROR, message);
  }
}

void LocalizationNode::on_state(const ImplState::Message::ConstSharedPtr msg)
{
  state_ = *msg;
  pub_state_->publish(*msg);
}

void LocalizationNode::on_initialize(
  const autoware::adapi_specs::localization::Initialize::Service::Request::SharedPtr req,
  const autoware::adapi_specs::localization::Initialize::Service::Response::SharedPtr res)
{
  if (!cli_initialize_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Initialize service is not ready");
    return;
  }
  res->status = localization_conversion::convert_call(cli_initialize_, req);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::LocalizationNode)
