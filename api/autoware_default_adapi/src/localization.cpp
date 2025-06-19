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

#include <autoware/component_interface_specs/utils.hpp>

namespace autoware::default_adapi
{

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions & options)
: Node("localization", options), diagnostics_(this)
{
  diagnostics_.setHardwareID("none");
  diagnostics_.add("state", this, &LocalizationNode::diagnose_state);

  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // AD API
  pub_state_ = create_publisher<autoware::adapi_specs::localization::InitializationState::Message>(
    autoware::adapi_specs::localization::InitializationState::name,
    autoware::component_interface_specs::get_qos<
      autoware::adapi_specs::localization::InitializationState>());
  srv_initialize_ = create_service<autoware::adapi_specs::localization::Initialize::Service>(
    autoware::adapi_specs::localization::Initialize::name,
    std::bind(&LocalizationNode::on_initialize, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, group_cli_);

  // Component Interface
  sub_state_ = create_subscription<
    autoware::component_interface_specs::localization::InitializationState::Message>(
    autoware::component_interface_specs::localization::InitializationState::name,
    autoware::component_interface_specs::get_qos<
      autoware::component_interface_specs::localization::InitializationState>(),
    std::bind(&LocalizationNode::on_state, this, std::placeholders::_1));
  cli_initialize_ =
    create_client<autoware::component_interface_specs::localization::Initialize::Service>(
      autoware::component_interface_specs::localization::Initialize::name);

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
