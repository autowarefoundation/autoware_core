^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_command_gate
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.0 (2026-06-24)
------------------
* chore: align package versions to 1.8.0 and reset changelogs
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_command_gate): extract ROS-free mode dispatch helper (`#1145 <https://github.com/autowarefoundation/autoware_core/issues/1145>`_)
  * refactor(autoware_command_gate): extract ROS-free mode dispatch helper
  Move the request-to-mode dispatch (switch on the requested operation mode and
  build the corresponding outputs) out of the node's service-callback lambda into a
  ROS-free static helper CommandGateModeBuilder::dispatch_mode, which returns
  std::optional<ModeOutputs> (std::nullopt for unknown/unsupported modes).
  This makes the dispatch and its only error path (unknown mode -> PARAMETER_ERROR)
  unit-testable with plain gtest, without spinning up rclcpp. Behavior is preserved:
  the node still publishes outputs and sets status.success/code/message on the happy
  path, and emits success=false with PARAMETER_ERROR on an unknown mode.
  Add unit tests for dispatch_mode covering all four valid modes and the previously
  untested unknown-mode nullopt branch.
  The public node API is unchanged; dispatch_mode is an additive static method.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * refactor(autoware_command_gate): address review feedback
  Move autoware_system_msgs/srv/change_operation_mode include from the
  builder header to the .cpp where ChangeOperationMode::Request is used,
  decoupling includers from the service definition.
  Assert stamp propagation in DispatchModeAutonomous/Local/Remote tests
  by passing non-zero stamps and checking outputs->state.stamp.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  * refactor(autoware_command_gate): split mode-output and response builders (`#80 <https://github.com/autowarefoundation/autoware_core/issues/80>`_)
  Make the ChangeOperationMode service callback route through two pure
  message-in -> message-out builders: create_mode_output() builds the
  messages to publish (std::nullopt for unknown modes) and create_response()
  builds the service response. Both are ROS-node-free static functions that
  take the request, so they are unit-testable in isolation. Behavior is
  preserved; the publish-only ModeOutputs no longer carries response status.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
  ---------
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* feat: add `autoware_command_gate` and its tests (`#1012 <https://github.com/autowarefoundation/autoware_core/issues/1012>`_)
* Contributors: Junya Sasaki, Yutaka Kondo, github-actions
