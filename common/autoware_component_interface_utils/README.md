# autoware_component_interface_utils

## Features

This is a utility package that provides the following features:

- Instantiation of the wrapper class
- Opt-in service introspection for service and client
- Service exception for response
- Relays for topic and service

## Design

This package provides the wrappers for the interface classes of rclcpp.
The wrappers limit the usage of the original class to enforce the processing recommended by the component interface.
Do not inherit the class of rclcpp, and forward or wrap the member function that is allowed to be used.

## Instantiation of the wrapper class

The wrapper class requires interface information in this format.

```cpp
struct SampleService
{
  using Service = sample_msgs::srv::ServiceType;
  static constexpr char name[] = "/sample/service";
};

struct SampleMessage
{
  using Message = sample_msgs::msg::MessageType;
  static constexpr char name[] = "/sample/message";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};
```

Create the wrapper using the above definition as follows.

```cpp
// header file
autoware::component_interface_utils::Service<SampleService>::SharedPtr srv_;
autoware::component_interface_utils::Client<SampleService>::SharedPtr cli_;
autoware::component_interface_utils::Publisher<SampleMessage>::SharedPtr pub_;
autoware::component_interface_utils::Subscription<SampleMessage>::SharedPtr sub_;

// source file
const auto node = autoware::component_interface_utils::NodeAdaptor(this);
node.init_srv(srv_, callback);
node.init_cli(cli_);
node.init_pub(pub_);
node.init_sub(sub_, callback);
```

The adaptor also provides returning forms that take the specification as an explicit template argument instead of deducing it from the output variable.
They create the same wrappers as the `init_*` methods, so the two styles can be mixed.
These are non-const member functions, so the adaptor must not be declared `const` when they are used.

```cpp
// source file
auto node = autoware::component_interface_utils::NodeAdaptor(this);
srv_ = node.create_service<SampleService>(callback);
cli_ = node.create_client<SampleService>();
pub_ = node.create_publisher<SampleMessage>();
sub_ = node.create_subscription<SampleMessage>(callback);
```

## Opt-in service introspection for service and client

This package does not trace service calls by itself.
What it provides is a single node-scoped switch that turns on the standard [ROS 2 service introspection](https://docs.ros.org/en/jazzy/Tutorials/Demos/Service-Introspection.html) for every wrapper of that node, so the wrappers do not have to be configured one by one.
Each node that creates wrappers through `NodeAdaptor` declares the following parameter, and its value is applied to all the `Service` and `Client` wrappers that the adaptor creates.

| Name                                        | Type     | Default | Description                                                                       |
| ------------------------------------------- | -------- | ------- | --------------------------------------------------------------------------------- |
| `component_interface.service_introspection` | `string` | `off`   | Introspection mode of the service wrappers. One of `off`, `metadata`, `contents`. |

Introspection is disabled by default, so nothing is recorded unless the parameter is set.
When the mode is not `off`, each wrapper exposes the service event topic `<service name>/_service_event`, which is `/sample/service/_service_event` for the specification above.
The `metadata` mode records the event types and timestamps, and the `contents` mode also records the request and response contents.

Service introspection requires ROS 2 Iron (rclcpp 21) or later.
On ROS 2 Humble the parameter is not declared and the wrappers are created without introspection.

The `autoware_universe` version of this package published a `tier4_system_msgs/ServiceLog` message on a single `/service_log` topic for every request, response and error, and that tracer is not part of this package.
Service introspection is not a drop-in replacement for it: the events are published per service instead of being aggregated on one topic, they use the event types that ROS 2 generates instead of `tier4_system_msgs/ServiceLog`, and they are off unless enabled.
An unready service and a service timeout are still reported by `Client::call` as the `ServiceUnready` and `ServiceTimeout` exceptions.

## Service exception for response

If the wrapper class is used and the service response has status, throwing `ServiceException` will automatically catch and set it to status.
This is useful when returning an error from a function called from the service callback.

```cpp
void service_callback(Request req, Response res)
{
   function();
   res->status.success = true;
}

void function()
{
   throw ServiceException(ERROR_CODE, "message");
}
```

If the wrapper class is not used or the service response has no status, manually catch the `ServiceException` as follows.

```cpp
void service_callback(Request req, Response res)
{
   try {
      function();
      res->status.success = true;
   } catch (const ServiceException & error) {
      res->status = error.status();
   }
}
```

## Relays for topic and service

There are utilities for relaying services and messages of the same type.

```cpp
const auto node = autoware::component_interface_utils::NodeAdaptor(this);
service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
node.relay_message(pub_, sub_);
node.relay_service(cli_, srv_, service_callback_group_);  // group is for avoiding deadlocks
```
