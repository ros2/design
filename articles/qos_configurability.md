---
layout: default
title: External configurability of QoS policies.
permalink: articles/qos_configurability.html
abstract:
  This article describes a mechanism to allow reconfiguration of QoS settings at startup time.
author: '[Ivan Santiago Paunovic](https://github.com/ivanpauno)'
published: true
---

- This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Summary

Provide an standard way to configure QoS settings of different entities in the ROS graph at startup time, overriding the settings specified in code.

## Motivation

Up to ROS 2 Foxy, there's no standard mechanism to override QoS settings when creating a node.
It's quite common in the ROS 2 community to reuse nodes that other people have written and in a lot of use cases allowing QoS profiles to be changed make sense.

Up to now, different workarounds have been used in order to provide this reconfigurability:

- Command line arguments.
- ROS parameters to set particular policies.
- Combining parameters, the system default QoS profile, and vendor specific QoS profiles.

Here are some examples applying these mechanisms:

- rosbag2 has an ad-hoc mechanism to override QoS profiles when recording or doing playback ([docs](https://index.ros.org/doc/ros2/Tutorials/Ros2bag/Overriding-QoS-Policies-For-Recording-And-Playback/)).
- Image pipeline nodes use some parameters to allow changing some of the policies ([PR](https://github.com/ros-perception/image_pipeline/pull/521)).
- Ouster drivers also use some parameters to allow changing some policies ([PR](https://github.com/ros-drivers/ros2_ouster_drivers/pull/26)).
- Gazebo ROS packages allows configuring QoS profiles of the plugins in the SDF file ([issue](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1079)).

Not providing an standard mechanism for reconfiguring QoS results in a lot of slightly different interfaces to do the same thing, creating a pain point for end users when figuring out how to reconfigure QoS for nodes in their system.

## Design goals and other constraints

This proposal was written with the following goals in mind:

- The node's author should be able to control which entities have reconfigurable QoS, and if a profile is valid or not.
  - Rationale: Allowing to change all QoS policies could break a contract the node's implementer assumed.
  For example, the node's implementer may have assumed that the data was durable and not volatile.
- QoS should not be dynamically reconfigurable.
  - Rationale: Allowing runtime reconfiguration of QoS is complex, and in many cases it involves tearing down the previous entity and creating a new one.
  That process can cause loss of messages, and thus providing an automatic mechanism for it is not ideal.
- It should be possible to reuse the same QoS profile for different entities.
  - Rationale: Not having a way of setting the same QoS profile to different entities can cause errors, considering that in many cases it's desired to have matching qos.

The following constraint was also considered in the design:

- Only apply to publisher/subscriber QoS.
  - Rationale: QoS profiles for services and actions are not well defined currently, and modifying them is not recommended.
  Thus, making them reconfigurable doesn't make much sense right now.

## Adding a new mechanism vs using parameters

A previous proposal (see PR [#296](https://github.com/ros2/design/pull/296)) suggested using a new file format for reconfiguring QoS policies.
Some concerns were raised by the community:
1. It adds a different mechanism to the ones that already exist.
    Newcomers have to learn yet another thing.
1. QoS settings are not conceptually different to other parameters, e.g.: publishing rate.
1. It adds a new mechanism to configure QoS profiles.

Based on that feedback, this PR will explore a design based on parameters.

## Introduction

Current code will not allow QoS reconfigurability if not changed:

```cpp
node->create_publisher(
  "chatter",
  KeepLast{10});
```

To make reconfigurability easy, only a flag is needed to automatically create the parameters:

```cpp
node->create_publisher(
  "chatter",
  KeepLast{10},
  QosOverridingOptions{true});  // allow_reconfigurable all qos
```

That will automatically declare the parameters for reconfiguring the QoS policies that can be overridden in the following way:

```yaml
/my/ns/node_name:
  ros__parameters:
    qos_overrides:  # group for all the qos settings
      'my/fully/qualified/topic/name/here':
        publisher:
          reliability: reliable
          history_depth: 100
          history: keep_last
```

A callback to accept/reject the configured profile can also be specified:

```cpp
node->create_publisher(
  "chatter",
  KeepLast(10),
  [] (const QoSProfile & qos) -> bool {
      return qos.is_reliable();  // only accept reliable communication, we should add getters to rclcpp::QoSProfile class for this.
  });
```

The user provided QoS callback will be internally used as a [parameters callback](https://github.com/ros2/rclcpp/blob/3a4ac0ca2093d12035070443692798b0c9f9da3a/rclcpp/include/rclcpp/node_interfaces/node_parameters_interface.hpp#L183), but we provide a more user-friendly interface.

Note: The examples through this document are using `QosOverridingOptions` as a new argument in `create_publisher`/`create_subscription`.
Instead of that, `QosOverridingOptions` will likely be integrated in the existing `PublisherOptions`/`SubscriptionOptions` structures.

## Analysis of parameter API features and limitations

### Read-only parameters

ROS 2 currently provides read only parameters.
They can be modified when constructing a node by providing overrides (e.g. `--ros-args -p <param_name> <param_value>`), but they cannot be dynamically changed after the node was constructed.

This perfectly matches the goal of the proposal of not making QoS settings reconfigurable during runtime.

Note: parameter overrides can be constructed dynamically and then used to create a node, but it's not possible to modify the parameter after the node construction.
This allows, for example, to pass parameter overrides when dynamically adding a node to a component container.

### Parameter events

Up to ROS 2 Foxy, read-only parameters generate a parameter event with its initial value when declared.
If we start declaring parameters for each QoS policies of many entities, the amount of parameter events at startup time will significantly increase, as the declaration of each parameter generates an event.

To avoid this issue, a way to opt-out parameter events when declaring a parameter could be added.

Note: Opting-out generating a parameter event when setting/unsetting a parameter could also be possible, but not required for this.

### Parameter callbacks

If we want the following example to work:

```cpp
node->create_publisher(
  "chatter",
  KeepLast(10),
  [] (const QoSProfile & qos) -> bool{
      return qos.is_reliable();
  });
```

We could completely ignore parameters [on set callbacks](https://github.com/ros2/rclcpp/blob/3defa8fc9d7410bd833ecd95b305ac94bb9b627a/rclcpp/include/rclcpp/node_interfaces/node_parameters_interface.hpp#L180-L192), the publisher creation code would do something like:

```cpp
rcl_interfaces::msg::ParameterDescription policy_x_description{};
policy_x_description.read_only = true;
policy_x_description.description = "<string with a description of the policy, including node and topic name>";
auto policy_x_value = node->declare_parameter(
    "<name of the parameter>", qos_provided_in_code.<getter_for_policy_x>(), policy_x_description);
// Repeat for all policies
rclcpp::QoS final_qos();
// construct `final_qos` from the parameter values read above
if (!user_provided_callback(final_qos) {
    // qos profile rejected, throw error here
}
```

In that way, we avoid installing a parameter callback that will be triggered when any parameter is modified.

Note: this seems to be what you generally want in the case of read-only parameters.
Note 2: We could also have a `declare_parameters_atomically` method and use:
  ```cpp
  auto callback_handle = node->add_on_set_parameters_callback(wrapper_for_qos_callback(user_provided_callback));
  node->declare_parameters_atomically(...);  // names and defaults for all parameters given here
  // callback_handle goes out of scope, the callback is auto-removed.
  ```
  `declare_parameters_atomically` would be needed, if not you cannot have conditions like `history == keep_all or history_depth > 10`.

### Parameters URIs

Parameters URIs currently look like:

```
rosparam://my/full/namespace/node_name/param_group.nested_param_group.param_name
```

For the following QoS policy:
- Full node name: `/asd/bsd/my_node`
- Entity: `publisher`
- Topic name: `/foo/bar/topic_name`
- Policy: `reliability`

The URI of the automatically generated parameter would be:

```
rosparam://asd/bsd/my_node/qos_overrides.publisher./foo/bar/topic_name.reliability
```

The `/` in one of the subgroups looks odd, but it's currently accepted.

If in the future the addressing changes described in [#241](https://github.com/ros2/design/pull/241) are implemented, the URI would be:

```
rosparam://asd.bsd.my_node/qos_overrides/publisher/foo/bar/topic_name/reliability
```

but in both cases the previous parameters file example wouldn't change.

## Caveats

### Creating more than one publisher/subscription in the same topic with different QoS

If only the topic name is used for the parameter name, it would be impossible to create two publishers with different QoS in the same topic.

That could be solved by adding an optional extra identifier:

```cpp
node->create_publisher(
  "chatter",
  KeepLast(10),
  QosOverridingOptions {
    [] (const QoSProfile & qos) -> bool {
        return qos.is_reliable();  // only accept reliable communication
    },
    "id1"
  });
node->create_publisher(
  "chatter",
  KeepLast(10),
  QosOverridingOptions { 
    [] (const QoSProfile & qos) -> bool {
      return !qos.is_reliable();  // only accept best effort communication
    },
    "id2"
  });
```

```yaml
/my/ns/node_name:
  ros__parameters:
    qos_overrides:
      'my/fully/qualified/topic/name/here': 
        publisher_id1:  # {entity_type}_{id}
          reliability: reliable
          history_depth: 100
          history: keep_last
      'my/fully/qualified/topic/name/here': 
        publisher_id2:  # {entity_type}_{id}
          reliability: best_effort
          history_depth: 100
          history: keep_last
```

### Lowering the amount of declared parameters

This two subsections describe some optional mechanism to lower the amount of declared parameters.
The two mechanism are compatible with what it was said before and are not mutually exclusive.

#### Hidden parameters

Currently, ROS 2 has the concept of hidden topics and services.
Those aren't shown by the cli tools, except when explicitly requested:

```bash
ros2 topic list  # Will not list hidden topics.
ros2 topic --include-hidden-topics list  # Will list hidden topics.
```

Similarly, hidden parameters could be added.
All the parameters used for overriding QoS could be declared as hidden, thus avoiding "noise" in commands like `ros2 param list`.

#### Only declaring some parameters

Instead of hidden parameters that correspond to the QoS policies, we could only declare parameters for the policies that the user want to be reconfigurable.
That will lower the amount of declared parameters, but make publisher/subscription creation a bit more verbose.

Example API:

```cpp
enum class QosPolicyKind {
  Reliability,
  ...,
};

/// Options that are passed in subscription/publisher constructor
class QosOverridingOptions
{
    /// Constructor, allowing to declare all the "default" parameters. If `false` is passed, reconfiguring QoS is not allowed.
    explicit QosOverridingOptions(bool declare_default_parameters, std::string id = "");
    /// Implicit constructor, a parameter for each policy provided in the initializer list will be declared.
    QosOverridingOptions(std::initializer_list<QosPolicyKind>, std::string id = "");
    /// Same with a callback for accepting/rejecting the qos profile.
    QosOverridingOptions(std::initializer_list<QosPolicyKind>, QosCallback callback, std::string id = "");
    /// Declares the "default" parameters and pass a callback.
    QosOverridingOptions(QosCallback callback, std::string id = "");
};
```

```cpp
// New method in Node class
template<
typename MessageT,
typename AllocatorT = std::allocator<void>,
typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>>
std::shared_ptr<PublisherT>
create_publisher(
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const rclcpp::QosOverridingOptions & qos_options = QosOverridingOptions{false},
  const PublisherOptionsWithAllocator<AllocatorT> & options = PublisherOptionsWithAllocator<AllocatorT>()
);
```

```cpp
// example usage
node->create_publisher(
    "my_topic",
    user_qos_provided_in_code_1,
    {  // implicit, to lower verbosity
        {QosPolicyKind::Reliability, QosPolicyKind::History, QosPolicyKind::HistoryDepth}, // only declare parameters for history depth, history kind, reliability
        [](const QoSProfile qos) -> bool {
            return qos.is_keep_all() || qos.history_depth() > 10u;  // constrains involving more than one QoS in callbacks
        },
        "my_id"  // id to disambiguate entities in same topic
    });

// nothing is reconfigurable
node->create_publisher(
    "my_topic",
    user_qos_provided_in_code_2);

// allow reconfiguring durability
node->create_publisher(
    "other_topic",
    user_qos_provided_in_code_3,
    QosOverridingOptions{QosPolicyKind::Durability});

// "default" policies are reconfigurable
node->create_publisher(
    "another_topic",
    user_qos_provided_in_code_4,
    QosOverridingOptions{true});
```

The intent of being able to opt-in a set of "default" policies is to make the API easier to use and less verbose.

All policies could be included in the "set" of default reconfigurable policies, with some exceptions:
- Liviliness kind: It requires special care by the node's author.
- Lifespan: It only applies to publishers, thus it shouldn't be declared for subscribers.
  It also only applies for "transient local" durability, so it shouldn't be reconfigurable if durability isn't.

We could also have a restricted list of default policies to be declared in parameters:
- Reliability
- History kind
- History depth

which are the ones that usually require reconfigurability.

### Reusing profiles

It isn't currently possible to reuse a group of parameters in a parameter file.
We could, for example, leverage yaml anchors to allow this:

```yaml
/my/ns/node_name:
  ros__parameters:
    qos_overrides:
      'my/fully/qualified/topic/name/here':
        publisher: &profileA
          reliability: reliable
          history_depth: 1
          history: keep_last
/my/ns/node_name:
  ros__parameters:
    qos_overrides:
      'my/fully/qualified/topic/name/here':
        subscription:
          <<: *profileA,
          history_depth: 100  # override the history depth
```

## Reconfigurability of topics the ROS 2 core creates

TBD: discuss reconfigurability of topics packages in the ROS 2 core creates here, e.g.: clock, rosout, tf, tf_static, etc.

### Extending the ROS QoS profile with a rmw vendor specific payload

TBD

## QoS resolution order

- The QoS provided by the user in the publisher/subscription constructor is used as a default.
- Those defaults are overridden with the values provided in parameters, if the user allowed this kind of reconfigurability.
- After the two items above were completed, if a policy value is `RMW_QOS_*_POLICY_DEFAULT` then the qos resolution mechanism provided by the rmw vendor will be applied (e.g. DDS XML profile files).
