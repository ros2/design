---
layout: default
title: Configuring QoS from an external file
permalink: articles/qos_file.html
abstract:
  This article describes how to configure QoS from an external file.
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

## Overview

`ROS 2` added the [Quality of Service (QoS)](qos.md) to publishers, subscriptions, clients and services.
Up `ROS 2 Foxy`, the `QoS` can only be specified in code.
To avoid recompiling the source code with patched `QoS`, node implementers have used different mechanisms to configure `QoS` profiles:
- Command line arguments
- ROS parameters
- Combining system default QoS with vendor specific `QoS` profiles.

Instead of having different ways of externally specifying `QoS` for the different endpoints, it would be good to have a standardized way to specific `QoS` profiles.

## QoS file format

DDS uses the concept of QoS profiles ([rti documentation](https://community.rti.com/examples/using-qos-profiles)), in which each profile has a name. When creating an entity (e.g. a `DataWriter`), the passed profile name will be looked up in the qos profiles file being used, and the QoS can be loaded in that way.

ROS 2 can use a simpler approach, leveraging node name uniqueness <sup id="back_node_name_uniqueness">[1](#to_node_name_uniqueness)</sup>.

<b id="to_node_name_uniqueness">1</b> Node name uniqueness is not being enforced up to Foxy, though it is supposed. [↩](#back_node_name_uniqueness)

### Nodes

In an acceptable `QoS` file format, it should be to distinguish `QoS` settings targeted to different nodes.
Thus, a top level tag to identify the node should be available.
Example alternatives:

```xml
<qos_profiles>
    <node name="my_node" namespace="/my_ns/nested_ns">
        ...
    </node>
<qos_profiles>
```

```yaml
/my_ns/nested_ns/my_node:
    ros__qos_profiles:
        ...
```

### Endpoint type

It should be possible to distinguish between the different endpoint types in a qos profile file.
For example:

```xml
<qos_profiles>
    <node name="my_node" namespace="/my_ns/nested_ns">
        <publisher topic_name="asd">  <!--Relative name-->
            ...
        </publisher>
        <subscription topic_name="/another_ns/asd">  <!--Absolute name-->
            ...
        </subscription>
    </node>
<qos_profiles>
```

```yaml
/my_ns/nested_ns/my_node:
    ros__qos_profiles:
        publisher:
            topic_name: asd
            qos:
                ...
        subscription:
            topic_name: /another_ns/asd
            qos:
                ...
```

Besides `publisher` and `subscription` tags, `client` and `service` should be allowed.

### QoS profiles ids (optional)

In the case that a node wants to create two `publishers` on the same topic with different QoS, the above format wouldn't allow to identify uniquely the `publisher`. The same applies to other kinds of endpoints.

To solve that issue, the format could allow an using a profile id, instead of the topic name:

```xml
<qos_profiles>
    <node name="my_node" namespace="/my_ns/nested_ns">
        <publisher topic_name="asd" profile_id="1">
            ...
        </publisher>
        <subscription topic_name="asd" profile_id="2">
            ...
        </subscription>
    </node>
<qos_profiles>
```

```yaml
/my_ns/nested_ns/my_node:
    ros__qos_profiles:
        publisher:
            topic_name: asd
            profile_id: 1
            qos:
                ...
        subscription:
            topic_name: asd
            profile_id: 2
            qos:
                ...
```

`rcl` API would need to be extended to support profiles id.
This mechanism should be avoided, except in the few use cases where there are collisions.

### Default profiles for endpoints in different nodes

It is common to have several nodes creating the same endpoint, and the same qos profile is desired in all of them.
A default tag can be added to solve this:

```xml
<qos_profiles>
    <default>
        <publisher topic_name="/my_ns/nested_ns/asd">
            ...
        </publisher>
    </default>
    <node name="my_node" namespace="/my_ns/nested_ns">
        <publisher topic_name="asd">
            ...  <!--QoS settings here will override the ones in `default`. The settings specified in `default` will be used as a base.-->
        </publisher>
    </node>
<qos_profiles>
```

```yaml
/**:
    ros__qos_profiles:
        publisher:
            topic_name: /my_ns/nested_ns/asd
            qos:
                ...
        subscription:
            topic_name: /another_ns/asd
            qos:
                ...
/my_ns/nested_ns/my_node:
    ros__qos_profiles:
        publisher:
            topic_name: asd
            qos:
                ...
        subscription:
            topic_name: /another_ns/asd
            qos:
                ...
```

## Interaction with remapping and expansion

Topic, services and node names in the parameter file would be interpreted as the already remapped names. e.g.:

```bash
ros2run <package_name> <exec_name> --ros-args -r chatter:=my_chatter --qos-file /path/to/qos/file
```

The topic that will be looked up in the qos profile file is `my_chatter`, and not `chatter`.
Relatives topic/service names will be allowed used under a `node` tag, but it will not be allowed under the `default` tag.
In that case, they will be extended with the namespace specified in the parent `node` tag.

### Rationale

Parameter files works in the same way.
The usefulness of this kind of files is to allow the user to configure a set of nodes easily for a particular use case.
Applying remapping rules to the names in the file will only add complexity and be more confusing.

## rcl API


The proposed API aims to decouple the object that represents the loaded `QoS` profile file from the loader.

### rcl_qos_loaded_profiles_t

Structure that represents the loaded qos profiles file.

```c
/// Default initializer
rcl_qos_loaded_profiles_t
rcl_get_default_initialized_qos_loaded_profiles();

/// Gets qos for a publisher
rcl_ret_t
rcl_qos_loaded_profiles_get_publisher_qos(
    const rcl_qos_loaded_profiles_t * lp,
    const char * node_name,
    const char * node_ns,
    const char * topic_name,
    rmw_qos_profile_t * qos);

/// Gets qos for a publisher with a qos profile id
rcl_ret_t
rcl_qos_loaded_profiles_get_publisher_qos_with_id(
    const rcl_qos_loaded_profiles_t * lp,
    const char * node_name,
    const char * node_ns,
    const char * topic_name,
    const char * qos_profile_id,  // This could be an `uint` too, though having a readable name is probably better.
    rmw_qos_profile_t * qos);

/// Similar functions for subscription/clients/services
```

### Loader

Function to load the qos profiles file:

```c
rcl_ret_t
rcl_qos_load_profiles_from_xml_file(const char * file_path, rcl_qos_loaded_profiles_t * lp);
```

There could be different functions for different formats.

### Loaded profiles builder

Infrastructure to easily build a `rcl_qos_loaded_profiles_t` object.
The intent of this API is to decouple and actual parser (e.g. XML), from how a `rcl_qos_loaded_profiles_t` is built.

```c
/// Default initializer
rcl_qos_profiles_builder_t
rcl_get_default_initialized_qos_profiles_builder();

/// Set qos for publisher
rcl_ret_t
rcl_qos_profiles_builder_set_publisher_qos(
    rcl_qos_loaded_profiles_builder_t * builder,
    const char * node_name,
    const char * node_ns,
    const char * topic_name,
    const rmw_qos_profile_t * qos);

/// Set qos for publisher with profile id
rcl_ret_t
rcl_qos_profiles_builder_set_publisher_qos_with_id(
    rcl_qos_profiles_builder_t * builder,
    const char * node_name,
    const char * node_ns,
    const char * qos_profile_id,
    const rmw_qos_profile_t * qos);

/// Similar methods for subscriptions/clients/services
...

/// Build a `rcl_qos_loaded_profiles_t` object
rcl_ret_t
rcl_qos_profiles_builder_get_loaded_profiles(rcl_qos_profiles_builder_t * builder, rcl_qos_loaded_profiles_t * lp);
```

How to parse a `rmw_qos_profile_t` from the qos file is up to the specific loader.

### Usage in init options and node options

A `rcl_qos_loaded_profiles_t` member could be added in both `rcl_init_options_t` and `rcl_node_options_t`.
The one in init options will be common to all nodes in the same contexts, and can be overridden by the one in node options.

Optionally, we could also add in `rcl_node_options_t` a boolean to indicate to use the `rcl_qos_loaded_profiles_t` in init options if the profile was not found in the node specific loaded profiles object.

### New ros argument

A new argument will be needed to allow the user to specify which file to load.

```bash
ros2run <package_name> <exec_name> --ros-args --qos-file /path/to/qos/file
```

This loading mechanism will be equivalent to setting the `rcl_qos_loaded_profiles_t` member in init options.

## Further extensions

### Loading qos profiles in composable nodes

The interface definition of the [LoadNode](https://github.com/ros2/rcl_interfaces/blob/master/composition_interfaces/srv/LoadNode.srv) service could be extended with a field to specify the desired `QoS` of the loaded node.

In that way, the user loading the node is responsible of passing the desired QoS, instead of the node container.
The message to be sent could be loaded from a qos profile file.

#### Proposed interfaces

```
# EndpointQoS.msg

string name  # This can be either a topic or service name
uint8 type  # Endpoint type. An enum that can be a publisher/subscription/topic/service
QoS qos  # Message that defines the qos
```

```
# EndpointQoS.msg

string name  # This can be either a topic or service name
uint8 type  # Endpoint type. An enum that can be a publisher/subscription/topic/service
QoS qos  # Message that defines the qos
```

```
# composition_interfaces/srv/LoadNode.srv

# Current request fields
...
# New field
EndpointQoS[] requested_qos_profiles  # new field to allow loading user requested qos profiles
---
# Current response definition
...
```

A `rcl_qos_loaded_profiles_t` can be constructed from the received message and passed to the node options of the node to be loaded.

TBD: `QoS` message definition.

### Support in launch files

`launch_ros.actions.Node` and `launch_ros.actions.LoadComposableNode` actions could support a qos file argument in their constructors.

## File format

As showed in the above examples, both YAML or XML can correctly represent the desired file format.

XML advantages:
- Can be easily verified.
YAML advantages:
- It can be argued that the format is simpler and more readable than XML.
- The format proposed in the above examples can easily be integrated with the current parameter file format.
  That integration would lead in a unique "configuration file", that could be further extended in the future.
  For example, passing remapping rules in it.

## Explicitly allowing external configurability in entities

Both node and init options could have a flag to ignore the new `--qos-file` argument.
In a similar way, there could be an option for publishers/subscriptions/clients and services to allow side loading their qos or not.

In nodes designed to be reused, it does make sense to allow side-loading the qos settings of all entities.
To make this use case easier, the following options can be added:

- `rcl_init_options_t` is extended with a `bool allow_side_loading_qos`.
- The following enum is defined:
  ```c
  typedef enum rcl_side_load_qos_t {
      RCL_SIDE_LOAD_QOS_DEFAULT;
      RCL_SIDE_LOAD_QOS_ENABLED;
      RCL_SIDE_LOAD_QOS_DISABLED;
  } rcl_side_load_qos_t;
  ```
- A `rcl_side_load_qos_t allow_side_loading_qos` member is added to `rcl_node_options_t`, it will copy the behavior defined in the `rcl_init_options_t` of its parent context when the enum value is `RCL_SIDE_LOAD_QOS_DEFAULT`.
- In a similar way, publisher/subscription/client/service options will have a `rcl_side_load_qos_t allow_side_loading_qos` that will copy the behavior set in the node in case `RCL_SIDE_LOAD_QOS_DEFAULT` is used.

## Which QoS policies can be externally modified?

It might not make sense to be able to externally modify some of the `QoS` policies.
For example, modifying `rmw_qos_liveliness_policy_t` externally to `MANUAL_BY_TOPIC`.

### Analysis for each policy

#### History kind, history depth, durability, reliability, lifespan

These four policies provide an abstraction in a way that the code to publish or take a message from a publisher/subscription doesn't have to be changed if the history kind or history depth changed.
Thus, it does not make sense to restrict the external configurability of them.

#### Deadline

If the node is not listening to "deadline missed" events for that endpoint, modifying it externally won't have any effect.
For nodes that are using this feature, allowing to externally configure it makes sense.

#### Liveliness and liveliness_lease_duration

Different values of `liveliness_lease_duration` always make sense, regardless of how the node is implemented.
In the case of the liveliness kind, `MANUAL_BY_TOPIC` policy does not make sense if the author of the node doesn't thought about it.

### Proposed solution

Do not allow externally loading the `liveliness` qos policy, and allow loading all the others.
Authors that support both manual by topic and automatic liveliness can provide a parameter or argument to configure it.

### Alternatives

There could be a callback mechanism, in which the node's author validates that the `QoS` profile that is going to be applied is valid.
