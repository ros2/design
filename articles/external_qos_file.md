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

ROS 2 added the concept of [Quality of Service (QoS)](qos.md) settings to publishers, subscriptions, clients and services.
As of ROS 2 Foxy Fitzroy, QoS settings can only be specified in code.
To avoid recompiling the source code with patched QoS, node implementers have used different mechanisms to configure QoS profiles:
- Command line arguments
- ROS parameters
- Combining system default QoS with vendor specific QoS profiles.

Instead of having different ways of externally specifying QoS for the different entities, it would be good to have a standardized way to select QoS settings.

## QoS file format

DDS uses the concept of QoS profiles, in which each profile has a name.
When creating an entity (e.g. a `DataWriter`), the passed profile name will be looked up in the QoS profiles file being used, and the QoS can be loaded in that way rather than from hard-coded values in the source code.

If ROS would have QoS profiles names, like in DDS, collisions between those names in nodes written by different authors could happen.
Thus, QoS profiles names would need the full power of remapping and expansion.

This proposal uses the opposite approach, identifying an entity leveraging node name uniqueness <sup id="back_node_name_uniqueness">[1](#to_node_name_uniqueness)</sup> and through the use of topic/service names.

<b id="to_node_name_uniqueness">1</b> Node name uniqueness is not being enforced up to Foxy, though it is assumed. [↩](#back_node_name_uniqueness)

### Nodes

QoS settings apply to entities that are associated with ROS nodes.
Thus, a top level tag to identify the node should be available in the file format.
Here are a couple of examples:

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

### Entity type

It should be possible to distinguish between the different entity types in a QoS profile file.
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

### QoS profiles IDs (optional)

In the case that a node wants to create two `publishers` on the same topic with different QoS, the above format wouldn't allow to identify uniquely the `publisher`. The same applies to other kinds of entitys.

To solve this issue, the format could support using a profile ID, in addition to the topic name:

```xml
<qos_profiles>
    <node name="my_node" namespace="/my_ns/nested_ns">
        <publisher topic_name="asd" profile_id="1">
            ...
        </publisher>
        <publisher topic_name="asd" profile_id="2">
            ...
        </publisher>
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
        publisher:
            topic_name: asd
            profile_id: 2
            qos:
                ...
```

`rcl` API would need to be extended to support profiles id.
Users should only use this mechanism to avoid collisions.

### Default profiles for entities in different nodes

It is common to have several nodes creating the same entity, and the same qos profile is desired in all of them.
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

## Named QoS profiles

The file can also include named QoS profiles, that can be referenced within the same file:

```xml
<qos_profiles>
    <profiles>
        <qos name="reliable_depth_100">
            <reliability>reliable</reliability>
            <history_depth>100</history_depth>
            <history>keep_last</history>
        </qos>
    </profiles>
    <node name="my_node" namespace="/my_ns/nested_ns">
        <publisher topic_name="asd">
            <qos base="reliable_depth_100"/>  <!--Uses exactly reliable_depth_100 profile-->
        </publisher>
        <publisher topic_name="bsd">
            <qos base="reliable_depth_100">  <!--Uses reliable_depth_100 profile as a base, and overrides the durability policy-->
                <lifespan>10s</lifespan>
            </qos>
        </publisher>
    </node>
<qos_profiles>
```

```yaml
/**:
    ros__qos_profiles:
        profiles:
            reliable_depth_100:
                reliability: reliable
                history_depth: 100
                history: keep_last
/my_ns/nested_ns/my_node:
    ros__qos_profiles:
        publisher:
            topic_name: asd
            qos:
                base: reliable_depth_100
        publisher:
            topic_name: bsd
            qos:
                base: reliable_depth_100
                lifespan: 10s
```

This mechanism will help ensuring matching QoS profile, where that's needed in the system.

Having some implicitly defined named profiles can become handy to easily write profiles.
The following could be implicitly defined:

- ros_default: Equivalent to `rmw_qos_profile_default`.
- ros_sensor_data: Equivalent to `rmw_qos_profile_sensor_data`.
- ros_service_default: Equivalent to `rmw_qos_profile_services_default` (as of ROS 2 Foxy, the same as `ros_default`).
- ros_system_default: Equivalent to `rmw_qos_profile_system_default`.

See [qos_profiles.h](https://github.com/ros2/rmw/blob/master/rmw/include/rmw/qos_profiles.h).

## Implicit base profile

It might happen that a QoS profile is not fully specified and no base profile was chosen, in that case there will be an implicit QoS profile.
If there was a profile defined in the `<default>` section for the same topic or the same service, that one will be used:

```xml
<qos_profiles>
    <default>
        <publisher topic_name="/my_ns/nested_ns/asd">
            <reliability>reliable</reliability>
            <history_depth>100</history_depth>
        </publisher>
    </default>
    <node name="my_node" namespace="/my_ns/nested_ns">
        <publisher topic_name="asd">
            <history_depth>1000</history_depth>  <!--Overrides the history depth of 100 defined above.-->
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
                reliability: reliable
                history_depth: 100
/my_ns/nested_ns/my_node:
    ros__qos_profiles:
        publisher:
            topic_name: asd
            qos:
                history_depth: 1000
```

If that doesn't happen, `rmw_qos_profile_default` is used as a base.

### Rationale

There are other options to handle a non fully specified qos profile:

- Reject those qos profiles, and force users to specify all policies.
- Use the original qos profile specified in code as a base profile.

The first option would be extremely verbose, and it doesn't make much sense to explicitly set some QoS settings like `lifespan` when you don't care about them.
The last option is possible, but it would be hard to tell what was the original profile, and code would have been to be carefully checked.

## Interaction with remapping and expansion

Topic, services and node names in the parameter file would be interpreted as the already remapped names. e.g.:

```bash
ros2 run <package_name> <exec_name> --ros-args -r chatter:=my_chatter --qos-file /path/to/qos/file
```

The topic that will be looked up in the qos profile file is `my_chatter`, and not `chatter`.
Relative topic/service names will be allowed used under a `node` tag, but it will not be allowed under the `default` tag.
In that case, they will be extended with the namespace specified in the parent `node` tag.

### Rationale

Parameter files works in the same way.
The usefulness of this kind of files is to allow the user to configure a set of nodes easily for a particular use case.
Applying remapping rules to the names in the file will only add complexity and be more confusing.

## rcl API


The proposed API aims to decouple the object that represents the loaded QoS profile file from the loader.

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
ros2 run <package_name> <exec_name> --ros-args --qos-file /path/to/qos/file
```

This loading mechanism will be equivalent to setting the `rcl_qos_loaded_profiles_t` member in init options.

## Further extensions

### Loading qos profiles in composable nodes

The interface definition of the [LoadNode](https://github.com/ros2/rcl_interfaces/blob/master/composition_interfaces/srv/LoadNode.srv) service could be extended with a field to specify the desired QoS of the loaded node.

In that way, the user loading the node is responsible of passing the desired QoS, instead of the node container.
The message to be sent could be loaded from a qos profile file.

#### Proposed interfaces

```
# EntityQoS.msg

string name  # This can be either a topic or service name
uint8 type  # Entity type. An enum that can be a publisher/subscription/topic/service
QoS qos  # Message that defines the qos
```

```
# composition_interfaces/srv/LoadNode.srv

# Current request fields
...
# New field
EntityQoS[] requested_qos_profiles  # new field to allow loading user requested qos profiles
---
# Current response definition
...
```

A `rcl_qos_loaded_profiles_t` can be constructed from the received message and passed to the node options of the node to be loaded.

TBD: QoS message definition.

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

The node options could have a flag to ignore the new `--qos-file` argument.
In a similar way, there could be an option for publishers/subscriptions/clients and services to allow loading the QoS from an external source (side load).

In nodes designed to be reused, it does make sense to allow side-loading the qos settings of all entities.
To make this use case easier, the following options can be added:

- `rcl_node_options_t` is extended with a `bool allow_side_loading_qos`.
- The following enum is defined:
  ```c
  typedef enum rcl_side_load_qos_t {
      RCL_SIDE_LOAD_QOS_DEFAULT;
      RCL_SIDE_LOAD_QOS_ENABLED;
      RCL_SIDE_LOAD_QOS_DISABLED;
  } rcl_side_load_qos_t;
  ```
- A `rcl_side_load_qos_t allow_side_loading_qos` member is added to publisher/subscription/client/service options, it will copy the behavior defined in the `rcl_node_options_t` of its parent node when the enum value is `RCL_SIDE_LOAD_QOS_DEFAULT`.

## Which QoS policies can be externally modified?

It might not make sense to be able to externally modify some of the QoS policies.
For example, modifying `rmw_qos_liveliness_policy_t` externally to `MANUAL_BY_TOPIC`.

### Analysis for each policy

#### History kind, history depth, durability, reliability, lifespan

These four policies provide an abstraction in a way that the code to publish or take a message from a publisher/subscription doesn't have to be changed if the history kind or history depth changed.
Thus, it does not make sense to restrict the external configurability of them.

#### Deadline

If the node is not listening to "deadline missed" events for that entity, modifying it externally won't have any effect.
For nodes that are using this feature, allowing to externally configure it makes sense.

#### Liveliness and liveliness_lease_duration

Different values of `liveliness_lease_duration` always make sense, regardless of how the node is implemented.
In the case of the liveliness kind, `MANUAL_BY_TOPIC` policy does not make sense if the author of the node doesn't thought about it.

### Proposed solution

Do not allow externally loading the `liveliness` qos policy, and allow loading all the others.
Authors that support both manual by topic and automatic liveliness can provide a parameter or argument to configure it.

### QoS verification callbacks

The node author can, optionally, add a callback to verify that the side loaded QoS is valid for that node.
e.g.: the node author might want to ensure non-lossy QoS.

## Alternatives

Instead of a parameter file, we could use a well known set of parameters to specify the QoS profiles.
For example:

```yaml
/my_ns/my_node:
    ros__parameters:
        publisher./my/topic.reliability: reliable
        publisher./my/topic.history_depth: 100
/**:
    ros__parameters:
        client./my/service.reliability: reliable
```

There are some disadvantages of this approach:

- Parameters can only be scalar values or uniform lists, and none of those alternatives can represent completely a QoS profile.
  Thus, multiple parameters are needed to define a single profile, making the definition of them extremely verbose.
- A profile cannot be defined and reused, like in the proposed parameter file format.
  Thus, when profiles of different entities are required to match, it's easier to make a mistake.
- Parameters can be modified. We would need parameter callbacks rejecting changes in any of the parameters that are intended to modify QoS profiles, so only parameter overrides can be used for them.

QoS settings could also be specified using command line arguments.
The disadvantages are similar to making use of parameters for defining QoS profiles.

## References

- [rti documentation](https://community.rti.com/examples/using-qos-profiles)
- [DDS XML spec](https://www.omg.org/spec/DDS-XML/1.0/PDF)
