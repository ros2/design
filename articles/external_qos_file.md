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
</qos_profiles>
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
</qos_profiles>
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

In the case that a node wants to create two `publishers` on the same topic with different QoS, the above format wouldn't allow to uniquely identify the `publisher`. The same applies to other kinds of entitys.

To solve this issue, the format could support using a profile ID, in addition to the topic name:

```xml
<qos_profiles>
    <node name="my_node" namespace="/my_ns/nested_ns">
        <publisher topic_name="asd" profile_id="my_pub_profile_1">
            ...
        </publisher>
        <publisher topic_name="asd" profile_id="my_pub_profile_2">
            ...
        </publisher>
    </node>
</qos_profiles>
```

```yaml
/my_ns/nested_ns/my_node:
    ros__qos_profiles:
        publisher:
            topic_name: asd
            profile_id: my_pub_profile_1
            qos:
                ...
        publisher:
            topic_name: asd
            profile_id: my_pub_profile_2
            qos:
                ...
```

`rcl` API would need to be extended to support profiles IDs.
Users should only use this mechanism to avoid collisions.

### Default profiles for entities in different nodes

It is common to have several nodes creating the same entity, and the same QoS profile is desired in all of them.
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
</qos_profiles>
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
</qos_profiles>
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

See [qos_profiles.h](https://github.com/ros2/rmw/blob/0176872b10ff502ae34fa3eef5e900145f18b0b3/rmw/include/rmw/qos_profiles.h).

## Implicit base profile

It might happen that a QoS profile is not fully specified and no base profile was chosen, in that case there will be an implicit QoS profile.
If there was a profile defined in the `<default>` section for the same topic or the same service, that one will be used:

```xml
<qos_profiles>
    <default>
        <publisher topic_name="/my_ns/nested_ns/asd">
            <qos>
                <reliability>reliable</reliability>
                <history_depth>100</history_depth>
            </qos>
        </publisher>
    </default>
    <node name="my_node" namespace="/my_ns/nested_ns">
        <publisher topic_name="asd">
            <history_depth>1000</history_depth>  <!--Overrides the history depth of 100 defined above.-->
        </publisher>
    </node>
</qos_profiles>
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

If no base profile is specified and no default profile exists for an entity, then the QoS settings set by the node's author are used as a base.
For topics in the `default` section or for named QoS profiles, `rmw_qos_profile_default` is used as the implicit base.

Note: We could make the `base` attribute mandatory for profiles in the `default` or `profiles` section.

## Interaction with remapping and expansion

Topic, services and node names in the parameter file would be interpreted as the already remapped names. e.g.:

```bash
ros2 run <package_name> <exec_name> --ros-args -r chatter:=my_chatter --qos-file /path/to/qos/file
```

The topic that will be looked up in the QoS profile file is `my_chatter`, and not `chatter`.
Relative topic/service names will be allowed used under a `node` tag, but it will not be allowed under the `default` tag.
In that case, they will be extended with the namespace specified in the parent `node` tag.

### Rationale

- Parameter files works in the same way.
- The usefulness of this kind of files is to allow the user to configure a set of nodes easily for a particular use case.
- Applying remapping rules to the names in the file will only add complexity and be more confusing.

## rcl API

The proposed API aims to decouple the object that represents the loaded QoS profile file from the loader.

### rcl_qos_loaded_profiles_t

Structure that represents the loaded QoS profiles file.

```c
/// Default initializer
rcl_qos_loaded_profiles_t
rcl_get_default_initialized_qos_loaded_profiles();

/// Gets qos for a publisher
// The qos argument is "inout", it's original value is used as a default.
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

How to parse a `rmw_qos_profile_t` from the QoS file is up to the specific loader.

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

### Loading QoS profiles in composable nodes

The interface definition of the [LoadNode](https://github.com/ros2/rcl_interfaces/blob/master/composition_interfaces/srv/LoadNode.srv) service could be extended with a field to specify the desired QoS of the loaded node.

In that way, the user loading the node is responsible of passing the desired QoS, instead of the node container.
The message to be sent could be loaded from a QoS profile file.

#### Proposed interfaces

```
# EntityQoS.msg

string name  # This can be either a topic or service name
string profile_id  # ID for disambiguate entities
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

### QoS profiles for actions

Currently, there is a collection of [5 QoS profiles](https://github.com/ros2/rcl/blob/af438bcca1ffb69714c0b225ba0f80aceab04906/rcl_action/include/rcl_action/action_client.h#L44-L52) to be set for creating an action, that will set the QoS profiles of the topics and services making up the action.
The file format should support actions, if not the user would have to use the topics and service names with the mangling `rcl` applies, which is a bit hard to remember.

```xml
<qos_profiles>
    <node name="my_node" namespace="/my_ns/nested_ns">
        <action_client name="my_action">
            <feedback>
                <qos>
                    <history_depth>100</history_depth>
                </qos>
            <feedback>
        </action_client>
    </node>
</qos_profiles>
```

```yaml
/my_ns/nested_ns/my_node:
    ros__qos_profiles:
        action_client:
            name: my_action
            feedback:
                qos:
                    history_depth: 100
```

If in the future we have a single object for defining action QoS, the format should be revisited.

### Wildcard matching

The `Implicit base profile` section proposes some sort of node name wildcard matching.
The proposed yaml format uses wildcard matching explicitly.

There could be topic/service name wildcard matching too, for example:

```xml
<qos_profiles>
    <node name="my_node" namespace="/my_ns/nested_ns">
        <publisher topic_name="/prefix/*">
            <history_depth>1000</history_depth>  <!--Overrides the history depth of 100 defined above.-->
        </publisher>
    </node>
</qos_profiles>
```

```yaml
/my_ns/nested_ns/my_node:
    ros__qos_profiles:
        publisher:
            topic_name: /prefix/*
            qos:
                history_depth: 1000
```

Proposed wildcard matching rules:

- Multiple tokens matcher (**).
  - /**: Matches any topic/service name (e.g.: `/asd/bsd` or `/asd`).
  - /**/suffix: Matches any topic with the same name, ignoring the namespace. `suffix` can be a single token.
- Single token matcher (*):
  - /my/custom/namespace/*: Matches any topic with the same namespace.

### Support in launch files

`launch_ros.actions.Node` and `launch_ros.actions.LoadComposableNode` actions could support a QoS file argument in their constructors.

### rmw implementation specific payload

There are currently QoS settings that are supported by DDS and not by ROS.
An user might want to make use of them, and they won't find an easy way to do that without exposing the QoS setting on ROS.

There are some cases where an user might want to use a vendor specific QoS extension, for example RTI allows defining [flow controllers](https://community.rti.com/static/documentation/connext-dds/5.2.0/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/Content/UsersManual/CreatingandConfigCustomFlowC.htm) in publishers.

Though a DDS vendor specific QoS profile file can be created by the user to set up these QoS settings, you can only make use of the default QoS profile defined in the file.
This is a consequence of not being able to match ROS entities to DDS QoS profile names.

This problem can be solved in two ways:
- Having conventions to match DDS QoS profiles names to ROS entities.
  This has been proposed in https://github.com/ros2/rmw_fastrtps/pull/335, and could work well for most use cases.
- Adding a rmw implementation specific "payload" that it's then used by the rmw implementation.
  In DDS based implementations, this payload can be just the profile name.

The first alternative is limited, as the proposed convention would use exactly the same profile in all publishers in a topic.
This convention can be modified to match node name, topic name and profile id, but that approach will be hard to understand.
The second approach leverages the new file format and is more powerful.

As an example of the second alternative:

```xml
<qos_profiles>
    <node name="my_node" namespace="/my_ns/nested_ns">
        <publisher topic_name="asd">
            <qos>
                <rmw_payload>
                    <profile_name>my_qos_profile_name</profile_name>
                </rmw_payload>
            </qos>
        </publisher>
    </node>
</qos_profiles>
```

```yaml
/my_ns/nested_ns/my_node:
    ros__qos_profiles:
        publisher:
            topic_name: asd
            rmw_payload:
                profile_name: my_qos_profile_name
```

For this to work, `rmw_qos_profile_t` will need to be extended with a `rmw_qos_profile_payload_t` member defined like:

```c
typedef struct rmw_qos_profile_payload_item_t {
  char * key;
  char * value;
} rmw_qos_profile_payload_item_t;

typedef struct rmw_qos_profile_payload_t {
  rmw_qos_profile_payload_item_t * key_value_pairs;
} rmw_qos_profile_payload_t;
```

It would be up to the implementation what to do with the passed key value pairs.

Note:
  The proposed API is limited in that it treats all value as strings instead of taking advantage of natural representations supported by some markup languages.

## File format

As showed in the above examples, both YAML or XML can correctly represent the desired file format.

XML advantages:
- Can be easily verified.

YAML advantages:
- It can be argued that the format is simpler and more readable than XML.
- The format proposed in the above examples can easily be integrated with the current parameter file format.
  That integration would lead in a unique "configuration file", that could be further extended in the future.
  For example, passing remapping rules in it.

### Author's opinion

Using yaml sounds preferred.
libyaml is already a dependency of `rcl`, and the proposed format can be easily merged with the existing parameter file format if that's desired in the future.

Though through the document examples were showed both in XML and YAML, supporting both doesn't sound like a good idea.

## Explicitly allowing external configurability in entities

Nodes should have an option to enable having their QoS settings configured externally.
This gives authors the ability to define the allowable range of behavior for their nodes.
Furthermore, the option to externally configure QoS settings should be disabled by default (ie. node authors must opt-in).
The rationale for an opt-in policy is to force node authors to have to think about the implications of allowing various QoS changes to the node entities.

In nodes designed to be reused, it does generally make sense to allow overriding the qos settings of all (or most) sub-entities.
To make this use case easier, the following options can be added:

- `rcl_node_options_t` is extended with a `bool enable_qos_overrides`.
- The following enum is defined:
  ```c
  typedef enum rcl_enable_qos_overrides_t {
      RCL_ENABLE_QOS_OVERRIDES_DEFAULT;
      RCL_ENABLE_QOS_OVERRIDES_ENABLED;
      RCL_ENABLE_QOS_OVERRIDES_DISABLED;
  } rcl_enable_qos_overrides_t;
  ```
- A `rcl_enable_qos_overrides_t enable_qos_overrides` member is added to publisher/subscription/client/service options, it will copy the behavior defined in the `rcl_node_options_t` of its parent node when the enum value is `RCL_ENABLE_QOS_OVERRIDES_DISABLED`.

In this way the author can easily opt-in the feature in a Node, and opt-out it in the sub-entities where is not desired.

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
In the case of the liveliness kind, `MANUAL_BY_TOPIC`, it does not make sense if the node author hasn't considered it.

### QoS verification callbacks

The node author can, optionally, add a callback to verify that the QoS override is valid for that node.
e.g.: the node author might want to ensure non-lossy QoS.

The format could support changing the liveliness QoS too.
Node's author supporting different kinds of liveliness could check in the callback what liveliness kind was set, to then act accordingly.
The default callback will reject QoS profiles with a liveliness different to the one specified in code.

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
