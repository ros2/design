---
layout: default
title: Node to Participant mapping
permalink: articles/Node_to_Participant_mapping.html
abstract: This article analyzes the performance implications of enforcing a one-to-one mapping between ROS Nodes and DDS Participants, and proposes alternative implementation approaches.
author: '[Ivan Paunovic](https://github.com/ivanpauno)'
published: true
categories: Middleware
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Background

### Node

In ROS, a Node is an entity used to group other entities.
For example: Publishers, Subscriptions, Servers, Clients.
Nodes ease organization and code reuse, as they can be composed in different ways.

### Domain Participant

A Participant is a type of DDS entity.
Participant also group other entities, like Publishers, Subscribers, Data Writters, Data Readers, etc.
Creating more Participants adds overhead to an application:

- Each Participant participates in discovery.
  Creating more than one Participant usually increases CPU usage and network IO load.
- Each Participant keeps track of other DDS entities.
  Using more than one within a single process may result in data duplication.
- Each Participant may create multiple threads for event handling, discovery, etc.
  The number of threads created per Participant depends on the DDS vendor (e.g.: [RTI Connext](https://community.rti.com/best-practices/create-few-domainParticipants-possible)).

For those reasons, a Participant is a heavyweight entity.

Note: This might actually depend on the DDS vendor, some of them share these resources between Participants (e.g. `OpenSplice`).
Many DDS vendors, however, do not perform this kind of optimization (e.g.: `RTI Connext` and `Fast-RTPS`), and actually recommend creating just one Participant per process.

### Context

In ROS, a Context is the non-global state of an init-shutdown cycle.
It also encapsulates shared state between Nodes and other entities.
In most applications, there is only one ROS Context in a process.

## Behavior pre-Foxy

There is a one-to-one mapping between Nodes and DDS Participants.
This simplified the original implementation, as DDS Participants provide many features equivalent to the ones of ROS Nodes.
The drawback of this approach is the overhead that comes with creating many Participants.
Furthermore, the maximum number of Domain Participants is rather small.
For example, in [RTI Connext](https://community.rti.com/kb/what-maximum-number-Participants-domain) it is limited to 120 Participants per Domain.

## Proposed approach

The goal of this proposal is to improve overall performance by avoiding the creation of one Participant per Node.
API changes will be avoided, if possible.

### Mapping of DDS Participant to a ROS entity

There are two alternatives, besides the one-to-one Node to Participant mapping used pre-Foxy:
- Using one Participant per process.
- Using one Participant per Context.

The second approach is much more flexible, allowing more than one Participant in a single application for those that need it e.g. domain bridge applications.
Thus, a one-to-one Participant to Context mapping was chosen.

When multiple Nodes are running in a single process, there are different options for grouping them by - ranging from a separate context for each Node, over grouping a few Nodes in the same context, to using a single context for all Nodes.
For most applications, only one Context is created.

### Discovery information

If a one-to-one Node to Participant mapping is not used, extra discovery information is needed to be able to match other entities to a Node e.g. Publishers, Subscriptions, etc.
Several approaches can be used to share this information.
The proposed approach uses a topic for it.
Each Participant publishes a message with all the information needed to match an entity to a Node.
The message structure is the following:

* ParticipantInfo
  * gid
  * NodeInfo
    * Node namespace
    * Node name
    * Reader gid
    * writed gid

When one entity is updated (e.g.: a Publisher is created or destroyed), a new message is sent.

Identification of Clients and Servers happens according to the ROS conventions for their topic names (see [	
Topic and Service name mapping to DDS](140_topic_and_service_name_mapping.md)).

This topic is considered an implementation detail, and not all `rmw` implementations have to use it.
Thus, all the necessary logic has to be in the rmw implementation itself or in an upstream package.
Implementing this logic in `rcl` would make it part of the API, and not an implementation detail.

To avoid code repetition, a common implementation of this logic is provided by the [rmw_dds_common](https://github.com/ros2/rmw_dds_common/) package.

#### Details of the ROS discovery topic

- topic name: `ros_discovery_info`
- Writer qos:
  - durability: transient local
  - history: keep last
  - history depth: 1
  - reliability: reliable
- Reader qos:
  - durability: transient local
  - history: keep all
  - history depth: 1
  - reliability: reliable

## Other implications

#### Security

Previously, each Node could have different security artifacts.
That was possible because each Node was mapped to one Participant.
The new approach allows to specify different security artifacts for each process.
For more details, see [ROS 2 Security Enclaves](ros2_security_enclaves.md).

#### Ignore local publications option

There is an `ignore_local_publications` option that can be set when [creating a Subscription](https://github.com/ros2/rmw/blob/2250b3eee645d90f9e9d6c96d71ce3aada9944f3/rmw/include/rmw/rmw.h#L517).
That option avoids receiving messages from Publishers within the same Node.
This wasn't implemented in all the rmw implementations (e.g.: [FastRTPS](https://github.com/ros2/rmw_fastrtps/blob/099f9eed9a0f581447405fbd877c6d3b15f1f26e/rmw_fastrtps_cpp/src/rmw_Subscription.cpp#L118)).

After this change, implementing this feature will be less direct.
Some extra logic needs to be added in order to identify from which Node a Publisher was created.

## Alternative implementations

### Using keyed topics

Keyed topics could be used, with the Participant gid as the key.
That would allow the Reader side of the topic to use a keep last, depth 1 history.

### Using Participant/Writer/Reader userData QoS

Instead of using a custom topic to share the extra ROS specific discovery information, a combination of Participant/Reader/Writer `userData` could be used:

- Participant `userData`: list of Nodes created by the Participant (updated when Node created destroyed).
- Reader user `userData`: Node name/namespace.
- Writer user `userData`: Node name/namespace.

That information would be enough to satisfy ROS required graph API.
This mechanism would also preclude having a topic that has to be read and written by all Nodes, which is better from a security perspective.

This alternative wasn't implemented because of lack of support in some of the currently supported DDS vendors.

## Further work

This optimization has been applied to `rmw_cyclonedds`, `rmw_fastrtps_cpp` and `rmw_fastrtps_dynamic_cpp`.
Other DDS based `rmw` implementations, like `rmw_connext_cpp` could use the same approach.
