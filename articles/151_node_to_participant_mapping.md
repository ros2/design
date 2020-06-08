---
layout: default
title: Node to Participant mapping
permalink: articles/node_to_participant_mapping.html
abstract: This article analyzes the performance implications of enforcings a one-to-one mapping between ROS nodes and DDS participants, and proposes alternative approaches.
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

### `Node`

In ROS, a `Node` is an entity used to group other entities.
For example: `Publishers`, `Subscriptions`, `Services`, `Clients`.
`Nodes` ease organization and code reuse, as they can be composed in different ways.

### `Domain Participant`

A `Domain Participant` is a type of DDS entity.
`Participants` also group other entities, like `Publishers`, `Subscribers`, `Data Writters`, `Data Readers`, etc.
Creating more `Participants` adds overhead to an application:

- Each `Participant` participates in discovery.
  Creating more than one `Participant` increases cpu usage and network IO load.
- Each `Participant` keeps track of other `Domain Participants` and DDS entities.
  Using more than one will duplicate that data within a single process.
- Each `Participant` may create multiple threads for event handling, discovery, etc.
  The number of threads created per participant depend on the DDS vendor (e.g.: [connext](https://community.rti.com/best-practices/create-few-domainparticipants-possible)).

For those reasons, `Participants` are a heavyweight entity.

Note: This might actually depend on the DDS implementation, some of them share these resources between `Participants` (e.g. OpenSplice).
Many `DDS` vendors don't do this (e.g.: `rti Connext` and `Fast-RTPS`), and they actually recommend creating just one `Participant` per process.

### `Context`

In ROS, a `Context` is the no-global state of an init-shutdown cycle.
It also encapsulates shared state between nodes and other entities.
In most applications, there is only one `ROS Context` in a process.

## Current status

There is a one-to-one mapping between `Nodes` and `DDS Participants`.
This simplified the original implementation, as `DDS Participants` provide many features equivalent to the ones of a `ROS Node`.
The drawback of this approach is the overhead originated by creating many participants.
Furthermore, the maximum number of `Domain participants` is rather small.
For example, [RTI connext](https://community.rti.com/kb/what-maximum-number-participants-domain) is limited to 120 participants per domain.

## Proposed approach

The goal of this proposal is to improve overall performance by avoiding the creation of one `Domain Participant` per `Node`.
API changes will be avoided, if possible.

### Mapping of `DDS Participant` to a `ROS` entity

There are two alternatives, besides the current `Node` to `Participant` mapping:
- Using one participant per process.
- Using one participant per context.

The second approach allows more flexibility, and it will allow creating more than one `Participant` for applications that need it --e.g. domain bridge applications--.
Thus, a one to one `Participant` to `Context` mapping was chosen.

In the case where multiple nodes are running in a single process, we have different options for grouping them by - ranging from a separate context for each node, over grouping a few nodes in the same context, to using a single context for all nodes.
For most applications, only one `Context` is created.

### Discovery information

When not using a one to one `Node`/`Participant` mapping, extra discovery information is needed to be able to match other entities to a `Node` --e.g. publishers, subscriptions, etc--.
Several approaches can be used to share this information, the proposed approach uses a topic for it.
Each `Participant` publishes a message with all the information needed to match an entity to a `Node`.
The message definition is the following:

* ParticipantInfo
  * gid
  * NodeInfo
    * node namespace
    * node name
    * reader gid
    * writed gid

When one entity is updated --e.g.: A `Publisher` is created or destroyed--, a new message is sent.

Identification of `Clients` and `Services` happens according to the ros conventions for their topic names (see [	
Topic and Service name mapping to DDS](140_topic_and_service_name_mapping.md)).

This topic is considered an implementation detail, and not all `rmw` implementations have to use it.
Thus, all the necessary logic has to be in the rmw implementation itself or in a downstream package.
Implementing this logic in `rcl` would make it part of the API, and not an implementation detail.

To avoid code repetition, a common implementation of the logic was done in [rmw_dds_common](https://github.com/ros2/rmw_dds_common/).

#### Details of the ros discovery topic

- topic name: `ros_discovery_info`
- writer qos:
  - durability: transient local
  - history: keep last
  - history depth: 1
  - reliability: reliable
- reader qos:
  - durability: transient local
  - history: keep all
  - history depth: 1
  - reliability: reliable

## Other implications

#### Security

Previously, each node could have different security artifacts.
That was possible because each node was mapped to one `Participant`.
The new approach allows to specify different security artifacts for each process.
For more details, see [ROS 2 Security Enclaves](ros2_security_enclaves.md).

#### Ignore local publications option

There's an `ignore_local_publications` option that can be set when [creating a subscription](https://github.com/ros2/rmw/blob/2250b3eee645d90f9e9d6c96d71ce3aada9944f3/rmw/include/rmw/rmw.h#L517).
That option avoids receiving messages from `Publishers` within the same `Node`.
This wasn't implemented in all the rmw implementations (e.g.: [FastRTPS](https://github.com/ros2/rmw_fastrtps/blob/099f9eed9a0f581447405fbd877c6d3b15f1f26e/rmw_fastrtps_cpp/src/rmw_subscription.cpp#L118)).

After this change, implementing this feature will be less direct.
Some extra logic needs to be added in order to identify from which `Node` a `Publisher` was created.

## Alternative implementations

### Using keyed topics

Keyed topics could be used, with the participant gid as the key.
That would allow the reader side of the topic to use a keep last, depth 1 history.

### Using participant/writer/reader user data

Instead of using a custom topic to share the extra ROS specific discovery information, a combination of participant/reader/writer user data could be used.
e.g.:

- Participant user data: list of nodes created by the participant (updated when node created destroyed).
- Reader user data: node name/namespace.
- Writer user data: node name/namespace.

That information would be enough to satisfy ros2 required graph API.
This mechanism will also avoid to have a topic where all nodes need to have write and read access.

This alternative wasn't implemented because of lack of support in some of the currently supported DDS vendors.
