---
layout: default
title: Node to Participant mapping
permalink: articles/node_to_participant_mapping.html
abstract: This article analyzes the performance implications of enforcings a one-to-one mapping between ROS nodes and DDS participants, and propose alternative approaches.
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
But participants do more than that:

- Each `Participant` participates in discovery.
  Creating more than one `Participant` increases cpu usage and network IO load.
- Each `Participant` keeps track of other `Domain Participants` and DDS entities.
  Using more than one will duplicate that data within a single process.
- Each `Participant` may create multiple threads for event handling, discovery, etc.
  The number of threads created per participant depend on the DDS vendor (e.g.: [connext](https://community.rti.com/best-practices/create-few-domainparticipants-possible)).

For those reasons, `Participants` are heavyweight.

Note: This might actually depend on the DDS implementation, some of them share these resources between `Participants` (e.g. OpenSplice).
Many `DDS` vendors don't do this (e.g.: `rti Connext` and `Fast-RTPS`), and they actually recommend creating just one `Participant` per process.

### `Context`

The `ROS Context` is the no-global state of an init-shutdown cycle.
It also encapsulates shared state between nodes and other entities.
In most applications, there is only one `ROS Context` in a process.

## Current status

There is a one-to-one mapping between `Nodes` and `DDS Participants`.
This simplified the design, as `DDS Participants` provide the same organization that a `Node` needs.
The drawback of this approach, is that with an increasing number of nodes the overhead also increases.
Furthermore, the maximum number of `Domain participants` is rather small.
For example, [RTI connext](https://community.rti.com/kb/what-maximum-number-participants-domain) is limited to 120 participants per domain.

## Proposal

The goal of this proposal is to improve overall performance by avoiding the creation of one `Domain Participant` per `Node`.
API changes will be avoided, if possible.

### Mapping of the `Participant` to a `ROS` entity

There are two main alternatives, besides the current mapping to a `Node`:
- Using one participant per process.
- Using one participant per context.

The second approach allows more flexibility.
Considering that by default there's only one context per process, it wouldn't affect the case where each node runs in its own process.
In the case where multiple nodes are running in a single process, we have different options for grouping them by - ranging from a separate context for each node, over grouping a few nodes in the same context, to using a single context for all nodes.

In any of both options, a `Node` stops being a real middleware node, and starts being just a collection of `ROS` entities.

### ROS specific discovery information

#### Using a topic

The name of all the available `Nodes`, and its `Publishers`, `Subscriptions`, `Services`, `Clients` should be available for every `Participant`.
This information can be communicated using a `topic`.
That topic will be an implementation detail and hidden to the user (i.e.: the `rt/` prefix won't be added to this `DDS topic`).

One message could be sent for each:
- `Node`
- `Participant`

The second option reduces the amount of messages.
It also allow organizing the data using the `Participant` GUID as the key.
It's not possible to organize the data using the `Node` name as a key, because it can collide.
`Node` name uniqueness can be enforced using a collision resolution mechanism, but it can't be detected beforehand.
In the following, the second option will be considered.

##### State Message

Each `Participant` will send a message representing their state.
A keyed topic could be used for communicating it.
The `Participant` GUID can be used as the key.
This helps for keeping only one message per `Participant` in the history (see [QoS for communicating node information](#QoS-for-communicating-node-information)).
The rest of the message will be a sequence with information for each node.
For each `Node`, the message should contain the `Node` name, and four sequences:
- GUID of its `Publishers`
- GUID of its `Subscriptions`
- GUID of its `Services`
- GUID of its `Clients`

Vector bounds: TBD

This state message is sent each time a new `ROS Entity` is created.
e.g.: A participant will updates its message when a new `Node` is created.

##### QoS for communicating node information

Each published message should be available to late `Subscribers`, and only the last message of each key should be kept.
For that reason, the QoS of the `Publishers` should be:

- Durability: Transient Local
- History: Keep Last
- History depth: 1
- Reliability: Reliable

If a keyed topic is used, in which the history depth apply for each key, only one `Publisher` per process will be needed.
The QoS of the `Subscriber` should be:

- Durability: Transient Local
- History: Keep Last
- History depth: 1
- Reliability: Reliable

In case keyed topics aren't used, `keep all` history should be used.

The subscriber could access data in two different ways:
- Polled and accessed using `Subscriber` read method when needed.
- Listened, accessed using subscriber take method and organized in a local cache.

The second option allows better organization of this information (e.g.: in hash tables).

#### Using USER_DATA and GROUP_DATA QoSPolicy

Each `Participant` could store in its user data, the list of node names that it owns.
When this data is changed, each `ParticipantListener` will be notified.
This is not a good option, as `UserData` is just a sequence of bytes.
Organizing a complex message in it won't be easy nor performant.

Similarly to `UserData`, `GroupData` is a available in `Publishers` and `Subscribers`.
These entities only need to communicate the GUID of the `Participant` and the `Node` name from which it was created.
This idea can be combined with a topic just publishing a list of `Node` names of a `Participant`.

Support for `GroupData` was not available in some of the `DDS-vendors` at the moment of the implementation.
For that reason this option was discarded.

### Implementation

The implementation can be done in two different ways:

- Implementing the discovery logic in `rcl`.
- Modifying rmw implementations without modifying rmw API (as long as possible).

The first approach have the following disadvantages:
- There is no `Node` concept in `rmw` layer, as `Node` discovery is solved in `rcl`.
  Actually, all the `Node` APIs in `rmw` will not longer make sense.
- Currently, no threads are created in the `rcl` layer.
  It will be needed in case `Node` discovery is done in this layer.
- It will force us to build the concept of `Node` on top of the underlying middleware, regardless if the middleware already has a lightweight entity similar to a `Node`.
- It will break API in many layers.


The second approach has the following disadvantages:
- Each RMW implementation has to reimplement node discovery logic.
  This can be avoided by arround creating a new common package that uses the abstractions in `rmw`.
  Each of the implementations that wants to use this should depend on this common package.

The second approach is preferred, as it is more flexible and it avoids breaking API in many layers.

### Other implications

#### Security

In `DDS`, security can be specified at a `Participant` level.
If one `Node` is mapped to one `Participant`, individual configuration of its security key and access control policy is possible.
From a security point of view, only being able to configure it at a `Participant` (or per process) level should be enough.
It does not make much sense to have different access control policies for `Nodes` in the same process.
As they share the same address space, other vulnerabilities are possible.

##### Security directory of each participant

Before, the environment variable `ROS_SECURITY_DIRECTORY` specified the root path of the keystore.
The security files for each participant were found using the node name from that root.

With this proposal, it won't be possible to find the security files from the node name, as the `Participant` will be associated with a `Context`.

A few alternatives are possible:
- Add a name to the `Context`, and use the same directory discovery logic.
- Just be able to pass a directory to each process. All the `Contexts` in a process will use the same security files.

The first alternative is more flexible, as it allows to specify different security files for `Contexts` in the same process.
That's particuarly useful for some use cases, e.g.: domain bridges.

The `Context` name will be available in ros2 graph API.
That will allow adapting the tool that generates the policy files from a running example.

The `Context` doesn't pretend to be unique, and it's just a way of specifying configurations.
Particularly, for specifying the security directory.
There will be a default context name, so a default security directory can be specified.
It should be possible to remap this name, to allow easy deployment of nodes.

##### Generating DDS permissions files from ROS policies files

Currently, ROS access control policy files allows specifying privilages to each `Node`.
From that file, the required DDS permission file is generated.

Considering this proposal, there are a few alternatives:
- Contexts are added to the policy file.
- A tool for generating a permission file from multiple ROS policy files is added.

In the first case, the `Context` will work as a way of grouping all the privilages of its nodes.
That also will work for `rmw` implementations where a `Node` can have separate policy files, in which case the context grouping will just be ignored.

A tool for combining policies files can be added, regardless if the policy file format is changed or not.

#### Node Name Uniqueness

In `Dashing` and before, `Node` name uniqueness is not enfornced.

When creating only one `Participant` per `Context`, we can distinguish two cases:
- There is an overlap between the name of two `Nodes` created within the same `Context`.
  This case can be trivially solved.
- There is a collision with the `Node` name created from another `Context`.
  By the nature of discovery, when a collision is detected, it's not possible to know what `Node` was created first without extra information.
  A collision resolution mechanism have to be decided for solving which `Node` continues living.
  A `timestamp` of the `Node` creation published in the state message can help to solve the problem.

If we don't change the `Node` to `Participant` mapping, the last item still stands and should be solved in a similar fashion.

#### Ignore local publications option

There's an `ignore_local_publications` option that can be set when [creating a subscription](https://github.com/ros2/rmw/blob/2250b3eee645d90f9e9d6c96d71ce3aada9944f3/rmw/include/rmw/rmw.h#L517).
That option avoids receiving messages from `Publishers` within the same `Node`.
This wasn't implemented in all the rmw implementations (e.g.: [FastRTPS](https://github.com/ros2/rmw_fastrtps/blob/099f9eed9a0f581447405fbd877c6d3b15f1f26e/rmw_fastrtps_cpp/src/rmw_subscription.cpp#L118)).

After this change, implementing this feature will be less direct.
Some extra logic needs to be added in order to identify from which `Node` a `Publisher` was created.


#### Intra process communication

Currently, intra-process communication can be enable disabled in each `Publisher` and `Subscription`.
An important reason for being able to selectively enable intra-process is that intraprocess communication doesn't support all QoS policies.

Inter process messages from `Publishers` that can also communicate with a `Subscription` using the intra process layer are ignored before handling the callback.
The same problem will happen when having only one `Participant` per context, and it can be solved in the same fashion.

If in the future our intra process communication support all the QoS policies, we could forbid the possibility of enabling and dissabling it at `Node`, `Publisher`, `Subscription` level.

#### Launching rclpy nodes

In `Dashing` and before, a container for dynamically composing `rclpy Nodes` is not available.
If this is not added, launching multiple `rclpy Nodes` in a launch file will create multiple participants.
That will make the performance worse, compared with composing `rclcpp Nodes`.
A `rclpy` component container should be added to solve the problem.
A generic container can also be considered, allowing to dynamically load `Nodes` from both clients.
