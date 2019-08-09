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

## Introduction

### What is a `Node`?

In ROS, a `Node` is an entity used to group other entities.
For example: `Publishers`, `Subscriptions`, `Services`, `Clients`.
`Nodes` ease organization and code reuse, as they can be composed and launched in different ways.

### What is a `Domain Participant`?

A `Domain Participant` is a type of DDS entity.
`Participants` also group other entities, like `Publishers`, `Subscribers`, `Data Writters`, `Data Readers`, etc.
But participants do more than that:

- Each `Participant` does discovery by its own.
  Creating more than one `Participant` increases cpu usage and network IO load.
- Each `Participant` keeps track of other `Domain Participants` and DDS entities.
  Using more than one will duplicate that data.
- Each `Participant` may create multiple threads for event handling, discovery, etc.
  The number of threads created per participant depend on the DDS vendor (e.g.: [connext](https://community.rti.com/best-practices/create-few-domainparticipants-possible)).

For those reasons, `Participants` are heavyweight.

### Current status

There is a one-to-one mapping between `Nodes` and `DDS Participants`.
This simplified the design, as `DDS Participants` provide the same organization that a `Node` needs.
The drawback of this approach, is that performance is deteriorated.
Furthermore, the maximum number of `Domain participants` is rather small.
For example, [RTI connext](https://community.rti.com/kb/what-maximum-number-participants-domain) is limited to 120 participants per domain.

## Proposal

The goal of this proposal is to improve overall performance by avoiding the creation of one `Domain Participant` per `Node`.
API changes will be avoided, if possible.

### What is a participant mapped to?

There are two main alternatives:
- One participant per process.
- One participant per context.

The second approach allows more flexibility.
Considering that by default there's only one context per process, it won't lower the performance.
Moreover, a mechanism for re-using the same participant in two separete contexts could be added.

### What is a Node now?

There's no lightweight DDS equivalent of a ROS `Node`, so these must be implemented on top of it.
A `Node` should be able to:
- Create other entities as `Publishers`, `Subscriptions`, `Services` and `Clients`.
  `Nodes` should own those entities, that is to say, those entity shouldn't outlive a `Node`.
- List all its entities.

For all the entities, it should be possible to get the `Node` that created them.
Each `Participant` should store all the information needed about its nodes, and communicate it other `Participants`.

### How `Node` information is communicated?

#### Using a topic

The name of all the available `Nodes`, and its `Publishers`, `Subscriptions`, `Services`, `Clients` should be available for every `Participant`.
This information can be communicated using a `topic`.
That topic will be an implementation detail and hidden to the user (i.e.: the `rt/` prefix won't be added to this `DDS topic`).

A message could be sent for:
- Each `Node`
- Each `Participant`

The second option reduces the amount of messages.
It also allow organizing the data using the `Participant` GUID as the key.
It's not possible to organize the data using the `Node` name as a key, because it can collide.
`Node` name uniqueness can be enforced using a collision resolution mechanism, but it can be detected beforehand (i.e.: this information will be needed by the resolution mechanism).
In the following, the second option will be considered.

##### State Message

Each `Participant` will send a message representing their state.
A keyed topic could be used for communicating it.
The `Participant` GUID can be used as the key.
This helps for keeping only one message per `Participant` in the history (see `QoS for communicating node information`).
The rest of the message will be a sequence of with the information of each node.
That message should contain the `Node` name, and four sequences:
- GUID of its `Publishers`
- GUID of its `Subscriptions`
- GUID of its `Services`
- GUID of its `Clients`

Vector bounds: TBD

##### QoS for communicating node information

Each published message should be available to late `Subscribers`, and only the last message of each key should be kept.
For that reason, the QoS of the `Publishers` should be:

- Durability: Transient Local
- History: Keep Last
- History depth: 1
- Reliability: Reliable

Considering that a keyed topic will be used, in which the history depth apply for each key, only one `Publisher` per process will be needed.
In that case, `unregister_instance` can be used for delete that key from the history (see [RTI Managing Data Instances](https://community.rti.com/static/documentation/connext-dds/5.2.3/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/Content/UsersManual/Managing_Data_Instances__Working_with_Ke.htm)).


The configuration of the `Subscriber` QoS depends on how the data will be accessed later:
- Polled using `Subscriber` read method when needed.
- Listened and organized in a local cache.

The second option allows better organization of this information (e.g.: in hash tables).
In the first case, the QoS of the `Subscriber` should be:

- Durability: Transient Local
- History: Keep Last
- History depth: 1
- Reliability: Reliable

In the second case, durability can be changed to `Volatile`.

#### Using USER_DATA and GROUP_DATA QoSPolicy

Each `Participant` could have in its user data the list of node names that owns.
When this data is changed, each `ParticipantListener` will be notified.
This is not a good option, as `UserData` is just a sequence of bytes.
Organizing a complex message in it won't be easy nor performant.

Similarly to `UserData`, `GroupData` is a available in `Publishers` and `Subscribers`.
These entities only need to communicate the GUID of the `Participant` and the `Node` name from which it was created.
This idea can be combined with a topic just publishing the list of `Node` names, without including all the other vectors in the message.
Although, it is more difficult to communicate this information for `Services` and `Clients`, as they use behind the scenes just a `DDS Publisher` and `Subscriber`.

### Other implications

#### Security

In `DDS`, security can be specified at a `Participant` level.
If one `Node` is mapped to one `Participant`, individual configuration of its security key and access control policy is possible.
From a security point of view, only being able to configure it at a `Participant` (or per process) level is enough.
There's not much sense on having different access control policies for `Nodes` in the same process.
As they share the same address space, other vulnerabilities are possible.

##### How to create a new security key?

Before, we were creating a key for each `Node`.
The full name of the node was used for creating it.

If we create one `Participant` per context, we will only need a key for each of them, and not one per `Node`.
There are two alternatives:

- Add the concept of `Context` name (or `Participant` name).
  In this way, the key of each participant could be specified independently.
- Use one key per process.
  All the `Participants` within one process will use the same key (is that possible?).

##### How to specify access policies?

Access control policies could still be specified per `Node` basis.
When a `Participant` is created, it should look at the access control policies of each of its `Nodes` and compose them in a single configuration.

QQ:
- Is it possible to add more access control policies after creating the `Participant` (e.g.: When later creating a `Node`).

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

For emulating this behavior, messages could be ignored by checking from what `Node` the `Publisher` was created.
This should be possible by querying the state messages, or the local chache were they are organized.


#### Intra process communication

Currently, intra-process communication can be enable disabled in each `Publisher` and `Subscription`.
There is only one reason for that: intraprocess communication doesn't support all QoS policies.

Inter process messages from `Publishers` that can also communicate with a `Subscription` using the intra process layer are ignored before handling the callback.
The same problem will happen when having only one `Participant` per context, and it can be solved in the same fashion.

If in the future our intra process communication support all the QoS policies, we could forbid the possibility of enabling and dissabling it at `Node`, `Publisher`, `Subscription` level.
Configuring intra process communication with `Context` granularity should be enough.

#### Launching rclpy nodes

In `Dashing` and before, a container for dinamically composing `rclpy Nodes` is not available.
If this is not added, launching multiple `rclpy Nodes` in a launch file will create multiple participants.
That will make the performance worse, compared with composing `rclcpp Nodes`.
A `rclpy` component container should be added to solve the problem.
A generic container can also be considered, allowing to dinamically load `Nodes` from both clients.
