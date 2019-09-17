---
layout: default
title: Zero Copy via Loaned Messages
permalink: articles/zero_copy.html
abstract:
author: '[Karsten Knese](https://github.com/karsten1987) [William Woodall](https://github.com/wjwwood) [Michael Carroll](https://github.com/mjcarroll)'
published: false
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

## Overview

There is a need to eliminate unnecessary copies throughout the ROS2 stack to maximize performance and determinism.
In order to eliminate copies, the user must have more advanced control over memory management in the client library and middleware.
One method of eliminating copies is via message loaning, that is the middleware can loan messages that are populated by the end user.
This document outlines desired changes in the middleware and client libraries required to support message loaning.

## Motivation

The motivation for message loaning is to increase performance and determinism in ROS2.

As additional motivation, there are specific middle implementations that allow for zero-copy via shared memory mechanisms.
These enhancements would allow ROS2 to take advantage of the shared memory mechanisms exposed by these implementations.
An example of zero-copy transfer is [RTI Connext DDS Micro](https://community.rti.com/static/documentation/connext-micro/3.0.0/doc/html/usersmanual/zerocopy.html)

### Publication Use Cases

There are two primary kinds of use cases when publishing data which are relevant in this article:

1. The user creates and owns an instance of a message which they which to publish and then reuse after publishing.
2. The user borrows a message instance from the middleware, copies data into the message, and returns the ownership of the message during publish.

Currently, only the first case is possible with the `rclcpp` API.
After calling publish, the user still owns the message and may reuse it immedeately.

In order to support the second use case, we need a way for the user to get at least a single message from the middleware, which they may then populate, then return when publishing.

In the second case, the memory that is used for the loaned message should be optionally provided by the middleware.
For example, the middleware may use this opportunity to use a preallocated pool of message instances, or it may return instances of messages allocated in shared memory.
However, if the middleware does not have special memory handling or preallocations, it may refuse to loan memory to the client library, and in that case, a user provided allocator should be used.
This allows the user to have control over the allocation of the message when the middleware would otherwise use the standard allocator (new/malloc)

#### Additional Publication Use Case

One additional publishing use case is allowing the user to loan the message to the middleware during asynchronous publishing.
This use case comes up when all or part of the data being published is located in memory that the middleware cannot allocate from, e.g. a hardware buffer via memory mapped I/O or something similar, and the user wants to still have zero copy and asynchronous publishing.
In this case, the user needs to call publish, then keep the message instance immutable until the middleware lets the user know that it is done with the loaned data.

This is a narrow use case and will require additional interfaces to support, therefore it will be out of scope for this document, but it is mentioned for completion.

### Subscription Use Cases

There are two ways the users may take message instances from a subscription when data is available:

1. Taking direction from the `Subscription` after polling it for data availability or waiting via a wait set.
2. Using an `Executor`, which takes the data from the user and delivers it via a user-defined callback.

Note: It is assumed that the user will be abe to take multiple messages at a time if they are available.

In the first case, the user could choose to either:

* Manage the memory for the message instance themselves, providing a reference to it, into which the middleware should fill the data.
* Take one or more loaned messages from the middleware and return the loans later.

In the second case, the user is delegating the memory management ot he client library via the `Executor`.The `Executor` may or may not borrow data from the middleweare, but the user callback does not care, so this can be considered an implementation detail.
The user should be able to influence what the `Executor` does, and in the case that memory needs to be allocated, the user should be able to provide an allocator or memory management strategy which would influence the `Executor`'s behavior.

## Requirements


## Design Proposal
