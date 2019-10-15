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

As additional motivation, there are specific middleware implementations that allow for zero-copy via shared memory mechanisms.
These enhancements would allow ROS2 to take advantage of the shared memory mechanisms exposed by these implementations.
An example of zero-copy transfer is [RTI Connext DDS Micro](https://community.rti.com/static/documentation/connext-micro/3.0.0/doc/html/usersmanual/zerocopy.html).

### Publisher Use Cases

There are two primary kinds of use cases when publishing data which are relevant in this article:

1. The user creates and owns an instance of a message which they wish to publish and then reuse after publishing.
2. The user borrows a message instance from the middleware, copies data into the message, and returns the ownership of the message during publish.

Currently, only the first case is possible with the `rclcpp` API.
After calling publish, the user still owns the message and may reuse it immediately.

In order to support the second use case, we need a way for the user to get at least a single message from the middleware, which they may then populate, then return when publishing.

In the second case, the memory that is used for the loaned message should be optionally provided by the middleware.
For example, the middleware may use this opportunity to use a preallocated pool of message instances, or it may return instances of messages allocated in shared memory.
However, if the middleware does not have special memory handling or pre-allocations, it may refuse to loan memory to the client library, and in that case, a user provided allocator should be used.
This allows the user to have control over the allocation of the message when the middleware might otherwise use the standard allocator (new/malloc).

#### Additional Publisher Use Case

One additional publishing use case is allowing the user to loan the message to the middleware during asynchronous publishing.
This use case comes up when all or part of the data being published is located in memory that the middleware cannot allocate from, e.g. a hardware buffer via memory mapped I/O or something similar, and the user wants to still have zero copy and asynchronous publishing.
In this case, the user needs to call publish, then keep the message instance immutable until the middleware lets the user know that it is done with the loaned data.

This is a narrow use case and will require additional interfaces to support, therefore it will be out of scope for this document, but it is mentioned for completion.

### Subscription Use Cases

There are two ways the users may take message instances from a subscription when data is available:

1. Taking direction from the `Subscription` after polling it for data availability or waiting via a wait set.
2. Using an `Executor`, which takes the data from the user and delivers it via a user-defined callback.

Note: It is assumed that the user will be able to take multiple messages at a time if they are available.

In the first case, the user could choose to either:

* Manage the memory for the message instance themselves, providing a reference to it, into which the middleware should fill the data.
* Take one or more loaned messages from the middleware and return the loans later.

In the second case, the user is delegating the memory management of the client library via the `Executor`.The `Executor` may or may not borrow data from the middleware, but the user callback does not care, so this can be considered an implementation detail.
The user should be able to influence what the `Executor` does, and in the case that memory needs to be allocated, the user should be able to provide an allocator or memory management strategy which would influence the `Executor`'s behavior.

## Requirements

Based on the use cases above, the general requirements are as follows:

* Users must be able to avoid all memory operations and copies in at least one configuration.


### Publisher Requirements

* The user must be able to publish from messages allocated in their stack or heap.
* The user must be able to get a loaned message, use it, and return it during publication.
* When taking a loan, the middleware should not do anything "special", that is that the user must be able to influence the allocation.

### Subscription Requirements

The following requirements should hold whether the user is polling or using a wait set.

* The user must be able to have the middleware fill data into messages allocated in the user's stack or heap.
* The user must be able to get a loaned message from the middleware when calling take.
* The user must be able to get a sequence of loaned messages from the middleware when calling take.
* The loaned message or sequence must be returned by the user.
* When taking a loan, the middleware should not do anything "special", that is that the user must be able to influence the allocation.

### Special Requirements

These requirements are driven by idiosyncrasies of various middleware implementations and some of their special operating modes, e.g. zero copy:

#### RTI Connext DDS

* The Connext API (more generally the DDS API) requires that the user to use a sequence of messages when taking or reading.
  * This means the ROS API needs to do the same, otherwise the middleware would be giving a "loan" to a message in a sequence, but it would also need to keep the sequence immutable.
* Needs to keep sample and sample info together, therefore `rclcpp` loaned message sequence needs to own both somehow.

Connext Micro Specific (ZeroCopy):

* The user needs to be able to check if data has been invalidated after reading it.

### Nice to Haves

* When using loaned messages, the overhead should be minimized, so that it can be used as the general approach as often as possible.
  * Versus reusing a user owned message on the stack or heap.
  * We will still recommend reusing a message repeatedly from the stack or heap of the user.
* Memory operations and copies should be avoided anywhere possible.

## Design Proposal

### RMW API

Introduce APIs for creating/destroying loaned messages, as well as structure for management:

```
void *
rmw_borrow_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message
);

rmw_ret_t
rmw_return_loaned_message(
  const rmw_publisher_t * publisher,
  void * loaned_message
);
```

Extend publisher API for loaned messages:

```
rmw_ret_t
rmw_publish_loaned_message(
  const rmw_publisher_t * publisher
  const void * ros_message
  rmw_publisher_allocation_t * allocation
);
```

Extend subscription API for taking loaned message.
Because a subscription does not necessarily allocate new memory for the loaned message during a take, the message only needs to be released rather than returned.

```
rmw_ret_t
rmw_take_loaned_message(
  const rmw_subscription_t * subscription
  void ** loaned_message,
  book * taken,
  rmw_subscription_allocation_t * allocation
);

rmw_ret_t
rmw_release_loaned_message(
  const rmw_subscription_t * subscription,
  void * loaned_message
);
```

### RCL API

Introduce APIs for creating/destroying loaned messages, as well as structure for management:

```
void *
rcl_borrow_loaned_message(
  const rcl_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message
);

rcl_ret_t
rcl_return_loaned_message(
  const rcl_publisher_t * publisher,
  rcl_loaned_message_t * loaned_message
);
```

Extend publisher API for loaned messages:

```
rcl_ret_t
rcl_publish_loaned_message(
  const rcl_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation
);

bool
rcl_publisher_can_loan_messages(const rcl_publisher_t * publisher);
```

Extend subscription API for taking loaned message:

```
rcl_ret_t
rcl_take_loaned_message(
  const rcl_subscription_t * subscription,
  void ** loaned_message,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation
);

rcl_ret_t
rcl_release_loaned_message(
  const rcl_subscription_t * subscription,
  void * loaned_message
);

bool
rcl_subscription_can_loan_messages(const rcl_subscription_t * subscription);
```

### RCLCPP LoanedMessage

In order to support loaned messages in `rclcpp`, we tntroduce the concept of a `LoanedMessage`.
A `LoanedMessage` provides a wrapper around the underlying loan mechanisms, and manages the loan's lifecycle.

```
template <class MsgT, typename Alloc = std::allocator<void>>
class LoanedMessage
{
public:
  // Get the underlying message
  MsgT& get()

  // Check if underlying message is valid and consistent
  bool is_consistent();

  // Loan message from the middleware, if loaning is not available,
  // allocate memory using Alloc
  LoanedMessage(
    const rclcpp::Publisher<MsgT, Alloc> * pub);

private:
  const rclcpp::Publisher<MsgT, Alloc> * pub_;

  // Holds details of the message loan
  rcl_loaned_message_t * loaned_msg_;

  // Will be initialized with memory from loaned_msg_, otherwise allocated.
  std::unique_ptr<MsgT> msg_;
};
```

### RCLCPP Publisher

Extend RCLCPP API:

```
// Return a loaned message from the middleware
rclcpp::LoanedMessage<MsgT, Alloc>
rclcpp::Publisher::loan_message()

// Publish a loaned message, returning the loan
void
rclcpp::Publisher::publish(std::unique_ptr<rclcpp::LoanedMessage<MsgT, Alloc>> loaned_msg);

// Test if the middleware supports loaning messages
bool
rclcpp::Publisher::can_loan_messages()
```

### RCLCPP Subscription

```
/// Test if the middleware supports loaning messages
bool
rclcpp::Subscription::can_loan_messages()

/// Subscription path for middlewares supporting loaned messages.
void
rclcpp::Subscription::handle_loaned_message(void * loaned_message, const rmw_message_info_t & message_info)
```

