---
layout: default
title: Proposal for Implementation of Intraprocess Communications in ROS 2
permalink: articles/intraprocess_communications.html
abstract: Proposal for improvements in the implementation of intraprocess communications in ROS2.
published: true
author: Michael Carroll
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}


## Requirements and Implementation of Intraprocess Communications



## Publisher/Subscription types

This section outlines a comparison of various potential methods of publishing and subscribing to ROS 2 messages.

* unique_ptr Publisher - The publisher gives up ownership when the `publish` method is called.
* shared_ptr Publisher - The publisher retains ownership when the `publish` method is called (and can reuse the message structure).
* unique_ptr Subscription - The subscriber is guaranteed to have ownership of the message when the callback is fired.  This variant of the subscriber would get the message in a form that can be written to.
* shared_ptr Subscription - The subscriber receives an object with shared ownership.  This variant of the subscriber would get a read-only copy of the message and acts similarly to the ROS1 nodelet API.

The type of subscriptions and publications fall in two categories:

* intraprocess - messages are sent from a publication to subscription via in-process memory
  * Currently, messages are sent via the intraprocess manager, which also sends a message via the middleware to enforce QoS or history settings.
  * Alternatively, there is a concept of "direct dispatch" where the `publish` method will invoke the subscription callback directly.
* interprocess - messages are serialized and sent via the underlying middleware.

## Characteristics of Publishers/Subscriptions

Various publisher/subscription types can be evaluated on several criteria:

* Publisher latency: The amount of time it takes for the `publish` call to be invoked
* Publisher copies: The minimum number of copies required for the `publish` call to execute.
* Subscription latency: The amount of time it takes for the `subscribe` call to be invoked
* Subscription copies: The minimum number of copies required for the `subscription` call to execute.

## Current Mechanism

In the current mechanism (`IntraProcessManager`), a singleton instance of the manager object is connected to an `rclcpp::Context` that is shared among all nodes that are to perform intraprocess communications.

When the user publishes a message, the message is stored in the `IntraProcessManager`'s internal ring buffer to then be dispatched to all intraprocess subscribers of the topic.
Currently, the `IntraProcessManager` will only accept a `std::unique_ptr` to a message instance.

### Publishing with only interprocess comms:

1. User calls `Publisher::publish(std::shared_ptr)` or `Publisher::publish(const std::shared_ptr)`
2. `Publisher::publish(std::shared_ptr)` calls `Publisher::do_inter_process_publish(const MessageT* msg)`
3. `Publisher::do_inter_process_publish` calls `rcl_publish`

* The message ownership goes back to the caller at the end of execution.
* No memory is allocated or deallocated in `rclcpp` or `rcl`

1. User calls `Publisher::publish(std::unique_ptr)`
2. `Publisher::publish(std::unique_ptr)` calls `Publisher::do_inter_process_publish(const MessageT* msg)`
3. `Publisher::do_inter_process_publish` calls `rcl_publish`

* The message pointer is reset at the end of execution and the memory is freed.
* No additional memory is allocated in `rclcpp` or `rcl`

### Publishing with only intraprocess comms

1. User calls `Publisher::publish(std::shared_ptr)` or `Publisher::publish(const std::shared_ptr)`
2. `Publisher::publish(std::shared_ptr)` creates a `std::unique_ptr` copy of the message
3. `Publisher::publish(std::shared_ptr)` calls `Publisher::publish(std::unique_ptr)`
4. `Publisher::publish(std::unique_ptr)` calls `store_intra_process_message_(MessageT*)`
5. `store_intra_process_message_(MessageT*)` stores the pointer in the ring buffer.

* When publishing a `shared_ptr` an additional copy of the message must be made
* When publishing a `unique_ptr` the message pointer is reset at the end of execution and the memory is freed.

### Publishing with both interprocess and intraprocess comms

1. User calls `Publisher::publish(std::shared_ptr)` or `Publisher::publish(const std::shared_ptr)`
2. `Publisher::publish(std::shared_ptr)` creates a `std::unique_ptr` copy of the message
3. `Publisher::publish(std::shared_ptr)` calls `Publisher::publish(std::unique_ptr)`
4. `Publisher::publish(std::unique_ptr)` calls `Publisher::do_inter_process_publish(cosnt MessageT* msg)`
5. `Publisher::do_inter_process_publish` calls `rcl_publish`
  * Execution waits for interprocess publications to finish
5. `Publisher::publish(std::unique_ptr)` calls `store_intra_process_message_(MessageT*)`
6. `store_intra_process_message_(MessageT*)` stores the pointer in the ring buffer.

* When publishing a `shared_ptr` an additional copy of the message must be made
* When publishing a `unique_ptr` the message pointer is reset at the end of execution and the memory is freed.
* All interprocess publication must finish before intraprocess communication executes.

### Subscribing with intraprocess comms

* Regardless of subscription signature, all subscriptions but one will receive copies of the message.


## Possible alternatives

### Storing `shared_ptr` in the IntraProcessManager

Rather than creating a copy of a message when a user publishes a `std::shared_ptr` or a `std::unique_ptr`, the `IntraProcessManager` could store a `shared_ptr`.
In the case of shared inter/intra-process comms, this would eliminate the need for an additional copy, as well as eliminate the need to wait for the interprocess publication to be done before intraprocess comms can begin.
Additionally, if subscriptions are taking `const std::shared_ptr` (much like nodelet), this can also eliminate the copies on the subscription side.
If a subscription signiture remains `std::unique_ptr`, then a copy would likely have to be made at the time the callback is fired.

