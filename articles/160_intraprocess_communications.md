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
