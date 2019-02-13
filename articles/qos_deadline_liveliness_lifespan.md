---
layout: default
title: ROS QoS - Deadline, Liveliness, and Lifespan
permalink: articles/qos_deadline_liveliness_lifespan.html
abstract:
  This article makes the case for adding Deadline, Liveliness, and Lifespan QoS settings to ROS. It outlines the requirements and explores the ways it could be integrated with the existing code base. 
author: '[Nick Burek](https://github.com/nburek)'
published: true
categories: Middleware
date: February 13th 2019
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

Glossary:

- DDS - Data Distribution Service
- RTPS - Real-Time Publish Subscribe
- QoS - Quality of Service

## Existing ROS QoS Settings
While DDS provides many settings to enable fine-grained control over the Quality of Service (QOS) for entities, ROS only provides native support for a handful of them. ROS users are able to specify the History, Depth, Reliability, and Durability via a QoS configuration struct when they create Publishers, Subscribers, etc. 

This leaves a lot of QoS settings that can only be set if DDS vendor can load additional default settings via a configuration file. If a user wants to hook their code into these additional QoS settings then they would need to get a reference to the rmw implementation and program against the vendor specific API. Without the abstraction layer provided by ROS their code becomes less portable.

## Proposed Changes
As users start to build more robust applications, they are going to need more control over when data is being delivered and information about when the system is failing. 
To address these needs it was proposed that we start by adding support for the following new DDS QoS settings. 

### Proposed New QoS Settings
- Deadline - The deadline policy establishes a contract for how often publishers will receive data and how often subscribers expect to receive data. 
- Liveliness - The liveliness policy is useful for determining if entities on the network are "alive" and notifying interested parties when they are not. 
- Lifespan - The lifespan policy is useful for preventing the delivery of stale messages by specifying a time at which they should no longer be delivered. 

More detailed information on these policies can be found below in Appendix A. 

### DDS Vendor Support
Some of the primary DDS vendors that ROS supports already provide support for these QoS settings. However, the default vendor, Fast-RTPS, does not yet provide full support for these new QoS settings in their library. This does not have to be a blocker for adding support for these policy settings to ROS. We can start adding support for them and just log warning messages when a user attempts to specify an unsupported QoS setting. When support is eventually added for the policies in the vendor library then we can enable it in the rmw implementation. 

### Native ROS Support vs Vendor Configuration
It may be argued that many DDS QoS settings could be used with ROS by simply specifying them using the vendor specific configuration mechanism, such as the xml configuration files used by RTI Connext. This may be acceptable for some QoS policies that only impact the behavior of the underlying DDS middleware, but in the case where support for QoS policy impacts the behavior of the application code this is not good enough. 

For example, it doesn't do much good to specify deadline policy via the vendor configuration mechanism if your ROS application is never notified when a publisher violates the deadline. In such cases the application needs to be able to respond to these events in order to log failures, notify users, or adjust system behavior.

When interaction with the application is needed it is also preferable to have a ROS defined interface as opposed to having the application program directly against the DDS vendor API. Many nodes in ROS are written to be shared and reused by the community. Since different end users will want to use different middleware vendors for their applications, it is important to maintain that flexibility for nodes that want to take advantage of these additional QoS policies. 

**Based on this criteria, native ROS support should be provided for Deadline and Liveliness but not for the Lifespan policy.** While Deadline and Liveliness will both have events that the application will initiate and need to be informed of, the Lifespan policy can be entirely handled by the underlying DDS service without the need for any intervention by the application. 

### ROS Changes
These are the various changes that would be needed within ROS in order to natively support Deadline and Liveliness. 

#### Topic Status Event Handler
Both the Deadline and Liveliness policies generate events from the DDS service layer that the application will need to be informed of. For Deadlines, the subscriber receives event notifications if it doesn't receive anything within the deadline and the publisher receives event notifications if it doesn't publish anything within the deadline. For Liveliness, subscribers receive events when the publisher they're listening to is no longer alive. Both of these fall under a category of "Topic Status Events". 

To handle these notifications, a new callback function can be provided by the user that will be called any time a `TopicStatusEvent` occurs for a particular topic. It will receive as a parameter a `TopicStatusEvent` enum value that specifies what type of event took place and a timestamp of when the event took place. This callback function would be provided by the user's application when it calls the create function for publishers and subscribers. The constructors and create functions will be overloaded to make this new handler optional. 

The choice to use a callback function as opposed to another notification mechanism is to remain consistent with the existing ROS interface and not add a new method of waiting on ROS data/events. 

#### QoS Struct
Minimal changes will need to be made to the QoS struct that is passed into the creation functions for Topics, Services, and Actions. A couple new enum values will be added for the Deadline and Liveliness settings and then a couple integers will be added to specify the time values for these policies. 

#### Assert Liveliness Functions
Two new functions would need to be added that could be used by the application to explicitly assert liveliness. One function to assert liveliness at the Node level and one to assert it at the Topic level. While liveliness will also be implicitly assumed just based on sending messages, it will also be able to be explicitly declared by the application. These functions will need to be implemented in the rmw layer, the rcl layer, and the language specific client libraries. 

#### rcl_wait and rmw_wait
The rcl layer is currently using a WaitSet in order to be informed of events from the rmw layer, such as incoming messages. These WaitSets contain lists of several types of conditions, such as timers firing or subscriptions receiving data. In order to support new Topic Status Events, a new type will be added to the existing WaitSet and the rmw layer will set them when these events occur. 


## FAQ
- How does the Deadline policy take into account the additional overhead of ROS (such as deserialization) when determining if a deadline was missed? 
  - As a simplification it is not going to attempt to take into account any ROS overhead. A deadline will be considered missed if the rmw layer does not receive a message by the deadline and not if the user application on top of ROS does not receive it by the deadline. 


## Open Questions
- How do Deadlines and Liveliness impact Services and Actions? 


## Appendix A
Definitions of the QoS policies from the DDS spec. 

### Deadline
This policy is useful for cases where a Topic is expected to have each instance updated periodically. On the publishing side this
setting establishes a contract that the application must meet. On the subscribing side the setting establishes a minimum
requirement for the remote publishers that are expected to supply the data values.

When the Service ‘matches’ a DataWriter and a DataReader it checks whether the settings are compatible (i.e., offered
deadline period<= requested deadline period) if they are not, the two entities are informed (via the listener or condition
mechanism) of the incompatibility of the QoS settings and communication will not occur.

Assuming that the reader and writer ends have compatible settings, the fulfillment of this contract is monitored by the Service
and the application is informed of any violations by means of the proper listener or condition.

The value offered is considered compatible with the value requested if and only if the inequality “offered deadline period <=
requested deadline period” evaluates to ‘TRUE.’

The setting of the DEADLINE policy must be set consistently with that of the TIME_BASED_FILTER. For these two policies
to be consistent the settings must be such that “deadline period>= minimum_separation.”

### Liveliness
This policy controls the mechanism and parameters used by the Service to ensure that particular entities on the network are
still “alive.” The liveliness can also affect the ownership of a particular instance, as determined by the OWNERSHIP QoS
policy.

This policy has several settings to support both data-objects that are updated periodically as well as those that are changed
sporadically. It also allows customizing for different application requirements in terms of the kinds of failures that will be
detected by the liveliness mechanism.

The AUTOMATIC liveliness setting is most appropriate for applications that only need to detect failures at the process-
level 27 , but not application-logic failures within a process. The Service takes responsibility for renewing the leases at the
required rates and thus, as long as the local process where a DomainParticipant is running and the link connecting it to remote
participants remains connected, the entities within the DomainParticipant will be considered alive. This requires the lowest
overhead.

The MANUAL settings (MANUAL_BY_PARTICIPANT, MANUAL_BY_TOPIC), require the application on the publishing
side to periodically assert the liveliness before the lease expires to indicate the corresponding Entity is still alive. The action
can be explicit by calling the assert_liveliness operations, or implicit by writing some data.

The two possible manual settings control the granularity at which the application must assert liveliness.
• The setting MANUAL_BY_PARTICIPANT requires only that one Entity within the publisher is asserted to be alive to
deduce all other Entity objects within the same DomainParticipant are also alive.
• The setting MANUAL_BY_TOPIC requires that at least one instance within the DataWriter is asserted.

### Lifespan
The purpose of this QoS is to avoid delivering “stale” data to the application.

Each data sample written by the DataWriter has an associated ‘expiration time’ beyond which the data should not be delivered
to any application. Once the sample expires, the data will be removed from the DataReader caches as well as from the
transient and persistent information caches.

The ‘expiration time’ of each sample is computed by adding the duration specified by the LIFESPAN QoS to the source
timestamp. The source timestamp is either automatically computed by the Service
each time the DataWriter write operation is called, or else supplied by the application by means of the write_w_timestamp
operation.

This QoS relies on the sender and receiving applications having their clocks sufficiently synchronized. If this is not the case
and the Service can detect it, the DataReader is allowed to use the reception timestamp instead of the source timestamp in its
computation of the ‘expiration time.