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
- Client - Refers to an application that connects to a ROS Service to send requests and receive responses.
- Owner - Refers to the application that is running a ROS Service that receives requests and sends responses.

## Existing ROS QoS Settings

While DDS provides many settings to enable fine-grained control over the Quality of Service (QoS) for entities, ROS only provides native support for a handful of them. ROS users are able to specify the History, Depth, Reliability, and Durability via a QoS configuration struct when they create Publishers, Subscribers, etc.

This leaves a lot of QoS settings that can only be set if DDS vendor can load additional default settings via a configuration file.
If a user wants to hook their code into these additional QoS settings then they would need to get a reference to the rmw implementation and program against the vendor specific API.
Without the abstraction layer provided by ROS their code becomes less portable.

## New QoS Settings

As users start to build more robust applications, they are going to need more control over when data is being delivered and information about when the system is failing.
To address these needs it was proposed that we start by adding support for the following new DDS QoS settings.

### Deadline

The deadline policy establishes a contract for the amount of time allowed between messages.
For Topic Subscribers it establishes the maximum amount of time allowed to pass between receiving messages.
For Topic Publishers it establishes the maximum amount of time allowed to pass between sending messages.
For Service Owners it establishes the maximum amount of time allowed to pass between receiving a request and when a response for that request is sent.
For Service Clients it establishes the maximum amount of time allowed to pass between sending a request and when a response for that request is received.

Topics and Services will support the following levels of deadlines.
- DEADLINE_DEFAULT - Use the ROS specified default for deadlines (which is DEADLINE_NONE).
- DEADLINE_NONE - No deadline will be offered or requested and events will not be generated for missed deadlines.
- DEADLINE_RMW - The rmw layer of ROS will track the deadline. For a Publisher or Subscriber this means that a deadline will be considered missed if the rmw layer has not received a message within the specified time. For a Service this means the time a request is started is marked when the request reaches the rmw layer and the time at which it is finished is when the response message reaches the rmw layer.

In order for a Subscriber to listen to a Publisher's Topic the deadline that they request must greater than the deadline set by the Publisher.
A Service Client will **not** be prevented from making a request to a Service Owner if the Owner provides a deadline greater than the deadline requested by the Client.

### Liveliness

The liveliness policy establishes a contract for how entities report that they are still alive. 
For Topic Subscribers it establishes the level of reporting that they require from the Topic entities that they are subscribed to. 
For Topic Publishers it establishes the level of reporting that they will provide to Subscribers that they are alive. 
For Service Owners it establishes both the level of reporting that they will provide to Clients and also the level of reporting that they require from Clients.
For Service Clients it establishes both the level of reporting that they require from Service Owners and the level of reporting that they will provide to the Owner.

Topics will support the following levels of liveliness.
- LIVELINESS_DEFAULT - Use the ROS specified default for liveliness (which is LIVELINESS_AUTOMATIC).
- LIVELINESS_AUTOMATIC - The signal that establishes a Topic is alive comes from the ROS rmw layer.
- LIVELINESS_MANUAL_NODE - The signal that establishes a Topic is alive is at the node level. Publishing a message on any outgoing channel on the node or an explicit signal from the application to assert liveliness on the node will mark all outgoing channels on the node as being alive.
- LIVELINESS_MANUAL_TOPIC - The signal that establishes a Topic is alive is at the Topic level. Only publishing a message on the Topic or an explicit signal from the application to assert liveliness on the Topic will mark the Topic as being alive.

Services will support the following levels of liveliness.
- LIVELINESS_DEFAULT - Use the ROS specified default for liveliness (which is LIVELINESS_AUTOMATIC).
- LIVELINESS_AUTOMATIC - The signal that establishes a Service Owner is alive comes from the ROS rmw layer.
- LIVELINESS_MANUAL_NODE - The signal that establishes a Service is alive is at the node level. A message on any outgoing channel on the node or an explicit signal from the application to assert liveliness on the node will mark all outgoing channels on the node as being alive.
- LIVELINESS_MANUAL_SERVICE - The signal that establishes a Service is alive is at the Service level. Only sending a response on the Service or an explicit signal from the application to assert liveliness on the Service will mark the Service as being alive.

In order for a Subscriber to listen to a Publisher's Topic the level of liveliness tracking they request must be equal or less verbose than the level of tracking provided by the Publisher and the time until considered not alive set by the Subscriber must be greater than the time set by the Publisher.

Service Owners and Clients will each specify two liveliness policies, one for the liveliness policy pertaining to the Owner and one pertaining to the Client.
In order for a Client to connect to an Owner to make a request the Client_Liveliness level requested by the Owner must be greater than the level provided by the Client and the Owner_Liveliness requested by the Client must be greater than the level provided by the Owner.

### Lifespan

The lifespan policy establishes a contract for how long a message remains valid.
For Topic Subscribers it establishes the length of time a message is considered valid, after which time it will not be received.
For Topic Publishers it establishes the length of time a message is considered valid, after which time it will be removed from the Topic history and no longer sent to Subscribers.
For Service Owners it establishes the length of time a request is considered valid, after which time the Owner will not receive and process the request.
For Service Clients it establishes the length of time a response is considered valid, after which time the Client will not accept the response and the request will timeout.

- LIFESPAN_DEFAULT - Use the ROS specified default for lifespan (which is LIFESPAN_NONE).
- LIFESPAN_NONE - Messages do not have a time at which they expire.
- LIFESPAN_ENABLED - Messages will have a lifespan enforced.

If a Service Owner receives a request before the lifespan expires it should finish processing that request even if the lifespan expires while the request is being processed.

### DDS QoS Relation

These new policies are all based on the DDS QoS policies, but they do not require a DDS in order for an rmw implementation to support them.
More detailed information on the DDS specifics of these policies can be found below in Appendix A.

The only new QoS setting that does not directly map to DDS is the deadline policy for Services.
While the deadline policy could map directly to the underlying Publisher and Subscriber like they do for Topics, that would tie the QoS policy to the DDS implementation instead of the generic Service definition that does not specify it be implemented using two Topics.
The definition as it applies to messages on two underlying topics is also less useful than the definition of deadline as it pertains to life of a request.

### ROS Changes

These are the various changes that would be needed within ROS in order to natively support Deadline and Liveliness.

#### Resource Status Event Handler

Both the Deadline and Liveliness policies generate events from the rmw layer that the application will need to be informed of.
For Deadlines, the Subscriber receives event notifications if it doesn't receive anything within the deadline and the Publisher receives event notifications if it doesn't publish anything within the deadline.
For Liveliness, Subscribers receive events when the Publisher they're listening to is no longer alive.
Services generate similar events when Clients and Owners violate the defined policies.
Both of these fall under a category of "Resource Status Events".

To handle these notifications, a new callback function can be provided by the user that will be called any time a `ResourceStatusEvent` occurs for a particular Topic or Service.
It will receive as a parameter a `ResourceStatusEvent` enum value that specifies what type of event took place and a timestamp of when the event took place.
This callback function would be provided by the user's application when it calls the create function for publishers and subscribers.
The constructors and create functions will be overloaded to make this new handler optional.

The choice to use a callback function as opposed to another notification mechanism is to remain consistent with the existing ROS interface and not add a new method of waiting on ROS data/events.

#### QoS Struct

In the current version of ROS there is a single QoS struct that is used to specify the QoS policy whenever a Publisher, Subscriber, Service Owner, and Client are created.
With these new QoS settings the struct diverges for Topic participants and Service participants because Service participants will need to specify the QoS behavior for both the Client and Owner.
Separating them into using two different struct definitions for Topics versus Services will prevent unused QoS policies that would only apply to one being available in the other.

The new QoS policy structs will have additional fields that use enums based on the values defined above to specify the desired QoS settings.
Each enum field instance in the struct will also have an associated number field added to the struct that represents a time value for the policy.

#### Assert Liveliness Functions

New functions will need to be added that can be used by the application to explicitly assert liveliness.
One function to assert liveliness at the Node level, one to assert it at the Topic level, and one to assert it at the Service level.
While liveliness will also be implicitly assumed just based on sending messages, these functions will be used by the application to explicitly declare resources are alive.
These functions will need to be implemented in every layer from the rmw up through the rcl and language specific client libraries, such as rclcpp.

#### rcl_wait and rmw_wait

The rcl layer is currently using a WaitSet in order to be informed of events from the rmw layer, such as incoming messages.
These WaitSets contain lists of several types of conditions, such as timers firing or subscriptions receiving data.
In order to support new Topic Status Events, a new type will be added to the existing WaitSet and the rmw layer will set them when these events occur.

#### rcl_take_status and rmw_take_status

New functions called rcl_take_status and rmw_take_status will need to be added that can directly query the status for a Topic/Service.
It will operate in a similar manner to the rcl_take and rmw_take functions that are used to retrieve messages for a subscription.
It will be used by the executors when they receive a notice via the waitset mentioned above that a resource has a new status event available.

## RMW Vendor Support

All of these new QoS policies will need to be supported in the rmw implementations.
As we add new QoS policies it is likely that not all rmw vendors will provide support for every QoS policy that ROS defines.
This is especially true for non-DDS based implementations that do not already have native support for these features.
Because of this we need a way for the application to know if the rmw vendor that is being used supports the specified policies at the specified levels.

In the case where an rmw vendor does not support a requested QoS they can do one of two things.
They could fail with an exception that notes that an unsupported QoS policy was requested or they can choose to continue operating with a reduced QoS policy.
In the case where a vendor chooses to continue operating with a reduced policy it must trigger a status event for the resource that can be handled by the new callback outlined above.
This provides an application a way of being notified that even though their resource is operating, it is doing so with a reduced QoS.

## FAQ

- How does the Deadline policy take into account the additional overhead of ROS (such as deserialization) when determining if a deadline was missed?
  - As a simplification it is not going to attempt to take into account any ROS overhead. A deadline will be considered missed if the rmw layer does not receive a message by the deadline and not if the user application on top of ROS does not receive it by the deadline. A new deadline policy could be added later that takes this into account.
- What happens if multiple policy violations occur between the invocation of the callback? 
  - This is dependent on the rmw implementation. If the rmw layer supports tracking multiple status events then it can return them in subsequent calls to rmw_take_status. If it does not support tracking multiple events at once then it may just return one of the events, such as the first or the last to occur in the sequence.
- How do these QoS policies impact Actions?
  - The existing Actions interface exposes that Actions are implemented with multiple Topics and it exposes the QoS settings for those topics. It is therefore up to the application implementing the action to specify the QoS policies for the different Topics. A new interface will need to be added to allow Actions to specify a ResourceStatusHandler, but that will be added in a later design review.
- Why do Services not enforce deadline policies on connection like Publishers and Subscribers?
  - This is a simplification being made for initial implementation. It could be changed later to enforce it, but that would require additional complexity in the existing DDS based implementations of the rmw to disallow connections based on non-DDS QoS settings.

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