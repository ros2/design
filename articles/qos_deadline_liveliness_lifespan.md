---
layout: default
title: ROS QoS - Deadline, Liveliness, and Lifespan
permalink: articles/qos_deadline_liveliness_lifespan.html
abstract:
  This article makes the case for adding Deadline, Liveliness, and Lifespan QoS settings to ROS. It outlines the requirements and explores the ways it could be integrated with the existing code base.
author: '[Nick Burek](https://github.com/nburek)'
date_written: 2019-09
last_modified: 2019-09
published: true
categories: Middleware
date: February 13th 2019
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Authors: {{ page.author }}

Date Written: {{ page.date_written }}

Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}

Glossary:

- DDS - Data Distribution Service
- RTPS - Real-Time Publish Subscribe
- QoS - Quality of Service
- Service Client - Also referred to as just Client, refers to an application that connects to a ROS Service to send requests and receive responses.
- Service Server - Also referred to as just Server, refers to the application that is running a ROS Service that receives requests and sends responses.

## Existing ROS QoS Settings

While DDS provides many settings to enable fine-grained control over the Quality of Service (QoS) for entities, ROS only provides native support for a handful of them.
ROS users are able to specify the History, Depth, Reliability, and Durability via a QoS configuration struct when they create Publishers, Subscribers, etc.

This leaves a lot of QoS settings that can only be set if DDS vendor can load additional default settings via a configuration file.
If a user wants to hook their code into these additional QoS settings then they would need to get a reference to the rmw implementation and program against the vendor specific API.
Without the abstraction layer provided by ROS their code becomes less portable.

## New QoS Settings

As users start to build more robust applications, they are going to need more control over when data is being delivered and information about when the system is failing.
To address these needs it was proposed that we start by adding support for the following new DDS QoS settings.

### Deadline

The deadline policy establishes a contract for the amount of time allowed between messages.
For Subscriptions it establishes the maximum amount of time allowed to pass between receiving messages.
For Publishers it establishes the maximum amount of time allowed to pass between sending messages.
Topics will support deadline tracking only up to the rmw layer.
This means that a deadline will be considered missed if the rmw layer has not received a message within the specified time.
In order for a Subscriber to listen to a Publisher's Topic the deadline that they request must be greater than, or equal to, the deadline set by the Publisher.
A deadline time of 0 will disable the deadline tracking.
The default deadline time will be 0.

### Liveliness

The liveliness policy establishes a contract for how entities report that they are still alive.
For Subscriptions it establishes the level of reporting that they require from the Publishers to which they are subscribed.
For Publishers it establishes the level of reporting that they will provide to Subscribers that they are alive.

Topics will support the following levels of liveliness:
- LIVELINESS_SYSTEM_DEFAULT - Use the ROS specified default for liveliness (which is LIVELINESS_AUTOMATIC).
- LIVELINESS_AUTOMATIC - The signal that establishes a Topic is alive comes from the ROS rmw layer.
- LIVELINESS_MANUAL_BY_NODE - The signal that establishes a Topic is alive is at the node level. Publishing a message on any outgoing channel on the node or an explicit signal from the application to assert liveliness on the node will mark all outgoing channels on the node as being alive.
- LIVELINESS_MANUAL_BY_TOPIC - The signal that establishes a Topic is alive is at the Topic level. Only publishing a message on the Topic or an explicit signal from the application to assert liveliness on the Topic will mark the Topic as being alive.

In order for a Subscriber to listen to a Publisher's Topic the level of liveliness tracking they request must be equal or less verbose than the level of tracking provided by the Publisher and the time until considered not alive set by the Subscriber must be greater than the time set by the Publisher.

### Lifespan

The lifespan policy establishes a contract for how long a message remains valid.
For Subscriptions it establishes the length of time a message is considered valid, after which time it will not be received.
For Publishers it establishes the length of time a message is considered valid, after which time it will be removed from the Topic history and no longer sent to Subscribers.
A lifespan time of 0 will disable the lifespan tracking.
The default lifespan time will be 0.

### DDS QoS Relation

These new policies are all based on the DDS QoS policies, but they do not require DDS in order for an rmw implementation to support them.
More detailed information on the DDS specifics of these policies can be found below in Appendix A.

### ROS Changes

These are the various changes that would be needed within ROS in order to natively support Deadline and Liveliness.

#### Resource Status Event Handler

Both the Deadline and Liveliness policies generate events from the rmw layer of which the application will need to be informed.
For Deadlines, the Subscriber receives event notifications if it doesn't receive anything within the deadline and the Publisher receives event notifications if it doesn't publish anything within the deadline.
For Liveliness, Subscribers receive events when there are no longer any Publishers alive to assert the topic is alive.
Services generate similar events when Clients and Servers violate the defined policies.
Both of these fall under a category of "Resource Status Events".

To handle these notifications, new callback functions can be provided by the user that will be called any time an event occurs for a particular Topic.
It will receive as a parameter a struct value that contains the information about the event such as when the event took place and other metadata related to the event.
These callback functions would be optionally provided by the user's application when it calls the create function for publishers and subscribers.
The constructors and create functions will be overloaded to make this new handler optional.

The status event handlers will not be called once for every status event.
Instead, the event handlers will only be called if there is an status change event that has not yet been handled when the Executor that services the callbacks checks.

#### QoS Struct

In the current version of ROS there is a single QoS struct that is used to specify the QoS policy whenever a Publisher and Subscriber are created.
With these new QoS settings the supported set of QoS policies for Topics and Services diverges.
Despite this, we are going to stick with a single struct for both Topics and Services instead of switching to two different struct types in order to keep the changes to a minimum and maintain as much backwards compatibility as possible in the client library interfaces.

The existing QoS policy struct will have new fields added to specify the desired QoS settings for Deadline, Liveliness, and Lifespan.
These new fields instances will be a combination of enum and time values.

#### Assert Liveliness Functions

New functions will need to be added that can be used by the application to explicitly assert liveliness.
One function to assert liveliness at the Node level and one to assert it at the Topic level.
While liveliness will also be implicitly assumed just based on sending messages, these functions will be used by the application to explicitly declare resources are alive.
These functions will need to be implemented in every layer from the rmw up through the rcl and language specific client libraries, such as rclcpp.

#### rcl_wait and rmw_wait

The rcl layer is currently using a WaitSet in order to be informed of events from the rmw layer, such as incoming messages.
These WaitSets contain lists of several types of conditions, such as timers firing or subscriptions receiving data.
In order to support new Topic Status Events, a new type will be added to the existing WaitSet and the rmw layer will set them when these events occur.

#### rcl_take_status and rmw_take_status

New functions called rcl_take_status and rmw_take_status will need to be added that can directly query the status for a Topic.
It will operate in a similar manner to the rcl_take and rmw_take functions that are used to retrieve messages for a subscription.
It will be used by the executors when they receive a notice via the waitset mentioned above that a resource has a new status event available.

## RMW Vendor Support
All of these new QoS policies will need to be supported in the rmw implementations.
As we add new QoS policies it is likely that not all rmw vendors will provide support for every QoS policy that ROS defines.
This is especially true for non-DDS based implementations that do not already have native support for these features.
Because of this we need a way for the application to know if the rmw vendor that is being used supports the specified policies at the specified levels.
In this case, the rmw vendor should fail the operation by returning an error code that specifies that the requested QoS policy is not supported.
The explicit failure will ensure the requesting application is receiving defined behavior and not operating under unexpected conditions.

In addition to an application being prevented from running with an unsupported policy, it is useful for an application to be able to query what QoS policies the rmw vendor supports.
The best way to do this is to provide an API that allows the application to check if a specific policy is supported and also get all the supported settings.
The design and implementation of this API is out of scope for this document and should be considered for part of the future work.

## FAQ

- How does the Deadline policy take into account the additional overhead of ROS (such as deserialization) when determining if a deadline was missed?
  - As a simplification it is not going to attempt to take into account any ROS overhead. A deadline will be considered missed if the rmw layer does not receive a message by the deadline and not if the user application on top of ROS does not receive it by the deadline. A new deadline policy could be added later that takes this into account.
- Why will the callback not get called for every status change event instead of potentially combining events of the same type?
  - Adding this functionality would require an additional buffer that would be used to store multiple events between servicing them. Additionally, the DDS API lends itself better to only being informed of the latest change and would require a realtime response to status change events so as to not miss a single event. This is not a one way door and we could change this later to allow buffering events without breaking backwards compatibility.
- How do these QoS policies impact Actions and Services?
  - The initial implementation does not support Actions and Services as there are more complex subtleties to how these concepts natively support these QoS features. In the future work section below we explore some ways that Services could implement these policies.
- How are these QoS policies affected by DDS topic instances?
  - While all of these policies can and will eventually support keyed instances, this document does not focus on the details of how as it is highly dependent on the design for ROS 2 to support keyed messages in general.

## Future Work

Actions and Services were considered out of scope for the initial implementation. Here we detail how Services could potentially support these QoS policies in the future.

### Deadline

For Service Servers it establishes the maximum amount of time allowed to pass between receiving a request and when a response for that request is sent.
For Service Clients it establishes the maximum amount of time allowed to pass between sending a request and when a response for that request is received.
Services will support deadline tracking only up to the rmw layer.
This means the time a request is started is marked when the request reaches the rmw layer and the time at which it is finished is when the response message reaches the rmw layer.
A Service Client will **not** be prevented from making a request to a Service Server if the  Server provides a deadline greater than the deadline requested by the Client.

### Liveliness

For Service Servers it establishes both the level of reporting that they will provide to Clients and also the level of reporting that they require from Clients.
For Service Clients it establishes both the level of reporting that they require from Service Servers and the level of reporting that they will provide to the Server.

Services will support the following levels of liveliness:
- LIVELINESS_DEFAULT - Use the ROS specified default for liveliness (which is LIVELINESS_AUTOMATIC).
- LIVELINESS_AUTOMATIC - The signal that establishes a Service Server is alive comes from the ROS rmw layer.
- LIVELINESS_MANUAL_NODE - The signal that establishes a Service is alive is at the node level. A message on any outgoing channel on the node or an explicit signal from the application to assert liveliness on the node will mark all outgoing channels on the node as being alive.
- LIVELINESS_MANUAL_SERVICE - The signal that establishes a Service is alive is at the Service level. Only sending a response on the Service or an explicit signal from the application to assert liveliness on the Service will mark the Service as being alive.

Service Servers and Clients will each specify two liveliness policies, one for the liveliness policy pertaining to the Server and one pertaining to the Client.
In order for a Client to connect to a Server to make a request the Client_Liveliness level requested by the Server must be greater than the level provided by the Client and the Server_Liveliness requested by the Client must be greater than the level provided by the Server.

### Lifespan

For Services it establishes a time at which the request is no longer valid.
It would act similar to a timeout on the request.
There are a lot of open questions about how you would notify Client and the Server if a request was timed out and what action should be taken when the lifespan expires.

## Appendix A

Definitions of the QoS policies from the DDS spec.

### Deadline

This policy is useful for cases where a Topic is expected to have each instance updated periodically.
On the publishing side this setting establishes a contract that the application must meet.
On the subscribing side the setting establishes a minimum requirement for the remote publishers that are expected to supply the data values.

When the DDS Service ‘matches’ a DataWriter and a DataReader it checks whether the settings are compatible (i.e., offered
deadline period<= requested deadline period) if they are not, the two entities are informed (via the listener or condition
mechanism) of the incompatibility of the QoS settings and communication will not occur.

Assuming that the 'DataReader' and 'DataWriter' ends have compatible settings, the fulfillment of this contract is monitored by the DDS Service
and the application is informed of any violations by means of the proper listener or condition.

The value offered is considered compatible with the value requested if and only if the inequality “offered deadline period <=
requested deadline period” evaluates to ‘TRUE.’

The setting of the DEADLINE policy must be set consistently with that of the TIME_BASED_FILTER. For these two policies
to be consistent the settings must be such that “deadline period>= minimum_separation.”

### Liveliness

This policy controls the mechanism and parameters used by the DDS Service to ensure that particular entities on the network are still “alive.”
The liveliness can also affect the ownership of a particular instance, as determined by the OWNERSHIP QoS policy.

This policy has several settings to support both data-objects that are updated periodically as well as those that are changed sporadically.
It also allows customizing for different application requirements in terms of the kinds of failures that will be detected by the liveliness mechanism.

The AUTOMATIC liveliness setting is most appropriate for applications that only need to detect failures at the process-
level 27 , but not application-logic failures within a process.
The DDS Service takes responsibility for renewing the leases at the required rates and thus, as long as the local process where a DomainParticipant is running and the link connecting it to remote participants remains connected, the entities within the DomainParticipant will be considered alive.
This requires the lowest overhead.

The MANUAL settings (MANUAL_BY_PARTICIPANT, MANUAL_BY_TOPIC), require the application on the publishing
side to periodically assert the liveliness before the lease expires to indicate the corresponding Entity is still alive.
The action can be explicit by calling the assert_liveliness operations, or implicit by writing some data.

The two possible manual settings control the granularity at which the application must assert liveliness.
• The setting MANUAL_BY_PARTICIPANT requires only that one Entity within the publisher is asserted to be alive to
deduce all other Entity objects within the same DomainParticipant are also alive.
• The setting MANUAL_BY_TOPIC requires that at least one instance within the DataWriter is asserted.

### Lifespan

The purpose of this QoS is to avoid delivering “stale” data to the application.

Each data sample written by the DataWriter has an associated ‘expiration time’ beyond which the data should not be delivered to any application.
Once the sample expires, the data will be removed from the DataReader caches as well as from the transient and persistent information caches.

The ‘expiration time’ of each sample is computed by adding the duration specified by the LIFESPAN QoS to the source timestamp.
The source timestamp is either automatically computed by the DDS Service each time the DataWriter write operation is called, or else supplied by the application by means of the write_w_timestamp operation.

This QoS relies on the sender and receiving applications having their clocks sufficiently synchronized.
If this is not the case and the DDS Service can detect it, the DataReader is allowed to use the reception timestamp instead of the source timestamp in its computation of the expiration time.
