---
layout: default
title: RPC API design in ROS
permalink: articles/ros_rpc.html
abstract:
  This is an exploration of possible design patterns for the next generation of ROS Remote Procedure Call interfaces.
  This paper is focused on specifying the user API and leaves the implementation unspecified.
  It is expected that there are one or more RPC implementations which can be used, such as Apache Thrift, ROS RPC, and MsgPack.
author: '[Tully Foote](https://github.com/tfoote)'
published: true
---

* This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

In ROS there are two types of Remote Procedure Call (RPC) primitives.
The concept of Services in ROS are basic request-response style RPC's, but the concept of Actions in ROS additionally have feedback during the time between between the request and response and the request is preemptable.

## Ideal System

It is useful to consider the ideal system to understand how it relates to the current system and how a new system could work.
An ideal RPC system would have the qualities laid out in the following paragraphs.

### Asynchronous API

This allows alternative threading models.
The synchronous API can be implemented on top to provide the simpler API to the user.
Providing an asynchronous API on top of a Synchronous API is harder or less efficient.

### Timeouts

If a service provider hangs or otherwise does not return correctly, then a calling thread may be hung indefinitely.
Having a timeout allows for recovery behavior in the case of failure conditions besides a dropped connection, allowing the user to choose to continue waiting for another timeout cycle or abort the request.

### Preemptability

Preemption is a desirable feature anytime there are long running or non-deterministically running remote procedures.
The ability to preempt a long running procedure with either a timeout on synchronous calls or a call to cancel on asynchronous requests.
Preemptability is a required feature for the concept of Actions to be implemented.

### Feedback

In order to effectively use preemption without a timeout, periodic or procedural feedback is usually required.
Feedback can be provided via an external mechanism such as having an implicit parallel publish-subscribe type topic.
Feedback is also central the concept of Actions in ROS.

### Reliable Transport

It is important that the system cannot get into an undetermined state if there is packet loss.
If a request or response is never received by either side the system must be able to recover or inform the user in some way.
In the past, this is been a problem for ROS Actions over a lossy wireless link.

### Logging and Introspection

When logging a ROS system recording all topics does not capture any information about ongoing service calls.
Since service calls are point to point, logging them is difficult.
Some level of the Services and Actions would be made available and ideally, they could be efficiently recorded and could be played back in some manner, though it is not clear how that would work.

## Proposed Approach

The features outlined above are desirable but if implemented as a monolithic implementation will be much more complicated than necessary for most use cases, e.g. feedback is not always required, but if monolithic it would always be a part of the API.
There are 4 levels of abstraction the abstraction which the above features can be sorted into at each level providing higher levels of functionality to the user.

> TODO: add graphic

### Plain RPC

The Plain RPC is expected to be able to be leveraged from one or more externally developed RPC libraries.
We expect several libraries to meet the minimum requirements and aim to make them interchangeable.

### ROS Asynchronous RPC API

The ROS Asynchronous RPC API will provide the lowest level of abstraction.
It will provide a callback based API with a timeout.
It will utilize the core Plain RPC libraries to do the communication as well as provide reliable communications either by leveraging the Plain RPC’s capabilities or providing a layer on top of them with message acknowledgments.

For logging/introspection purposes the RPC Server instance might publish all incoming requests and outgoing responses on topics inside the namespace of the service.

### ROS Preemptable RPC API

The ROS preemptable RPC API will extend the Asynchronous API to enable preempting of a RPC in progress using a UID.
This UID will be provided by the initial request method.

### ROS Action RPC API (Not effecting RPC Protocol)

The feedback topic can be isolated to a separate topic, which avoids integrating the feedback into the core RPC implementation.
The ROS Action RPC API will extend the preemptable RPC API to provide a feedback channel via published ROS topic.
This can be built on top of the preemptable RPC API with the PubSub API thus isolating it from the RPC design.

### ROS Synchronous RPC API (Not Effecting RPC Protocol)

For each of the above Asynchronous APIs a thin wrapper can be built on top to provide a single function based interface for ease of use.
It will block until a response is returned or the timeout is reached.
If wrapping a preemptable RPC it will both timeout on the user side as well as preempt the remote side.
This will just be a thin layer on top of the Asynchronous API requiring no additional features of the core RPC protocol.

## Technical Issues

There are some issues with the above proposed approach, which are outlined below.

### Visibility of UID's

UID’s are generally necessary for asynchronous communications to pair the requests and the responses.
There are possible ways to build this without a UID embedded in the data type, however it will require some level of heuristics to do data association.

There are two options to either require the user to embed the UID into the message, or add those fields during message generation time.
It is unclear how this affects the user's ability to introspect the messages?
This also introduces issues when trying to record and potentially playback Service or Actions.

### Action Files?

Should there be a separate `.action` file type?
Or should it be more like a `.srv` + `.msg` pair?
This is highly influenced by the way UID's are handled.

### Logging

Recorded service calls can not be played back generically like topics because the service client will not handle it because it will not be associated with a request.
There could be some test service servers which can respond with similar queries, however it is not clearly defined.

Logging of RPCs is still valuable for debugging and introspection.
It would be valuable to have a view of events that happened in sequence as well as their content.
It should be possible to associate recorded request and response pairs.
Which raises the question about how to embed this association without significantly affecting the user's API.

This is a generic issue with logging and effects potentially all logging and should be captured in a separate white paper.
It might be possible to pad communications with debugging data.

The above UID’s may be only locally unique (client specific for instance).
For logging UID’s need to be unique within the entire ROS instance for debugging.

### Collapse Preemptable and Asynchronous?

These two types are very similar and limiting the variants on the API might be easier for the user.
If the UID must be generated/embedded into the protocol.
Then it should be embedded into all calls, which will help with logging.
For the non-preemptable case the implementation can simply not instantiate the state machine and preemption mechanisms.

### caller_id availability?

There are use cases when the concept of `caller_id` is valuable.
Users of ROS Actions would sometimes use it to provide connection based information.
By default the anonymous publish-subscribe mechanism is to not provide the `caller_id`.
In the current implementation the `caller_id` is actually the node name, which is ambiguous in cases like nodelets.
Providing a mechanism for declaring the `caller_id` might be helpful, but would require a lot of tools to parameterize and remap in order to gain the full benefit.

One possible solution is to write a spec where a field that matches `[CallerID caller_id]` would be automatically substituted by a publisher if embedded into a message to provide this specifically where valuable.

### Do not bundle feedback at core level?

If namespace remapping works, feedback could simply be a topic which is in the same namespace as a peer.
This would enable multiple feedback topics of differing types and frequencies.
Action API classes could be provided on top of the core infrastructure to bundle an RPC with a feedback.
