---
layout: default
title: RPC API design in ROS
permalink: articles/ros_rpc.html
abstract:
  This is an exploration of possible design patterns for the next generation of ROS Remote Procedure Call interfaces.
  This paper is focused on specifying the user API and leaves the implementation unspecified.
  It is expected that there are one or more RPC implementations which can be used, such as Apache Thrift, ROS RPC, and MsgPack.
author: '[Tully Foote](https://github.com/tfoote)'
published: false
---

* This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Background

In ROS there are two types of Remote Procedure Calls (RPCs).
A primitive of ROS is the Service call which provides a synchronous blocking API to make RPCs.
The other RPC available in ROS is an Action.
An action is built on top of the anonymous publish subscribe topics and provides feedback channel as well as timeout mechanisms and is preemptable.

## Areas for Improvement

There are several areas where the existing API could be improved.
Several of these have been proven in their use in Action based APIs.

### Asynchronous API


This allows alternative threading models.
The synchronous API can be implemented on top to provide the simpler API to the user.
Providing an asynchronous API on top of a Synchronous API is harder.

### Timeouts

If a service provider hangs or otherwise does not return correctly without a timeout a calling thread may be hung indefinitely.
Allowing a timeout allows for recovery behavior in the case of failure conditions besides a dropped connection, allowing the user to choose to continue waiting for another timeout cycle or abort the request.

### Preemptability

One of the features of Actionlib is the ability to cancel a request which is in process.
If there are long running RPCs this is valuable to enable the system to react promptly, enabling the recalling of the RPC.
This requires the user defined service implementation to support preempting long running operations.

### Feedback


Actions also provide a mechanism for giving feedback.
This provides good introspection into the state of the system.
The feedback is specifically relevant to decisions about when/if to preempt previous calls.
Feedback can be provided via an external mechanism such as having a parallel topic.

### Reliable Transport


It is important that the system cannot get into an undetermined state if there is packet loss.
This is been a problem in particular for Actions over a wireless link.
To avoid this the Actions API could be switched to the reliable RPC calls while providing the feedback which is a streaming service via unreliable topics.

### Logging and Introspection


When logging a ROS system recording all topics does not capture any information about ongoing service calls.
Since service calls are point to point, logging them is hard.
The information needs to be exposed loggable and introspectable to be useful.

## Proposed Approach


The features outlined above for existing implementations and improvements can be very powerful but if implemented as a monolithic implementation will be much more complicated than necessary for most use cases.
There are 4 levels of abstraction the abstraction which the above features can be sorted into at each level providing higher levels of functionality to the user.

TODO add graphic

### Plain RPC


The Plain RPC is expected to be able to be leveraged from one or more externally developed RPC libraries.
We expect several libraries to meet the minimum requirements and aim to make them interchangeable.

### ROS Asynchronous RPC API


The ROS Asynchronous RPC API will provide the lowest level of abstraction.
It will provide a callback based API with a timeout.
It will utilize the core Plain RPC libraries to do the communication as well as provide reliable communications either by leveraging the Plain RPC’s capabilities or providing a layer on top of them with message acknowledgements.
It will expose to the User a timeout. 

For logging/introspection purposes the RPC Server instance would publish all incoming requests and outgoing responses on topics inside the namespace of the service.

### ROS Preemptable RPC API


The ROS Preemptable RPC API will extend the Asynchronous API to enable preempting of a RPC in progress using a UID.
This UID will be provided by the initial request method.

### ROS Action RPC API (Not effecting RPC Protocol)



The feedback topic can be separated to use a topic, this avoids integrating the feedback into the core RPC implementation.
The ROS Action RPC API will extend the Preemptable RPC API to provide a feedback channel via published ROS topic.
This can be built on top of the Preemptable RPC API with the PubSub API thus isolating it from the RPC design.

### ROS Synchronous RPC API (Not Effecting RPC Protocol)


For each of the above Asynchronous APIs a thin wrapper can be built on top to provide a single function based interface for ease of use.
It will block until a response is returned or the timeout is reached.
If wrapping a preemptable RPC it will both timeout on the user side as well as preempt the remote side.
This will just be a thin layer on top of the Asynchronous API requiring no additional features of the core RPC protocol.

## Technical Issues

Visibility of UIDs
------------------

UID’s are generally necessary for asynchronous communications to pair the requests and responses.
There are possible ways to build this without a UID embedded in the datatype, however it will require some level of heuristics to do data association. 

There are two options to either require the User to embed the UID into the message, or add those fields during generation.
How does this affect introspectability? There are issues for how to introspect if topics are used for introspection/logging. 

### Action Files?


Should we keep .action file types with their generators? Change to .srv + .msg.
This is highly influenced by the above. 

### Logging


Recorded service calls can not be played back generically like topics because the service client will not handle it because it will not be associated with a request. 
There could be some test service servers which can respond with similar queries, however it’s not clearly defined. 

Logging of RPCs is still valuable for debugging and introspection. It would be valuable to have a view of events that happened in sequence as well as their content.
These events and content should be associatable. How to embed this association without significantly affecting the User API is a challenge. 

This is a generic issue with logging and effects potentially all logging and should be captured in a separate white paper.
It might be possible to pad communications with debugging data.

The above UID’s may be only locally unique (client specific for instance).
For logging UID’s need to be unique within the entire ROS instance for debugging.

### Collapse Preemptable and Asynchronous?

These two types are very similar and limiting the variants on the API might be easier for the user. 
If the UID must be generated/embedded into the protocol. It should be embedded into all calls. This will help loggability.
For the non-preemptable case the implementation can simply not instantiate the state machine and preemption mechanisms.


### caller_id availability?

There are use cases when caller_id is valuable.
Actionlib users sometimes used it to provide connection based information.
By default the anonymous publish subscribe is to not provide the caller_id.
In the current implementation the caller_id is actually the node name, which is ambiguous in cases like nodelets.
Providing a mechanism for declaring the caller_id might be helpful, but would require a lot of tools to parameterize and remap for full power. 

One possible solution is to write a spec where a field that matches [CallerID caller_id] would be automatically substituted by a publisher if embedded into a message to provide this specifically where valuable.

### Not bundle feedback at core level

If namespace remapping works, feedback could simply be a topic which is in the same namespace as a peer.
This would enable multiple feedback topics of differing types and frequencies.
Action API classes could be provided on top of the core infrastructure to bundle an RPC with a feedback.
