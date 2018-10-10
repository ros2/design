---
layout: default
title: Actions
permalink: articles/actions.html
abstract:
  Despite their implementation as a separate library and lack of a detailed specification, actions are one of the three core types of interaction between ROS nodes.
  Their asynchronous nature combined with the feedback and control mechanism gives them significantly more power than a remote procedure call.
  This article specifies the requirements for actions, including what a ROS user should see, what the middleware layer should provide, and how actions are communicated.
author: '[Geoffrey Biggs](https://github.com/gbiggs)'
published: true
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Background

ROS services, which provide synchronous Remote Procedure Calls, are a useful concept for sending a request and getting a rapid reply.
But in robotics there are many instances where a reply may take a significant length of time.
Additionally, there are occasions when it is useful to send a request to do some processing or perform some action in the world, where the result is less important than the effect of carrying it out.
The progress of such requests often needs to be tracked, success or failure must be known in addition to receiving back information produced, and the request may need to be cancelled or altered before it completes.
These requirements cannot be fulfilled by a simple RPC mechanism, whether or not it is asynchronous.

To satisfy these use cases, ROS provides a third communication paradigm known as "actions".
An action is a goal-oriented request that occurs asynchronously to the requester, is typically (but not necessarily) longer-running than immediate, can be cancelled or replaced during execution, and has a server that provides feedback on execution progress.

This document defines how actions are specified in the ROS Message IDL, how they will be created and used by ROS users (node developers and system integrators), and how they will be communicated by the middleware.


## Entities involved in actions

The following entities are involved in providing, using and executing an action.

- Action server

  The provider of the action.
  There is only one server for any unique action.
  The action server is responsible for:

  - advertising the action to other ROS entities;

  - for executing the action when a request is received, or rejecting that request, as appropriate;

  - for monitoring execution of the action and providing feedback as appropriate to the action design;

  - for sending the result of the action, including a mandatory success/failure value, to the client when the action completes, whether the action succeeded or not; and

  - for managing the execution of the action in response to additional requests by the client.

- Action client

  The entity making a request to execute an action.
  There may be more than one client for each action server.
  However, the semantics of multiple simultaneous clients is action-specific, i.e. it depends on what the action is and how it is implemented whether multiple simultaneous clients can be supported.
  The action client is responsible for:

  - making a request to the action, passing any needed data for the action execution;

  - optionally periodically checking for updated feedback from the action server;

  - optionally requesting that the action server cancel the action execution; and

  - optionally checking the result of the action received from the action server.


## Action specification

Actions are specified using a form of the ROS Message IDL.
The specification contains three sections, each of which is a message specification:

1. Goal

   The "request" part of the action.
   Contains the data passed to the server of the action from the client, along with the request to begin executing that action.

1. Result

   The final result part of the action.
   Contains the data passed to the client of the action from the action server once the action execution ends, whether successfully or not.
   This data is produced by the action server as appropriate to that action's implementation, and is used by the client to understand how the action turned out.

1. Feedback

   Contains data passed to the client of the action from the action server between commencing action execution and prior to the action completing.
   This data is used by the client to understand the progress of executing the action.

Any of these sections may be empty.

Between each of the three sections is a line containing three hyphens, `---`.

Action specifications are stored in a file ending in `.action`.
There is one action specification per `.action` file.

An example action specification is shown below.

```
# Define the goal
uint32 dishwasher_id  # Specify which dishwasher we want to use
---
# Define the result that will be published after the action execution ends.
uint32 total_dishes_cleaned
---
# Define a feedback message that will be published during action execution.
float32 percent_complete
uint32 number_dishes_cleaned
```


## Goal Identifiers

In ROS 1 Action clients are responsible for creating a goal ID when submitting a goal.
This leads to a race condition between goal creation and cancellation.
If a client submits a goal and immediatly tries to cancel it, the cancelation may fail if it is received by the action server prior to the goal being accepted.

In ROS 2 the action server will be responsible for generating the goal ID and notifying the client.
It won't be possible for the client to cancel the goal until after it has received the goal ID.

## API

This is a high-level overview of how to interact with action servers and action clients.

### API for action servers

Action servers are created with the node interface:

- **create\_action\_server** - This requires the action *type* (specification), action *name* (topic string), and a *callback* for handling goals.
Optionally, a callback for **cancel** requests can also be registered.

Handlers:

- **handle_goal** - *Accepts* or *rejects* a goal request.
- **handle_cancel** - *Accepts* or *rejects* a cancel request for a given goal ID.
Note, 'accepting' does not mean the goal is canceled, but signals to the client that the goal will be canceled (ie. preempting).

Once created, an action server provides the following commands:

- **publish_feedback** - Provide feedback for a goal.
Publishes a message matching the action feedback type as defined in the specification.
- **set_canceled** - Termiante an active goal with a cancel result message.
- **set_succeeded** - Terminate an active goal with a successful result message.
- **set_aborted** - Terminate an active goal with an aborted result message.

### API for action clients

### Example usage

Disclaimer: These examples show how we **imagine** actions to be used, but it is subject to change.

### C++

#### Action server usage

TODO

#### Action client usage

TODO

### Python

#### Action server usage

TODO

#### Action client usage

TODO

### Real-time actions

Actions may be used from or served by real-time nodes.
The action server and action client APIs should be real-time capable.


## Introspection tools

Actions, like topics and services, are introspectable from the command line.

In ROS 1, actions are visible in the output of the `rostopic` tool.

In ROS 2, actions will not be visible as a set of topics nor a set of services.
They will be visible using a separate `ros2 action` command line tool.

The command line tool will be similar to the `ros2 service` tool.
It will be able to:

- list known actions,

- display the arguments for an action's goal,

- display the type of an action's feedback and result,

- display information about the server of an action,

- display the underlying topics and/or services providing the action,

- find actions by action type, and

- call an action, display feedback as it is received, display the result when received, and cancel the action (when the tool is terminated prematurely).

Each action will be listed and treated as a single unit by this tool.
This is irrespective of the implementation, which may use several topics or services to provide a single action.


## Middleware implementation

### ROS 1 Background
In ROS 1, actions are implemented as a separate library using a set of topics under a namespace taken from the action name.
This implementation was chosen because ROS services are inherently synchronous, and so incompatible with the asynchronous nature of the action concept.
There is also a need for a status/feedback channel and a control channel.

### ROS 2

Actions will be implemented on top of topics an services.
However, they will be included in all client libraries in ROS 2 with a common implmentation in C.
This reduces the work to implement actions at the client library level since existing middlewares do not need to be updated.

It is possible actions could be implemented in the middlware layer in the future.
One option for DDS middlewares is Remote Procedure Call (DDS-RPC).
However, DDS-RPC does not provide facilities for interrupting service calls or receiving feedback on their progress.
It does provide for receiving a return value from a request and an indication of whether the request was successful.
Unsuccessful requests are returned with an exception.
A DDS based middlware would still need to separately provide status and feedback channels.

### Topics and Services Used

In ROS 1 an action is defined entirely using topics.
In ROS 2 an action is the combination of the following services and topics.

#### Goal Submission Service

* **Direction**: Client calls Server
* **Request**: Description of goal
* **Response**: Whether goal was accepted or rejected, and a unique identifier for the goal, and time goal was accepted.

The purpose of this service is to submit a goal to the action server.
It is the first service called to begin an action.
A user-define description of the goal is sent as the request.
The response is a standard action message indicating whether or not the goal was accepted, and if so the identifier the server will use to describe the goal.

#### Cancel Request Service

* **Direction**: Client calls Server
* **Request**: Goal identifier, time stamp
* **Response**: Goals that will be attempted to be cancelled

The purpose of this service is to request to cancel one or more goals on the action server.
A cancellation request may cancel multiple goals.
The result indicates which goals will be attempted to be cancelled.
Whether or not a goal is actually cancelled is indicated by the status topic and the result service.

The cancel request policy is the same as in ROS 1.

* If the goal ID is empty and time is zero, cancel all goals
* If the goal ID is empty and time is not zero, cancel all goals accepted at or before the time stamp
* If the goal ID is not empty and time is not zero, cancel the goal with the given id regardless of the time it was accepted
* If the goal ID is not empty and time is zero, cancel the goal with the given id and all goals accepted at or before the time stamp

#### Get Result Service

* **Direction**: Client call Server
* **Request**: Goal ID
* **Response**: Status of goal and user defined result

The purpose of this service is to get the final result of a service.
After a goal has been accepted the client should call this service to receive the result.
The result will indicate the final status of the goal and any user defined data.

#### Goal Status Topic

* **Direction**: Server publishes
* **Content**: Goal id, time it was accepted, and an enum indicating the status of this goal.

This topic is published by the server to broadcast the status of goals it has accepted.
The purpose of the topic is for introspection; it is not used by the action client.
Messages are published when transitions from one status to another occur.

The possible statuses are:

* *Accepted*
  * The goal has been accepted by the action server
  * Next status *Executing* or *Accepted Cancellation*
* *Executing*
  * The action server is attempting to reach the goal
  * Next status *Accepted Cancellation*, *Succeeded*, *Aborted*
* *Accepted Cancellation*
  * The action server will try to cancel the indicated goal
  * Next status *Cancelled*, *Succeeded*, *Aborted*
* *Cancelled*
  * The action server successfully canceled the goal
  * No more statuses will be published
* *Succeeded*
  * The action server successfully reached the goal
  * No more statuses will be published
* *Aborted*
  * The action server failed reached the goal
  * No more statuses will be published

#### Feedback Topic

* **Direction**: Server publishes
* **Content**: Goal id, user defined feedback message

This topic is published by the server to send application specific progress about the goal.
It is up to the author of the action server to decide how often to publish the feedback.
