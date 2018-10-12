---
layout: default
title: Actions
permalink: articles/actions.html
abstract:
  Actions are one of the three core types of interaction between ROS nodes.
  This article specifies the requirements for actions, how they've changed from ROS 1, and how they're communicated.
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

There are three forms of communication in ROS: topics, services, and actions.
Topic publishers broadcast to multiple subscribers, but communication is one-way.
Service clients request service servers to do something and get a response back, but there is no information about the progress.
Actions clients request an action server reach some end state, and the action server will report progress as well as the final result.

Actions are useful when a response may take a significant length of time.
They allow a client to track the progress of a request, get the final outcome, and optionally cancel the request it before it completes.

This document defines requirements for actions, how they are specified in the ROS Message IDL, and how they are communicated by the middleware.

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

## Differences between ROS 1 and ROS 2 actions

### First Class Support
In ROS 1, actions are implemented as a separate library `actionlib` which is built on top of the client libraries.
This was done to avoid increasing the work required to create a client library in a new language, but actions turned out to be very important to packages like the navigation stack<sup>[1](#separatelib)</sup>.
In ROS 2 actions will be included in the client library implementations.
The work of writing a client library in a new language will be reduced by creating a common implmentation in C.

### Services used for Actions

In ROS 1 actions were implemented using a set of topics under a namespace taken from the action name.
ROS 1 Services were not used because they are inherently synchronous, and actions need to be asynchronous.
Actions also needed to send status/feedback and be cancelable.
In ROS 2 services are asynchronous in the common C implementation, so actions will use a combination of services and topics.

### Goal Identifiers

In ROS 1, Action clients are responsible for creating a goal ID when submitting a goal.
In ROS 2 the action server will be responsible for generating the goal ID and notifying the client.

The server is better equiped to generate a unique goal id than the client because there may be multiple clients who could independently generate the same goal id.
Another reason is to avoid a race condition between goal creation and cancellation that exists in ROS 1.
In ROS 1 if a client submits a goal and immediatly tries to cancel it then the cancelation may or may not happen.
If the cancelation is processed before the goal submission then `actionlib` will ignore the cancellation request without notifying the user's code.

### Namespacing of Generated Messages and Services

Multiple message and service definitions are generated from a single action definition.
In ROS 1, the generated messages were prefixed with the name of the action to avoid conflicts with other messages and services.
In ROS 2, the generated service and message definitions will exist in a different namespace to be impossible to conflict with non-action message and service definitions.
For example, in Python the code from the generated definitions should be in the module `action` instead of `srv` and `msg`.
In C++, the generated code should be in the namespace and folder `action` instead of `srv` and `msg`.

### Visibility of Action Services and Topics

In ROS 1 `rostopic list` would show all action topics in its output.
In ROS 2 `ros2 topic list` and `ros2 service list` will not show topics and services used by actions by default.
The can still be shown by passing an option to the commands to show hidden services and topics.

## Action Interface Definition

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

## Goal States

![Action Server State Machine](../img/actions/action_server_state_machine.png)

The action server is responsible for accepting (or rejecting) goals requested by clients.
The action server is also responsible for maintaining a separate state for each accepted goal.

There are two active states:

- **EXECUTING** - The goal has been accepted and is currently being executed by the action server.
- **CANCELING** - The client has requested that the goal be canceled and the action server has accepted the cancel request.
This state is useful for any "clean up" that the action server may have to do.

And three terminal states:

- **SUCCEEDED** - The goal was achieved successfully by the action server.
- **ABORTED** - The goal was terminated by the action server without an external request.
- **CANCELED** - The goal was canceled after an external request from an action client.

State transitions triggered by the action server:

- **set_succeeded** - Notify that the goal completed successfully.
- **set_aborted** - Notify that an error was encountered during processing of the goal and it had to be aborted.
- **set_canceled** - Notify that canceling the goal completed successfully.

State transitions triggered by the action client:

- **send_goal** - A goal is sent to the action server.
The state machine is only started if the action server *accepts* the goal.
- **cancel_goal** - Request that the action server stop processing the goal.
A transition only occurs if the action server *accepts* the request to cancel the goal.
The goal may transition to CANCELING or CANCELED depending on the action server implementation.

## API

Proposed examples can be in the respository [examples (branch: actions_proposal)](https://github.com/ros2/examples/tree/actions_proposal).

C++:

- [examples/rclcpp/minimal_action_server](https://github.com/ros2/examples/tree/actions_proposal/rclcpp/minimal_action_server)
- [examples/rclcpp/minimal_action_client](https://github.com/ros2/examples/tree/actions_proposal/rclcpp/minimal_action_client)

Python:

- [examples/rclpy/actions](https://github.com/ros2/examples/tree/actions_proposal/rclpy/actions)

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

Under the hood, an action is made up of three services and two topics, each descibed in detail here.

### Goal Submission Service

* **Direction**: Client calls Server
* **Request**: Description of goal
* **Response**: Whether goal was accepted or rejected, a unique identifier for the goal, and the time when the goal was accepted.

The purpose of this service is to submit a goal to the action server.
It is the first service called to begin an action, and is expected to return quickly.
A user-define description of the goal is sent as the request.
The response indicates whether or not the goal was accepted, and if so the identifier the server will use to describe the goal.

The QoS settings of this service should be set the so the client is guaranteed to receive a response or an action could be executed without a client being aware of it.

### Cancel Request Service

* **Direction**: Client calls Server
* **Request**: Goal identifier, time stamp
* **Response**: Goals that will be attempted to be canceled

The purpose of this service is to request to cancel one or more goals on the action server.
A cancellation request may cancel multiple goals.
The result indicates which goals will be attempted to be canceled.
Whether or not a goal is actually canceled is indicated by the status topic and the result service.

The cancel request policy is the same as in ROS 1.

* If the goal ID is empty and time is zero, cancel all goals
* If the goal ID is empty and time is not zero, cancel all goals accepted at or before the time stamp
* If the goal ID is not empty and time is not zero, cancel the goal with the given id regardless of the time it was accepted
* If the goal ID is not empty and time is zero, cancel the goal with the given id and all goals accepted at or before the time stamp

### Get Result Service

* **Direction**: Client calls Server
* **Request**: Goal ID
* **Response**: Status of goal and user defined result

The purpose of this service is to get the final result of a service.
After a goal has been accepted the client should call this service to receive the result.
The result will indicate the final status of the goal and any user defined data.

Once the server sends the result to the client it should free up any resources used by the action.
If the client never asks for the result then the server should discard the result after a timeout period.

### Goal Status Topic

* **Direction**: Server publishes
* **Content**: Goal ID, time it was accepted, and an enum indicating the status of this goal.

This topic is published by the server to broadcast the status of goals it has accepted.
The purpose of the topic is for introspection; it is not used by the action client.
Messages are published when transitions from one status to another occur.

The possible statuses are:

* *Accepted*
  * The goal has been accepted by the action server and may now be executing
* *Cancelling*
  * The action server will try to cancel the indicated goal
* *Cancelled*
  * The action server successfully canceled the goal
* *Succeeded*
  * The action server successfully reached the goal
* *Aborted*
  * The action server failed reached the goal

### Feedback Topic

* **Direction**: Server publishes
* **Content**: Goal id, user defined feedback message

This topic is published by the server to send application specific progress about the goal.
It is up to the author of the action server to decide how often to publish the feedback.

### Client/Server Interaction Examples

Here are a couple of sequence diagrams depicting typical interactions between an action client and action server.

#### Example 1

In this example, the action client request a goal and gets a response from the server accepting the goal (synchronous).
Upon accepting the goal, the action server starts a user defined execution method for completing the goal.
Following the goal request, the client makes an asynchronous request for the result.
The user defined method publishes feedback to the action client as it executes the goal.
Ultimately, the user defined method populates a result message that is used as part of the result response.

![Goal Lifecycle Example 0](../img/actions/interaction_example_0.png)

#### Example 2

This example is almost identical to the first, but this time the action  client requests for the goal to be canceled mid-execution.
Note that the user defined method is allowed to perform any shutdown operations after the cancel request before returning with the cancellation result.

![Goal Lifecycle Example 1](../img/actions/interaction_example_1.png)

#### Example 3

Here is a more complex example involving multiple goals.

TODO

## Bridging between ROS 1 and ROS 2

TODO

## Alternatives

These alternative approaches to actions in ROS 2 were considered.

### Actions in rmw

An alternative to using services and topics is to implement actions in the rmw layer.
This would enable using middleware specific features better suited actions.
The default middleware in ROS 2 uses DDS, and there don't appear to be any DDS features better for actions than what are used for services and topics.
Additionally implementing actions in the rmw implementations increases the complexity of writing an rmw implementation.
For these reasons actions will be implemented at a higher level.

## References

1. <a name="separatelib" href="https://discourse.ros.org/t/actions-in-ros-2/6254/5">https://discourse.ros.org/t/actions-in-ros-2/6254/5</a>
