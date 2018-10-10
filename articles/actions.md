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

Actions are a first-class citizen in the ROS API, alongside topics and services.
Action servers and clients are created using the node interface.

### API for action servers

### API for action clients

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

In ROS 1, actions are implemented using a set of topics under a namespace taken from the action name.
This implementation was chosen because ROS services are inherently synchronous, and so incompatible with the asynchronous nature of the action concept.
There is also a need for a status/feedback channel and a control channel.

The Remote Procedure Call over DDS (DDS-RPC) specification does not explicitly provide facilities for interrupting service calls or receiving feedback on their progress.
It does provide for receiving both a return value from a request and, at the same time, an indication of whether the request was successful or raised an exception, with the exception type included in this information.

This means that an implementation of actions cannot simply be a DDS-style RPC.
The implementation must separately provide status/feedback and control channels.

An asynchronous service is used to provide the initial action request channel and the final result.
The action server shall create a server for this service upon construction.
The action client shall create a client for this service upon construction.

A topic is used to provide the feedback channel.
The action server shall publish this topic upon construction.
The action client shall subscribe to this topic upon construction.

A topic is used to provide the control channel.
The action server shall publish this topic upon construction.
The action client shall subscribe to this topic upon construction.

### Topic and service naming

The topic and service used to provide an action are named as follows.
