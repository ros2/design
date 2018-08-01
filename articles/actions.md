---
layout: default
title: Actions
permalink: articles/actions.html
abstract:
  Despite their implementation as a separate library and lack of a detailed specification, actions are one of the three core types of interaction between ROS nodes. Their asynchronous nature combined with the feedback and control mechanism gives them significantly more power than a standard RPC. This article formalises the requirements for actions, including what a ROS user should see and what the middleware layer should provide.
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

ROS services, which provide synchronous Remote Procedure Calls, are a useful concept for sending a request and getting a rapid reply. But in robotics there are many instances where a reply may take a significant length of time. Additionally, there are occasions when it is useful to send a request to do some processing or perform some action in the world, where the result is less important than the effect of carrying it out. The progress of such requests often needs to be tracked, success or failure must be known in addition to receiving back information produced, and the request may need to be cancelled or altered before it completes. These requirements cannot be fulfilled by a simple RPC mechanism, whether or not it is asynchronous.

To satisfy these use cases, ROS provides a third communication paradigm known as "actions". An action is a goal-oriented request that occurs asynchronously to the requester, is typically (but not necessarily) longer-running than immediate, can be cancelled or replaced during execution, and has a server that provides feedback on execution progress.

This document defines how actions are specified, what they look like to ROS users (both node developers and system integrators)

## Action specification

Actions are specified using a form of the ROS Message IDL. The specification contains three sections, each of which is a message specification:

1. Goal

1. Result

1. Feedback

Any of these sections may be empty.

Between the three sections is a line containing three hyphens, `---`.

Action specifications are stored in a file ending in `.action`. There is one action specification per `.action` file.

An example action specification [taken from the actionlib wiki] is shown below.

```
# Define the goal
uint32 dishwasher_id  # Specify which dishwasher we want to use
---
# Define the result
uint32 total_dishes_cleaned
---
# Define a feedback message
float32 percent_complete
uint32 number_dishes_cleaned
```

## Serving and using actions

Actions are a first-class citizen in the ROS API, alongside topics and services.

Action clients will use an API that provides a proxy object for the action. This will be a templated class, using the action class generated from the action specification as the template parameter. The client shall create an instance of this class, providing the address of the intended action server. Each instance of this class can only be related to one action server. Methods of the class will provide facilities for sending a goal to the action server, receiving a result, and getting feedback.

Action servers will use an API that provides a templated server class, using the action class generated from the action specification as the template parameter. The node implementer will create a function that implements the action's behaviour, create an instance of the templated server class, and bind the implementing function to the server. The implementing function will receive as one of its parameters the received goal message, and as another parameter the action server instance. The implementation shall use the action server instance to provide progress feedback and to report the result and success/failure/error status of the action's execution.

Actions may be used from or served by real-time nodes. Therefore the actions API must be real-time capable.

## Introspection tools

Actions, like topics and services, are introspectable from the command line.

In ROS 1, actions are visible in the output of the `rostopic` tool. 

In ROS 2, actions will not be visible as a set of topics. Nor will they be visible as a set of services [in the case that services be used to implement them]. They will be visible using a separate `ros2 action` command line tool.

The command line tool will be similar to the `ros2 service` tool. It will be able to:

- list known actions,
- display the arguments for an action's goal,
- display the type of an action's feedback and result,
- display information about the server of an action,
- display the underlying topics and/or services providing the action,
- find actions by action type, and
- call an action, display feedback as it is received, display the result when received, and cancel the action (when the tool is terminated prematurely)

Each action, despite using multiple topics and/or services in its implementation, will be listed and treated as a single unit by this tool. [This will probably be a namespace that contains the underlying topics, etc.]

## Middleware implementation

In ROS 1, actions are implemented using a set of topics under a namespace taken from the action name. This implementation was chosen because ROS services are inherently synchronous, and so incompatible with the asynchronous nature of the action concept. There is also a need for a status/feedback channel and a control channel.

The Remote Procedure Call over DDS (DDS-RPC) specification does not explicitly provide facilities for interrupting service calls or receiving feedback on their progress. It does provide for receiving both a return value from a request and, at the same time, an indication of whether the request was successful or raised an exception, with the exception type included in this information.

This means that an implementation of actions cannot simply be a DDS-style RPC. The implementation must separately provide status/feedback and control channels. While a control channel could be implemented as a separate RPC, due to the dataflow nature of feedback it would be best implemented as a separate topic.

