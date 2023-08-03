---
layout: default
title: Managed Lifecycle Nodes
abstract:
  This article describes the concept of a node with a managed life cycle.
  It aims to document some of the options for supporting managed Lifecycle nodes in ROS 2.
  It has been written with consideration for the existing design of the ROS 2 C++ client library, and, in particular, the current design of executors and rmw.
author: '[Geoffrey Biggs](https://github.com/gbiggs) [Tully Foote](https://github.com/tfoote)'
date_written: 2015-06
last_modified: 2023-08
published: true
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Authors: {{ page.author }}

Date Written: {{ page.date_written }}

Last Modified: {% if page.last_modified %}{{ page.last_modified }}{% else %}{{ page.date_written }}{% endif %}

## Background

A managed life cycle for nodes allows greater control over the state of `ROS 2` system.
It allows `roslaunch` to ensure that all components have been instantiated correctly before it allows any component to begin executing its behavior.
It also allows nodes to be restarted or replaced on-line.

The most important concept of this document is that a managed node presents a known interface, executes according to a known Lifecycle state machine, and otherwise can be considered a black box.
This allows freedom to the node developer on how they provide the managed Lifecycle functionality, while also ensuring that any tools created for managing nodes can work with any compliant node.

### Entity Types
There are 2 "types" of `ROS 2` entities (e.g., `Publisher`, `Subscribers`, `Services` etc.):

#### 1. Persistent Entities
These are entities that do not align with the Lifecycle state machine (i.e., standard/normal entities that are allocated/spin as they would in a typical `Node`).
For clarity, these entities are referred to as "Persistent Entities" within this document.

#### 2. Managed Entities 
These are the entities that align with the underlying Lifecycle state machine outlined in the [Managed Entities](#managed-entities) section.
These "Managed Entities" have historically been interchangeable with "Lifecycle Entities".
For clarity and consistency, these entities are referred to as "Managed Entities".

## Lifecycle State Machine
This section outlines the Lifecycle state machine with respective transition behaviors, `CallbackReturn` semantic meaning, and updated states.

![The proposed node Lifecycle state machine](/img/node_lifecycle/life_cycle_sm.png "The proposed node Lifecycle state machine")

There are 5 Primary States:

- `Unconfigured`: starting state with minimal initialization and processing; Can be recovered to in order to re-configure a node
- `Inactive`: entities with memory allocated with minimal entity processing
- `Active`: entities with memory allocated and processing
- `Finalized`: all processing stopped and memory cleaned up; A terminal state with the node being ready to be cleanly destroyed
- `UncleanFinalized`: the node errored and failed to recover internally; A terminal state without the guarantee of a clean destroy

There are also 6 Transition States which are intermediate states during a requested transition.

- `Configuring`: allocate memory (i.e., load and store)
- `Activating`: set up entities to begin processing
- `Deactivating`: stop entities processing 
- `CleaningUp`: deallocate any loaded and stored memory
- `ShuttingDown`: get ready to be cleanly destroyed
- `ErrorProcessing`: attempt to recover to `Unconfigured`

In the transitions states logic will be executed to determine if the transition is successful, failed, or resulted in error.
Success, failure, and error shall be communicated to lifecycle management software through the lifecycle management interface.

There are 8 transitions, they are:

- `create`
- `configure`
- `cleanup`
- `activate`
- `deactivate`
- `shutdown`
- `raise_error`
- `destroy`


### Primary State Definitions and Expected Behaviors
Primary States allow for assumptions and expected behavior of the Lifecycle Node with respect to the  node’s [Managed Entities](#managed-entities). 
Except when in terminal state, all outbound transitions from a Primary State must be to Transition States.

Note "processing" refers to Managed Entities receiving/sending messages, those messages being propagated through the `ROS 2` stack, and user callbacks processing these messages with respect to the executor.
Receiving "no processing" refers to an entity that will not send messages nor have messages taken out of the middleware queue.
See [Managed Entities](#managed-entities) section for more details.

#### **Primary State: `Unconfigured`** 
The Lifecycle state the node is in immediately after being instantiated.
This is the state in which a node may be returned to after an error has happened and an attempted recovery was successful.

*State Assumptions / Behaviors*
- No Managed Entities allocated
- No events processed for Managed Entities

*Outbound State Transitions*
- `Transition:CONFIGURE` → `Configuring`
- `Transition:SHUTDOWN` → `ShuttingDown`
- `RAISE_ERROR` → `ErrorProcessing`


#### **Primary State: `Inactive`**
 This state represents a node that is performing minimal processing with allocated Managed Entities.
 Thus, as defined in the [Managed Entities](#managed-entities) section, the Managed Entities will not be added to the `ROS` graph with data retention to topics following that of an unallocated Persistent Entity.
 The main purpose of this state is to allow a node to be (re-)configured (changing configuration parameters, adding and removing topic publications/subscriptions, etc.) without the need to re-allocate entities or other configured memory.

*State Assumptions / Behaviors*
- Managed Entities are allocated
- No events processed for these Managed Entities

*Outbound State Transitions*
- `Transition:ACTIVATE` → `Activating`
- `Transition:CLEANUP` → `CleaningUp`
- `Transition:SHUTDOWN` →  `ShuttingDown`
- `RAISE_ERROR` → `ErrorProcessing`

#### **Primary State: `Active`**
Equivalent to the default behavior of a created `Node`.
Thus, while in this state, the node performs any processing, responds to service requests, reads and processes data, produces output, etc.

*State Assumptions / Behaviors*
- Managed Entities are allocated
- Events processed for these Managed Entities

*Outbound State Transitions*
- `Transition:DEACTIVATE` →  `Activating`
- `Transition:SHUTDOWN` → `ShuttingDown`
- `RAISE_ERROR` →  `ErrorProcessing`


#### **Primary State: `Finalized`**
The `Finalized` state is the state in which the node ends immediately before being destroyed.
This state exists to support debugging and introspection.
A node which has failed will remain visible to system introspection and may be potentially introspectable by debugging tools instead of directly destructing.
There will be no Lifecycle internal/backend policy for automatically restarting a node.

*State Assumptions / Behaviors*
- This state can only be reached via a `ShuttingDown` → `SUCCESS` → `Finalized`
- Anything the node is deemed responsible for (e.g., Managed Entities) using CPU is/are stopped
- Anything the node is deemed responsible for (e.g., Managed Entities) using RAM/Mem is/are de-allocated
- `node->destroy()` can be safely run


**Terminal state**: No Outbound Transitions



#### **Primary State: `UncleanFinalized`**
This state exists to denote a node that failed recovery in `ErrorProcessing` and therefor the `LifecycleNode` cannot be guaranteed to be cleanly destroyed.
An external supervisor would therefor be responsible for handling of this node (i.e., keeping it around in an unclean state or cleaning it up such as killing the process it is in).

*State Assumptions / Behaviors*
- This state can only be reached upon  `ErrorProcessing` → `FAILURE` → `UncleanFinalized` 
- This state indicates failed recovery and thus is in an unclean / uncertain state w.r.t. Managed Entities / memory / cpu (i.e., node is in an uncertain finalized state where we cannot guarantee it can be cleanly destroyed)

**Terminal state**: No Outbound Transitions


### Transition State Definitions and Expected Behaviors

Transition States exist to transition a node's [Managed Entities](#managed-entities) while allowing for node set up, teardown, and error recovery. As such, these states cannot be terminal states.
Further, the Transition States hold fewer assumptions of Managed Entities as, at any given point in a Transition State, one cannot be exactly sure the state of a Managed Entity nor the ongoing operations of a transition.
Two core design principles govern the given transition graph:

1. Transitions can only be dictated by either a transition request from a Primary State (e.g., `Transition:CONFIGURE` from an `Unconfigured` state) or `CallbackReturn` values (see [Return Values](#return-values)) from a Transition State or `raise_error()` operation.
No other processes transition the state machine (e.g., thrown exceptions).
This keeps a single interface with the state machine and decouples underlying library implementations. 
1. From a given Transition state, each `CallbackReturn` value has a unique destination (i.e., each outgoing edge of a Transition State is unique).


For (1), it is worth noting this is a key departure from the initial design which caught exceptions/errors from transition callbacks and transitioned to `ErrorProcessing`. However, catching exceptions/errors in the backend can be quite opaque from a user standpoint (e.g., what happens when you throw an exception from a separate thread?) and promotes "poor" code design (e.g., if an underlying library has the potential to throw an exception, surround it in a user try/catch or equivalent. If the user would like to, they can then return a `CallbackReturn::ERROR` to transition the state machine).


#### Definition of a Transition
State machines and their respective transition designs can introduce subtle bugs.
To combat this, we explicitly define what a "transition" is.

A "transition" is defined via the `change_state` process with requests processed on a first-come, first-serve basis ordered by time received.
The `change_state` process is considered active (i.e., transitioning) the instant after a request is fully validated.
A request to transition (internal or external) is valid with respect to the transition graph.
To better delineate transitions, a request is considered invalid if there is currently an active transition (i.e., the `change_state` process has started from a prior request but has not completed). 
This must be stipulated as the `change_state` process can be considered active prior to the underlying state machine changing to a Transition State.

The `change_state` process is considered inactive (i.e., complete and not transitioning) on start up as well as the instant the underlying state machine is transitioned to a Primary State.
Once a Primary State is reached from a transition, there will be no automatic retries.
It is worth noting this is prior to publishing the state machine transition event (as outlined in [Management Interface](#management-interface)) as well as responding to the original client request.
These event publications or service responses can fail.
These are expected after a fully successful transition, however, and thus do not affect the underlying state machine.

In the event the backend throws and exception or error mid-transition, the backend will attempt to recover. 
If the error is prior to a calling a user transition callback, the state will fallback (i.e., follow the `CallbackReturn::FAILURE`) path.
The `change_state` process will end and any client will be responded to accordingly.
For example and as outlined in [rclcpp::#1880](https://github.com/ros2/rclcpp/issues/1880), the `OnStateMachineTransitioned` event publication can fail.
If this failure happens after the state machine update but prior to the user callback being called (e.g., `Unconfgured` → `Configuring` → publication fails), the state machine will be transitioned via the `CallbackReturn::FAILURE` (e.g., back to `Unconfgured` within this example).

If the error occurs after a user callback has been called, the state machine will instead follow a `CallbackReturn::ERROR` pathway.
Any client will be responded to accordingly.

#### User Transition Functions
From the user perspective, transitions are completed in user transition functions.
Each Transition state registers exactly one user transition function.
This is to avoid needing to handle a more complicated register/deregister and ordering scheme for multiple callbacks (see [rclcpp::#2216](https://github.com/ros2/rclcpp/issues/2216)).
If a user would like multiple callbacks, they are expected to set up their own mechanism for doing so.

All user transition functions are overridable and contain the prior state as an argument.
This reduces having to explicitly write all user transition functions (e.g., the `Activating` callback may often just be a `CallbackReturn::SUCCESS` as Managed Entities are automatically transitioned). 
All user transition functions are expected to return control back to the Lifecycle backend of a [Return Values](#return-values) either through function return type (synchronous) or response deferral (through a shared handler).
See [Synchronous and Asynchronous Transitions](#synchronous-and-asynchronous-transitions) for details.

By default, all user transition functions return `CallbackReturn::SUCCESS` **except the `ErrorProcessing` callback which returns `CallbackReturn::FAILURE` by default**. 
This avoids the unintentional situation of returning a `CallbackReturn::ERROR` that silently succeeds/transitions into `Unconfigured`.
With this design, the node will transition to `UncleanFinalized` giving a more clear debugging pathway.

#### Synchronous and Asynchronous Transitions
By default, a transition is synchronous (i.e., thread-blocking requiring a return value from the user transition function). 
Asynchronous (i.e., response deferrable) transition functions will also be supported.
A user must specify a transition to be asynchronous.
For a given user transition function (e.g., the `Configuring` callback) only one callback can be registered at a given time.
While it is encouraged for the design of Transition States to be "short-lived" / "quickly" complete, they are not guaranteed to do so.
For example, response deferrable functions can rely on external dependencies with no guarantee on when those dependencies will be met.

For an asynchronous user transition function, the function should additionally receive some method or handler to respond to the transition at a later time (e.g., a handler similar to a `ROS 2 Action GoalHandle`).
When calling an asynchronous transition callback, the Lifecycle backend relinquishes control of the executor thread to the transition callback and does not expect it back until response.
The user should be able to send this response from any thread at any point of execution.
The user will only be able to send a single response with any subsequent responses being deemed invalid (e.g., atomically invalidating the handler).
Finally, the user will have the ability to atomically check if the method of response is still valid.

#### Cancelling a Transition

A cancel service or equivalent will be available to allow cancelling mid-transition.
This is the only built-in interface for an external node to interact with a node in a Transition State.

The cancellation service will follow a [cooperative cancellation approach](https://www.drdobbs.com/parallel/interrupt-politely/207100682). Similar to a transition, a cancel request is serviced on a first-come, first-serve basis with respect to time received. To avoid accidental cancellations, the cancel request must include the Transition State `id` (or equivalent identifier) of the desired transition to cancel.
This follows the same principles of [concurrent `PUT` requests for REST APIs](https://medium.com/swlh/handling-concurrent-requests-in-a-restful-api-5a25a4b81a1). A cancel request is considered valid if the ongoing transition matches this `id` as well as there is not already an ongoing cancel request (similar to the [Definition of a Transition](#definition-of-a-transition)). 

Once a cancel is valid and active, it is up to the user code to check on this cancel request (similar to a `ROS 2 Action`).
This is designed as only the user can demarcate appropriate times and code paths to clean up an ongoing transition (i.e., only the user knows how to safely clean up a request at any given part of the ongoing transition).
The user can opt to ignore this request (i.e., the transition continues as if no cancel request existed) or attempt to handle the request.
The user can denote a successful handling of the request or a failed handling.

If the user accepts and successfully handles the cancellation request, the state will follow the `CallbackReturn::FAILURE` path of the state machine. 
This follows the "fallback" behavior of a transition.
If the user accepts and unsuccesfully handles the cancellation request, the state will follow the `CallbackReturn::ERROR` path of the state machine. 
An ignored cancellation request becomes inactive and failed the instant a transition is inactive.
There will be no automated retrying or transitioning of an ignored cancellation request.
A handled cancel will invalidate any asynchronous handle (i.e., the user transition will be considered inactive).


#### **Transition State: `Configuring`**
In this Transition State the node's, the node is expected to load its configuration and conduct any required setup. 
The node uses this to set up any resources it must hold throughout its life (irrespective of if it is `Active` or `Inactive`).
Example resources may include memory that is held continuously or initializing configuration parameters.

*Outbound Transitions*
- `SUCCESS` → `Inactive`
- `FAILURE` → `Unconfigured`
- `ERROR` → `ErrorProcessing`

#### **Transition State: `Activating`**
This state is expected to do any final preparations to start executing.
This may include acquiring resources that are only held while the node is actually active, such as access to hardware.
Ideally, no preparation that requires significant time (such as lengthy hardware initialization) should be performed in this callback.
Ideally, activating should avoid failure if at all possible.
If there is a possibility of failure, it may be worthwhile to consider moving code to the `Configuring` transition.

*Outbound Transitions*
- `SUCCESS` → `Active`
- `FAILURE` → `Inactive`
- `ERROR` → `ErrorProcessing`

#### **Transition State: `Deactivating`**
This state is expected to do any cleanup to stop executing, essentially reversing the `Activating` changes.

*Outbound Transitions*
- `SUCCESS` → `Inactive`
- `FAILURE` → `Active`
- `ERROR` → `ErrorProcessing`

#### **Transition State: `CleaningUp`**
This state is expected to clear all state and return the node to a functionally equivalent state as when first created, essentially reversing the `Configuring` changes.


*Outbound Transitions*
- `SUCCESS` → `Unconfigured`
- `FAILURE` → `Active`
- `ERROR` → `ErrorProcessing`


#### **Transition State: `ErrorProcessing`**

This Transition State is where any error can be cleaned up in an attempt to recover to `Unconfigured`.
If a full cleanup is not possible, a user can elect to either return:
- `FAILURE`: in which case the node will transition to `UncleanFinalized`
- `ERROR`: in which case a runtime error will be thrown

*Outbound Transitions*

- `SUCCESS` → `Unconfigured`
- `FAILURE` → `UncleanFinalized`
- `ERROR` → `throw runtime_error`


#### **Transition State: `ShuttingDown`**
This state is expected to do any cleanup necessary before destruction.
It may be entered from any Primary State except terminal states (i.e., `Finalized` and `UncleanFinalized`).

*Outbound Transitions*
- `SUCCESS` → `Finalized`
- `FAILURE` → `prior_state`: {`Unconfigured`, `Active`, `Inactive`}
- `ERROR` → `ErrorProcessing`

#### **Operation `create()`**
This operation will instantiate the node, but will not run any code beyond the constructor.

#### **Operation `destroy()`**
This is a stateless operation to be taken from the `Finalized` state.
This operation will simply cause the deallocation of the node.
This operation should always succeed if taken from the `Finalized` state.

#### **Operation `raise_error()`**
This operation transitions the node from the primary states `{Unconfigured, Inactive, Active}` to `ErrorProcessing`.
This is used to attempt a recovery to `Unconfigured` from a Primary State in the case of an internal error.
Thus, this operation is only available internally to the node.


### Return Values
As outlined in [Transition State Definitions and Expected Behaviors](#transition-state-definitions-and-expected-behaviors), the Lifecycle state machine can only be transitioned via transition requests and `CallbackReturn` values.
`CallbackReturn` values are defined as:

#### **Return `SUCCESS`**

Indicates the requested transition completed all necessary behaviors (e.g., set up/teardown/recovery) to transition to the goal Primary State.

#### **Return `FAILURE`**

Indicates the requested transition did not complete the necessary behaviors to transition to the goal Primary State. 
With the exception of returning `FAILURE` from within `ErrorProcessing`, this return indicates that the user has completed **all necessary behavior to fallback to the previous Primary State**. 
If thrown within `ErrorProcessing`, this return value indicates an `UncleanFinalized` state.

#### **Return `ERROR`**

Indicates an attempted recovery to `Unconfigured`, often in the case of an error arising in user code.
With the exception of returning `ERROR` from within `ErrorProcessing`, this return indicates the current state should transition to `ErrorProcessing` in an attempt to recover to an `Unconfigured` state.
If returned within `ErrorProcessing`, a runtime error will be thrown.

It is worth noting, unlike the prior design and as outlined in [Transition State Definitions and Expected Behaviors](#transition-state-definitions-and-expected-behaviors), there is no backend mechanism to catch thrown exceptions or errors.
Therefor it is expected that user code catches errors and exceptions, returning `CallbackReturn::ERROR` or calling `raise_error()` when appropriate (e.g., in the `catch` or equivalent statement).

## Management Interface

A managed node will be exposed to the `ROS 2` ecosystem by the following interface, as seen by tools that perform the managing.
This interface should not be subject to the restrictions on communications imposed by the lifecycle states.

It is expected that a common pattern will be to have a container class which loads a managed node implementation from a library and through a plugin architecture automatically exposes the required management interface via methods and the container is not subject to the lifecycle management.
However, it is fully valid to consider any implementation which provides this interface and follows the lifecycle policies a managed node.
Conversely, any object that provides these services but does not behave in the way defined in the Lifecycle state machine is malformed.

These services may also be provided via attributes and method calls (for local management) in addition to being exposed `ROS` messages and topics/services (for remote management).
In the case of providing a `ROS` middleware interface, specific topics must be used, and they should be placed in a suitable namespace.

Each possible supervisory transition will be provided as a service by the name of the transition except `create`.
`create` will require an extra argument for finding the node to instantiate.
The service will report whether the transition was successfully completed.

### Supported Interfaces

This section outlines Persistent Entities created by the backend of Lifecycle nodes. These Persistent Entities make up the external facing management interface of a Lifecycle node. Each of these external interfaces must also include an internal equivalent (i.e., a function the user can call from within the node itself).

The user must be able to specify not creating these Persistent Entities and internal APIs via `NodeOptions`.
Both internal and external transition APIs are created by default. 
`LifecycleNodeOptions` must include an option to disable these interfaces.
E.g., 
```cpp
LCTalker(LifecycleNodeOptions({
   "internal_transition_apis": false, 
   "external_transition_interfaces": true
}));
```
Either all Persistent Entities are created or none are created.
If the node is on a spinning executor, the backend Persistent Entities will be spun by default.

The following interface behaviors will be supported:
- `GetState`: returns the `current_state` of the node.
- `GetAvailableStates`: returns an array of possible Primary States that can be transitioned to.
- `GetAvailableTransitions`: return an array of the possible `start_state` → `goal_state` transitions with respect to the current state.
- `GetTransitionGraph`: returns the full transition graph irrespective of the current state.
- `ChangeState`: 
  - Request includes a transition `id` to attempt a transition.
  - Response includes the success/failure state of the transition.
- `CancelTransition`:
  - Request include a Transition State `id` to attempt the cancellation
  - Response includes the success/failure state of the transition.
- `OnStateMachineTransitioned`: 
  - Publication happening immediately after the underlying state machine is updated.
  - Must include the prior state and the current state.

While not dictated by this design doc, implementation should strongly consider including an `failure_msg` or equivalent for services that include a `bool success` (e.g., [rclcpp::#2154](https://github.com/ros2/rclcpp/issues/2154) and [rclcpp::#2231](https://github.com/ros2/rclcpp/issues/2231)).
This message would be to indicate why the service failed (e.g., `ChangeState` failing due to an already ongoing transition).
It should be considered that if a service does not contain a `bool success`, it should not fail from the Lifecycle client library perspective.

An additional implementation consideration would be to group the `ChangeState`, `CancelTransition`, and `OnStateMachineTransitioned` to be grouped into a single `ROS 2 Action` as it covers all desired behaviors.
This would reduce needed code duplication if implemented as separate services (e.g., [rclcpp::#2213](https://github.com/ros2/rclcpp/issues/2213)).

## Managed Entities

Managed entities transition with the state machine, being automatically allocated and spun. 
To reduce developer confusion, "Managed Entities" will be the canonical naming scheme for these entities (as opposed to "Lifecycle Entities").
A separate API will exist when creating a Managed Entity (e.g., `create_managed_x`) to best denote what type of entity you are creating (Persistent Entity vs. Managed Entitiy).

### Managed Entity Active State
CPU usage (i.e., spinning) of entities is governed by their `active` state. An `inactive` (i.e., not `active`) entity will receive "minimal CPU usage".
There are multiple potential implemenentation schemes to accomplish "minimal CPU usage".
We discuss two here to garner conversation within the design PR (and will subsequently update/remove text once decided upon).
We first outline an "ideal" scenario.
However, this may or may not be practical given the current state of the `rmw` and further outline an alternative solution.

#### Active State via Middleware State Awareness
The `rmw` will have `active` state awareness of a managed entity.
An `active` entity requires being spun on an executor.
Therefor, it is the executor's responsibility to update the `rmw`'s `active` state awareness.
This `active` state will be client query-able the same as `rmw::discoverable` is now.
E.g., `rclcpp::service_is_ready()` currently reflects a `discoverable` service, queried at the `rmw` layer.
An equivalent `service_is_active()` will exist.

Messages can still be sent to an `inactive` entity.
A mechanism will exist where messages that are sent to an `inactive` entity stay in the middleware buffer - respecting the Quality of Service of the entity.

#### Active State via Discoverability Overload
Given the current state of `rmw`, however, an alternative design may instead consider the following:

To accomplish "minimal CPU usage", the `active` state will follow the same principles of `rmw::discoverable` (i.e., an `active` entity is a discoverable entity and an `inactive` entity is not discoverable). This means an `inactive` entity will not exist within the `ROS` graph. From a client perspective (e.g., subscribers/clients), the Managed Entities will not be discoverable. For an `inactive` entity, the `rmw` buffer will respect the Quality of Service the same as it does for a destroyed/nonexistent entity.

### Automating Transitions
Note this section assumes the [Active State via Middleware State Awareness](#active-state-via-middleware-state-awareness) implementation as this is the "ideal" scenario.

The process of {de}allocation as well as {de}activation are defined as:

- Automated {de}allocation: 
  - The entity memory is created/destroyed.
  - The entity is added/removed to/from the `ROS` graph.
- Automated {de}activation:
  - The entity is added/removed to/from the executor to receive processing of events.
  - In addition, a mechanism will exist where messages that are sent to an `inactive` entity stay in the middleware buffer - respecting the Quality of Service of the entity.

By default, Managed Entities will be automatically transitioned as outlined below. This will be configurable to disable any automated transitions on a per Transition State basis (e.g., disable `CleaningUp` automatically deallocating the entities but leave all other automations).
All behaviors are based on the definition of a transition outlined in [Definition of a Transition](#definition-of-a-transition):
  - `Configuring`: allocated as the last step of the transition.
  - `Activating`: activated as the last step of the transition.
  - `Deactivating`: deactivated as the first step of the transition.
  - `CleaningUp`:  deallocated as the first step of the transition.
  - `ErrorProcessing`: deactivated (if active) and then deallocated (if allocated) as the first step of the transition. 
  - `ShuttingDown`: deactivated (if active) and then deallocated (if allocated) as the first step of the transition.

If an automated {de}allocation or {de}activation fail, a `CallbackReturn::ERROR` will be raised in an attempt to recover to an `Unconfigured` state. If a failure happens within `ErrorProcessing`, however, a `CallbackReturn::FAILURE` will be raised to put the node into an `UncleanFinalized` state.

## Node Management

There are several different ways in which a managed node may transition between states.
Most state transitions are expected to be coordinated by an external management tool which will provide the node with it's configuration and start it.
The external management tool is also expected monitor it and execute recovery behaviors in case of failures.
A local management tool is also available, leveraging method level interfaces.
And a node could be configured to self manage, however this is discouraged as this will interfere with external logic trying to managed the node via the interface.

There is one operation expected to originate locally, which is the `raise_error()` operation with resulting transition.

A managed node may also want to expose arguments to automatically configure and activate when run in an unmanaged system.

## Extensions

This lifecycle will be required to be supported throughout the toolchain as such this design is not intended to be extended with additional states.
It is expected that there will be more complicated application specific state machines.
They may exist inside of any lifecycle state or at the macro level these lifecycle states are expected to be useful primitives as part of a supervisory system.
