---
layout: default
title: Managed nodes
abstract:
  This article describes the concept of a node with a managed life cycle.
  It aims to document some of the options for supporting manage d-life cycle nodes in ROS 2.
  It has been written with consideration for the existing design of the ROS 2 C++ client library, and in particular the current design of executors.
author: '[Geoffrey Biggs](https://github.com/gbiggs) [Tully Foote](https://github.com/tfoote)'
published: true
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Background

A managed life cycle for nodes allows greater control over the state of ROS system.
It will allow roslaunch to ensure that all components have been instantiated correctly before it allows any component to begin executing its behaviour.
It will also allow nodes to be restarted or replaced on-line.

The most important concept of this document is that a managed node presents a known interface, executes according to a known life cycle state machine, and otherwise can be considered a black box.
This allows freedom to the node developer on how they provide the managed life cycle functionality, while also ensuring that any tools created for managing nodes can work with any compliant node.

## Life cycle

![The proposed node life cycle state machine](/img/node_lifecycle/life_cycle_sm.png "The proposed node life cycle state machine")

There are 4 primary states:

- `Unconfigured`
- `Inactive`
- `Active`
- `Finalized`

To transition out of a primary state requires action from an external supervisory process, with the exception of an error being triggered in the `Active` state.

There are also 6 transition states which are intermediate states during a requested transition.

- `Configuring`
- `CleaningUp`
- `ShuttingDown`
- `Activating`
- `Deactivating`
- `ErrorProcessing`

In the transitions states logic will be executed to determine if the transition is successful.
Success or failure shall be communicated to lifecycle management software through the lifecycle management interface.

There are 7 transitions exposed to a supervisory process, they are:

- `create`
- `configure`
- `cleanup`
- `activate`
- `deactive`
- `shutdown`
- `destroy`

The behavior of each state is as defined below.

### Primary State: Unconfigured

This is the life cycle state the node is in immediately after being instantiated.
This is also the state in which a node may be retuned to after an error has happened.
In this state there is expected to be no stored state.

#### Valid transition out

- The node may transition to the `Inactive` state via the `configure` transition.
- The node may transition to the `Finalized` state via the `shutdown` transition.

### Primary State: Inactive

This state represents a node that is not currently performing any processing.

The main purpose of this state is to allow a node to be (re-)configured (changing configuration parameters, adding and removing topic publications/subscriptions, etc) without altering its behavior while it is running.

While in this state, the node will not receive any execution time to read topics, perform processing of data, respond to functional service requests, etc.

In the inactive state, any data that arrives on managed topics will not be read and or processed.
Data retention will be subject to the configured QoS policy for the topic.

Any managed service requests to a node in the inactive state will not be answered (to the caller, they will fail immediately).

#### Valid transitions out of Inactive

- A node may transition to the `Finalized` state via the `shutdown` transition.
- A node may transition to the `Unconfigured` state via the `cleanup` transition.
- A node may transition to the `Active` state via the `activate` transition.

### Primary State: Active

This is the main state of the node's life cycle.
While in this state, the node performs any processing, responds to service requests, reads and processes data, produces output, etc.

If an error that cannot be handled by the node/system occurs in this state, the transition to the `fatal error` state is taken.

#### Valid transitions out of Active

- A node may transition to the `Inactive` state via the `deactivate` transition.
- A node may transition to the `Finalized` state via the `shutdown` transition.

### Primary State: Finalized

The `Finalized` state is the state in which the node ends immediately before being destroyed.
This state is always terminal the only transition from here is to be destroyed.

This state exists to support debugging and introspection.
A node which has failed will remain visible to system introspection and may be potentially introspectable by debugging tools instead of directly destructing.
If a node is being launched in a respawn loop or has known reasons for cycling it is expected that the supervisory process will have a policy to automatically destroy and recreate the node.

#### Valid transitions out of Finalized

- A node may be deallocated via the `destroy` transition.

### Transition State: Configuring

In this transition state the node's `onConfigure` callback will be called to allow the node to load its configuration and conduct any required setup.

The configuration of a node will typically involve those tasks that must be performed once during the node's life time, such as obtaining permanent memory buffers and setting up topic publications/subscriptions that do not change.

The node uses this to set up any resources it must hold throughout its life (irrespective of if it is active or inactive).
As examples, such resources may include topic publications and subscriptions, memory that is held continuously, and initialising configuration parameters.

#### Valid transitions out of Configuring

- If the `onConfigure` callback succeeds the node will transition to `Inactive`
- If the `onConfigure` callback results in a failure code (TODO specific code) the node will transition back to `Unconfigured`.
- If the `onConfigure` callback raises or results in any other result code the node will transition to `ErrorProcessing`

### Transition State: CleaningUp

In this transition state the node's callback `onCleanup` will be called.
This method is expected to clear all state and return the node to a functionally equivalent state as when first created.
If the cleanup cannot be successfully achieved it will transition to `ErrorProcessing`.

#### Valid transitions out if CleaningUp

- If the `onCleanup` callback succeeds the node will transition to `Unconfigured`.
- If the `onCleanup` callback raises or results in any other return code the node will transition to `ErrorProcessing`.

### Transition State: Activating

In this transition state the callback `onActivate` will be executed.
This method is expected to do any final preparations to start executing.
This may include acquiring resources that are only held while the node is actually active, such as access to hardware.
Ideally, no preparation that requires significant time (such as lengthy hardware initialisation) should be performed in this callback.

#### Valid transitions out if Activating

- If the `onActivate` callback succeeds the node will transition to `Active`.
- If the `onActivate` callback raises or results in any other return code the node will transition to `ErrorProcessing`.

### Transition State: Deactivating

In this transition state the callback `onDeactivate` will be executed.
This method is expected to do any cleanup to start executing, and should reverse the `onActivate` changes.

#### Valid transitions out of Deactivating

- If the `onDeactivate` callback succeeds the node will transition to `Inactive`.
- If the `onDeactivate` callback raises or results in any other return code the node will transition to `ErrorProcessing`.

### Transition State: ShuttingDown

In this transition state the callback `onShutdown` will be executed.
This method is expected to do any cleanup necessary before destruction.
It may be entered from any Primary State except `Finalized`, the originating state will be passed to the method.

#### Valid transitions out of ShuttingDown

- If the `onShutdown` callback succeeds the node will transition to `Finalized`.
- If the `onShutdown` callback raises or results in any other return code the node will transition to `ErrorProcessing`.

### Transition State: ErrorProcessing

This transition state is where any error can be cleaned up.
It is possible to enter this state from any state where user code will be executed.
If error handling is successfully completed the node can return to `Unconfigured`,
If a full cleanup is not possible it must fail and the node will transition to `Finalized` in preparation for destruction.

Transitions to `ErrorProcessing` may be caused by error return codes in callbacks as well as methods within a callback or an uncaught exception.

#### Valid transitions out of ErrorProcessing

- If the `onError` callback succeeds the node will transition to `Unconfigured`.
  It is expected that the `onError` will clean up all state from any previous state.
  As such if entered from `Active` it must provide the cleanup of both `onDeactivate` and `onCleanup` to return success.

- If the `onShutdown` callback raises or results in any other result code the node will transition to `Finalized`.

### Destroy Transition

This transition will simply cause the deallocation of the node.
In an object oriented environment it may just involve invoking the destructor.
Otherwise it will invoke a standard deallocation method.
This transition should always succeed.

### Create Transition

This transition will instantiate the node, but will not run any code beyond the constructor.settings.settings.

## Management Interface

A node that has a managed life cycle complying with the above life cycle shall provide the following interface via ROS topics and services.
This interface is to be used by tools to manage the node's life cycle state transitions, either automatically or manually according to the tool's purpose.

The topics and services of this interface shall function in all states of the node's life cycle.
They shall not be disabled by the node shifting to the `Inactive` state, for example.

### Interface namespace

The interface shall be provided in a namespace named "infra/lifecycle" underneath the node's namespace.

For example, if a node named `talker` has a managed life cycle complying with the state machine described above, it shall provide topics under the namespace `/talker/infra/lifecycle`.

All examples in the following sections are also given assuming a node named `talker`.

If the `infra/lifecycle` namespace is available under a node's namespace, then that node shall be assumed to be functioning according to the managed life cycle.
If the node is not functioning according to the managed life cycle, the `infra/lifecycle` namespace shall not exist.
In other words, tooling shall judge if a node is managed or not by the presence of the `infra/lifecycle` namespace.

### State enumerations

The messages used by the interface shall use the following enumeration for indicating states.

    uint8 UNKNOWN=0
    uint8 UNCONFIGURED=1
    uint8 INACTIVE=2
    uint8 ACTIVE=3
    uint8 FINALIZED=4
    uint8 CONFIGURING=10
    uint8 CLEANING_UP=11
    uint8 SHUTTING_DOWN=12
    uint8 ACTIVATING=13
    uint8 DEACTIVATING=14
    uint8 ERROR_PROCESSING=15

### Life cycle state changes topic

When the node's life cycle changes, it shall broadcast the following message on the `infra/lifecycle/state_change` topic.

    uint8 previous_state
    uint8 next_state
    string trigger

`trigger` may be filled in containing a reason for the life cycle change.
This value is optional.

This topic must at a minimum make the most recent message available to new subscribers at all times.
This may be achieved by use of appropriate QoS settings.

For example, if the `talker` node transitions from the `Active` state to the `Finalised` state via the `ShuttingDown` state in response to an external request to shut down the node, it will produce the following sequence of messages on this topic (values for `trigger` are illustrative only):

    previous_state = INACTIVE
    next_state = SHUTTTING_DOWN
    trigger = "shutdown request"

    previous_state = SHUTTING_DOWN
    next_state = FINALIZED
    trigger = "shutdown returned OK"

If the `talker` node transitions from the `Inactive` state to the `Unconfigured` state via a request to activate the node and an error occurring in activation processing, it will produce the following sequence of messages:

    previous_state = INACTIVE
    next_state = ACTIVATING
    trigger = "activate request"

    previous_state = ACTIVATING
    next_state = ERROR_PROCESSING
    trigger = "error in activating state"

    previous_state = ERROR_PROCESSING
    next_state = UNCONFIGURED
    trigger = "error processing returned OK"

### Current life cycle state service

The node's current life cycle state shall be available via the `infra/lifecycle/get_state` service.
The service definition is:

    ---
    uint8 state
    string state_name

`state_name` must assume one of the following values, according to the value of `state`.

    Value of state   | Value of state_name
    UNKNOWN          | unknown
    UNCONFIGURED     | unconfigured
    INACTIVE         | inactive
    ACTIVE           | active
    FINALIZED        | finalized
    CONFIGURING      | configuring
    CLEANING_UP      | cleaning_up
    SHUTTING_DOWN    | shutting_down
    ACTIVATING       | activating
    DEACTIVATING     | deactivating
    ERROR_PROCESSING | error_processing

The `UNKNOWN`/`unknown` value shall be assumed by clients of the service to indicate that the node is in an unknown state and thus unusable.

### Transition request service

The service `infra/lifecycle/change_state` service shall be provided by the life cycle interface.

When this service call is received, the node's life cycle state shall be shifted to the requested state via any appropriate intermediate states in accordance with the state diagram shown above.

For example, if the node is in the `Inactive` state and a request is made to shift to the `Active` state, the node's life cycle shall first be shifted to the `Activating` state.
Based on the result of the `onActivate()` function called during the `Activating` state, the node's life cycle shall then shift to either the `Active` state or the `ErrorProcessing` state.

The service definition is:

    # Allowable transitions
    uint8 CONFIGURE=0
    uint8 CLEANUP=1
    uint8 ACTIVATE=2
    uint8 DEACTIVATE=3
    uint8 SHUTDOWN=4
    # Transition results
    uint8 TRANSITION_ERROR=0
    uint8 SUCCESS=1
    uint8 WRONG_PREV_STATE=10


    uint8 transition
    ---
    uint8 result

`transition` must take one of the values defined in the transitions enumeration.
Additionally, the allowable values for `transition` is determined by the current life cycle state.
`transition` must take one of the following values, depending on the current life cycle state.

    Current state | `transition` allowable values
    Unconfigured  | `CONFIGURE`, `SHUTDOWN`
    Inactive      | `ACTIVATE`, `CLEANUP`, `SHUTDOWN`
    Active        | `DEACTIVATE`, `SHUTDOWN`
    Finalized     | None

`result` shall be `SUCCESS` if the node's life cycle successfully moved to the requested state and the results of any intermediate state functions were all `success`.

`result` shall be `TRANSITION_ERROR` if the result of any intermediate state function was anything other than `success` or an error was reported by any other means, and the node is now in the `ErrorProcessing` state or one of its successor states.

`result` shall be `WRONG_PREV_STATE` if the node's life cycle is not in the correct predecessor state for the requested transition, as described above.

### Provision of the interface

What provides the interface is implementation dependent.
It may be provided directly by the node object itself, by a container object, or by any other means as appropriate to the technologies being used to implement the node and ROS infrastructure.

It is expected that a common pattern will be to have a container class which loads a managed node implementation from a library and through a plugin architecture automatically exposes the required management interface via methods and the container is not subject to the lifecycle management.
However, it is fully valid to consider any implementation which provides this interface and follows the lifecycle policies a managed node.
Conversely, any object that provides these services but does not behave in the way defined in the life cycle state machine is malformed.

## Extensions

This lifecycle will be required to be supported throughout the toolchain as such this design is not intended to be extended with additional states.
It is expected that there will be more complicated application specific state machines.
They may exist inside of any lifecycle state or at the macro level these lifecycle states are expected to be useful primitives as part of a supervisory system.
