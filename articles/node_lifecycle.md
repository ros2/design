---
layout: default
title: Managed nodes
abstract:
  This article describes the concept of a node with a managed life cycle. It aims to document some of the options for supporting managed-life cycle nodes in ROS 2. It has been written with consideration for the existing design of the ROS 2 C++ client library, and in particular the current design of executors.
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

A managed life cycle for nodes allows greater precision over the state of ROS system. It will allow roslaunch to have greater control over launching a ROS system, and in particular will allow roslaunch to ensure that all components have been instantiated correctly before it allows any component to begin executing its behaviour. It will also allow nodes to be restarted or replaced on-line.

The most important concept of this document is that a managed node presents a known interface, executes according to a known life cycle state machine, and otherwise can be considered a black box. This allows freedom to the node developer on how they provide the managed life cycle functionality, while also ensuring that any tools created for managing nodes can work with any compliant node.


## Glossary

* Configuration: To alter aspects of a node's behaviour through known interfaces (services and topics). This includes setting parameters and connecting topics. Configuration may be performed multiple times during a node's life.
* Initialise: To shift a node from the created state to the inactive state, during which the node is prepared for execution. For example, memory buffers can be allocated.
* Activate: To shift a node from the inactive state to the active state. For example, internal variables can be re-initialised based on changed configuration parameters and hardware access may be obtained.
* Deactivate: To shift a node from the active state to the inactive state. For example, hardware access may be relinquished.
* Fatal error: An unforeseen error that cannot be handled by the robot application, requiring the node to be reset or destroyed.


## Life cycle

![The proposed node life cycle state machine](/img/node_lifecycle/life_cycle_sm.png "The proposed node life cycle state machine")

The behaviour of each state is as defined below.

### Created state

This is the life cycle state the node is in immediately after being instantiated.



Implementation note: In object-oriented languages, the onCreated method can be mapped to the class constructor.

## Valid transition out

- The node may transition to the `inactive` state by via the `initialise` transition.

### Inactive state

This state represents a node that is not currently performing any processing.

The main purpose of this state is to allow a node to be (re-)configured (changing configuration parameters, adding and removing topic publications/subscriptions, etc.) without altering its behaviour while it is running.

While in this state, the node will not receive any execution time to read topics, perform processing of data, respond to functional service requests, etc.

In the inactive state, any data that arrives on topics will not be read and discarded.
Any service requests to a node in the inactive state will not be answered (to the caller, they will fail immediately).

Implementation note: Causing the transition to the destroyed state can often be mapped to deletion of the node instance.

## Valid transition out

- A node may transition to the `destroyed` state via the `destroy` transition.
- A node may transition to the `created` state via the `cleanup` transition.
- A node may transition to the `active` state via the `activate` transition.
- A node may transition to the `Fatal error` state via the `error` transition.

### Active state

This is the main state of the node's life cycle. While in this state, the node performs any processing, responds to service requests, reads and processes data, produces output, etc.

If an error that cannot be handled by the node/system occurs in this state, the transition to the `fatal error` state is taken.

## Valid transition out

- A node may transition to the `inactive` state via the `deactivate` transition.
- A node may transition to the `Fatal error` state via the `error` transition.

### Fatal Error state

Known errors that can occur in a node should be considered designed behaviour of the system and so should be handled by the node where the error occurred and, if necessary, the surrounding nodes. The reason for this is that the system knows best how to handle errors that occur in its behaviour.

Any fatal errors (those which the node/system was not expecting and does not know how to handle) will cause a transition to the `fatal error` state. In this state, because the node does not know what to do in the presence of the error, it is not provided any execution time. Arriving data is not processed, service requests are not responded to, and no output is produced.

In the `Fatal Error` state, any data that arrives on topics will not be read and discarded.
Any service requests to a node in the `Fatal Error` state will not be answered (to the caller, they will fail immediately).

## Valid transition out

- A node may transition to the `destroyed` state via the `destroy` transition.

### Destroyed state

This is the state that a node goes through on its way to no longer existing.

In this state it is waiting to be deallocated and it will never be invoked again.

In the `Destryed` state, any data that arrives on topics will not be read and discarded.
Any service requests to a node in the `Destroyed` state will not be answered (to the caller, they will fail immediately).

Implementation note: In object-oriented languages, the onDestroyed method can be mapped to the class destructor.

## Transitions

All states have named transitions between them.
The node may register a callback for any transition.
This callback may return true or false if the transition succeeded.

### Initialize

If the callback fails the nodes stays in `Created` otherwise it goes to `Inactive`.

Initialisation of a node will typically involve those tasks that must be performed once during the node's life time, such as obtaining permanent memory buffers and setting up topic publications/subscriptions that do not change.

The node uses this to set up any resources it must hold throughout its life (irrespective of if it is active or inactive). As examples, such resources may include topic publications and subscriptions, memory that is held continuously, and initialising configuration parameters.

### Cleanup

If the callback fails it stays in `Inactive` otherwise it proceeds to `Created`.


### Activate
If the callback fails it stays in `Inactive` otherwise it proceeds to `Active`.

This may include acquiring resources that are only held while the node is actually active, such as access to hardware. Ideally, no preparation that requires significant time (such as lengthy hardware initialisation) should be performed in this callback.

### Deactivate
If the callback fails it stays in `Active` otherwise it proceeds to `Inactive`.

This provides the node a chance to clean up any resources it only holds while active.

### Error
The error transition always ends in `Fatal Error` regardless of the return code.

This provides the node with a chance to clean up resources it was using.


### Destroy

This is the node's last chance to clean up.

The error transition always ends in `Destroyed` regardless of the return code.




## Management Interface

A managed node will be exposed to the ROS ecosystem by the following interface, as seen by tools that perform the managing.
This interface should not be subject to the restrictions on communications imposed by the lifecycle states.

It is expected that a common pattern will be to have a container class which loads a managed node implementation from a library and through a plugin architecture automatically exposes the required management interface via methods and the container is not subject to the lifecycle management.
However, it is fully valid to consider any implementation which provides this interface and follows the lifecycle policies a managed node.
Conversely, any object that provides these services but does not behave in the way defined in the life cycle state machine is malformed.

These services may also be provided via attributes and method calls (for local management) in addition to being exposed ROS messages and topics/services (for remote management).
In the case of providing a ROS middleware interface, specific topics must be used, and they should be placed in a suitable namespace.


### Initialisation

The following service should be called to initialise the node. This service must be named `initialise`.

{% raw %}
    ---
    bool success
    string result_msg
{% endraw %}


### Activation

The following service should be called to activate the node. This service must be named `activate`.

{% raw %}
    ---
    bool success
    string result_msg
{% endraw %}


### Deactivation

The following service should be called to deactivate the node. This service must be named `deactivate`.

{% raw %}
    ---
    bool success
    string result_msg
{% endraw %}


### Clearing the error state

The following service should be called to clear the node's fatal error status. This is done when the node is to be re-activated in the hope that the error has gone away. This service must be named `reset`.

{% raw %}
    ---
    bool success
{% endraw %}

### State reporting

The following service should be called to query the current state of the node. The service must be named `get_state`.

`ComponentState.msg`:
{% raw %}
    ---
    int8 CREATED=0
    int8 INACTIVE=1
    int8 ACTIVE=2
    int8 ERROR=3
    int8 DESTROYED=4

    int8 state
{% endraw %}

The value of `state` must be one of the constants defined in the service definition.

### Lifecycle events

A topic should be provided to broadcast the new life cycle state when it changes.
This topic must be latched.
The topic must be named `current_state` it will carry both the end state and the transition, with return code.
It will publish ever time that a transition is triggered, whether successful or not.

`ComponentTransition.msg`;
{% raw %}

    int8 INITIALIZE=0
    int8 ACTIVATE=1
    int8 DEACTIVEATE=2
    int8 ERROR=3
    int8 DESTROY=4
    int8 RESET=5

    int8 transition
    bool success

{% endraw %}


{% raw %}
    ComponentTransition transition
    ComponentState resultant_state

{% endraw %}


## Node Management

There are several different ways in which a managed node may transition between states.
Most state transitions are expected to be coordinated by an external management tool which will provide the node with it's configuration and start it.
The external management tool is also expected monitor it and execute recovery behaviors in case of failures.
A local management tool is also a possibility, leveraging method level interfaces.
And a node could be configured to self manage, however this is discouraged as this will interfere with external logic trying to managed the node via the interface.

There is one transition expected to originate locally, which is the `ERROR` transition.

A managed node may also want to expose arguments to automatically configure and activate when run in an unmanaged system.


## Extensions

This lifecycle will be required to be supported throughout the toolchainm as such this design is not intended to be extended with additional states.
It is expected that there will be more complicated application specific state machines.
They may exist inside of any lifecycle state or at the macro level these lifecycle states are expected to be useful primitives as part of a supervisory system.
