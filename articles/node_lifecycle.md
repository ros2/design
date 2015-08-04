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

The node may transition to the `inactive` state by being initialised. Initialisation of a node will typically involve those tasks that must be performed once during the node's life time, such as obtaining permanent memory buffers and setting up topic publications/subscriptions that do not change.

Implementation note: In object-oriented languages, the onCreated method can be mapped to the class constructor.

### Inactive state

This state represents a node that is not currently performing any processing.

The main purpose of this state is to allow a node to be (re-)configured (changing configuration parameters, adding and removing topic publications/subscriptions, etc.) without altering its behaviour while it is running.

[To discuss: Should there be a callback for when configuration is changed, e.g. altered parameters, new topics, etc.?]

While in this state, the node will not receive any execution time to read topics, perform processing of data, respond to functional service requests, etc.

In the inactive state, any data that arrives on topics will not be read. Whether or not data is queued will depend on the QoS parameters of the topics involved. [In other words, to queue data or not is up to each topic.]

Any service requests to a node in the inactive state will not be answered (to the caller, they will fail immediately).

Entering the `inactive` state from the `created` state requires running the node's `onInitialised` method. The node uses this to set up any resources it must hold throughout its life (irrespective of if it is active or inactive). As examples, such resources may include topic publications and subscriptions, memory that is held continuously, and initialising configuration parameters.

Entering the `inactive` state from the `active` state requires running the node's `onDeactivated` method. See the `active` state description for more information.

Entering the `inactive` state from the `fatal error` state requires running the node's `onReset` method. See the `fatal error` state description for more information.

If an error occurs while in this state that cannot be handled by the node, then the transition to the `fatal error` state is taken.

The node may also exit this state by being destroyed, in which case the `destroy` transition is taken.

Implementation note: Causing the transition to the destroyed state can often be mapped to deletion of the node instance.

### Active state

This is the main state of the node's life cycle. While in this state, the node performs any processing, responds to service requests, reads and processes data, produces output, etc.

Entering the `active` state requires running the node's `onActivated` method. This is used to prepare the node for execution. This may include acquiring resources that are only held while the node is actually active, such as access to hardware. Ideally, no preparation that requires significant time (such as lengthy hardware initialisation) should be performed in this callback.

To shut down a node, it must first be deactivated. This will cause the `onDeactivate` method to be called, allowing the node to clean up any resources it only holds while active.

If an error that cannot be handled by the node/system occurs in this state, the transition to the `fatal error` state is taken.

### Fatal Error state

Known errors that can occur in a node should be considered designed behaviour of the system and so should be handled by the node where the error occurred and, if necessary, the surrounding nodes. The reason for this is that the system knows best how to handle errors that occur in its behaviour.

Any fatal errors (those which the node/system was not expecting and does not know how to handle) will cause a transition to the `fatal error` state. In this state, because the node does not know what to do in the presence of the error, it is not provided any execution time. Arriving data is not processed, service requests are not responded to, and no output is produced.

On entering the `fatal error` state, the node's `onError` method must be executed. This provides the node with a chance to clean up resources it was using.

When a node is in this state, roslaunch may choose to reactivate the node and hope the error has gone away (accomplished by clearing the error and then going through the standard activation process), restart the node (and hope the error does not repeat), or it may choose to replace it with an alternative implementation that provides the same functionality. Restarting or replacing the node can be accomplished destroying it and creating a new instance of its replacement.

### Destroyed state

This is the state that a node goes through on its way to no longer existing.

A node must be deactivated before it can be destroyed.

On entering this state, the node's `onDestroyed` method must be run. This is the node's last chance to clean up.

Implementation note: In object-oriented languages, the onDestroyed method can be mapped to the class destructor.




## Interface

A node must provide the messages and services defined in this section to qualify as a managed node. These services define the external interface of a managed node as seen by tools that perform the managing. As such, it should be possible to consider any object that provides these services correctly to be a managed node.

Conversely, any object that provides these services but does not behave in the way defined in the life cycle state machine is malformed.

These services may be provided via attributes and method calls (for local management) or via ROS messages and topics/services (for remote management). In the case of providing a ROS middleware interface, specific topics must be used, and they should be placed in a suitable namespace.

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

A topic should be provided to broadcast the new life cycle state when it changes. This topic must be latched. The topic must be named `current_state`
