---
layout: default
title: Parameter API design in ROS
permalink: articles/ros_parameters.html
abstract:
  This article is proposed design for the interfaces for interacting with parameters in ROS 2.0.
  We focus here on specifying the system design and leave the implementation unspecified.
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

## Background

In ROS 1 the parameters were implemented in a 'blackboard model' with unrestricted read and write access from all nodes.
The data model proved useful in many cases, but there were many cases where the lack of control or ownership proved to be a problem.
One of the common shortcomings was for setting parameters on drivers.
A tool called dynamic_reconfigure was developed to service this use case.
It provided a service based interface to interact with parameters of other nodes.

### Other resources

Other resources related to the parameter design process for ROS 2.0 include:

- Gonzalo's research on parameters.
  - Discussion: https://groups.google.com/forum/#!topic/ros-sig-ng-ros/YzCmoIsN0o8 and https://groups.google.com/forum/#!searchin/ros-sig-ng-ros/parameter/ros-sig-ng-ros/fwDBcei5Ths/L6ORPfjUDXYJ
  - Protoype: https://github.com/abellagonzalo/dynamic_config
  - Final Notes:  [http://wiki.ros.org/sig/NextGenerationROS/Parameters](http://wiki.ros.org/sig/NextGenerationROS/Parameters)
- Thibault's nodeparam draft REP: https://github.com/tkruse/rep/blob/nodeparam/nodeparam-REP.rst

## Ideal System

It is useful to consider the ideal system to understand how it relates to the current system and how a new system could work.
We'd like to support both the use cases of ROS 1.0 built in parameters as well as dynamic parameters.
An ideal parameter system would have the qualities laid out in the following paragraphs.

### Accept parameter values

At the most basic, a parameter system must be able to accept the setting of parameter values.
An extension is to accept a group of parameters updated atomically.
This allows you to update coupled parameters such as PID gains without worring about invalid intermediate states.

### Return parameter values

Also at the core functionality return the value of one or more parameters which have previously been set.
If the value of a parameter is requested which has not been set it will explicitly return `unset`.
Parameters can be queried individually or in groups atomically so as to get a consistent state.

### Unset a parameter value

If a parameter has been set it should be able to be unset.

### Introspection of available parameters

Provide a list of currently set parameters.

### Notifications of change

When a parameter changes value it should be possible to get a notification.

### Reject parameter changes

If a parameter change is requested to an invalid value, the parameter server should be able to reject the change.
Related to this, it should also be able to communicate the preconditions which are necessary but not sufficient to determine the acceptance of a change.

### Introspection of expected parameters

When using parameters, providing a list of expected parameters can prevent accidentally setting parameters with typos in their names etc.

### Control parameter lifetime

When parameters are set, there should be an understanding of what the lifetime of the parameter value will be and what conditions will clear its value.

### Unambigious naming

All parameters should be unambiguously addressable.

For example, one of the challenges of the current system is that there is a naming ambiguity between nodes and parameters `/foo/bar/baz` could be a node `/foo/bar/baz` or a private parameter `baz` on node `/foo/bar`.

### Logging

All changes to parameters should be visible when logging.
And when playing back a log file the parameter changes should be able to be reapplied.

## Proposed Approach

To cover the feature set above, the ROS 2.0 parameter system is proposed as follows.

### Parameters Hosted on nodes

For the sake of validating parameter lifecycle, all parameters will be hosted on a node.
Their lifetime will be implicitly tied to the nodes lifetime.
The node will be responsible for validating current values.

### Parameter addressing

All parameters will be addressed by two elements: the full node name and the parameter name.

### Supported datatypes

Each parameter consists of a key and a value.
The key is a string value.
The value can be one of the following datatypes:

- `float64`
- `int64`
- `string`
- `bool`
- `bytes[]`

The datatypes are chosen as non-complex primatives.
The full complement of datatypes of different bitdepth and unsigned types are avoided to allow interpretation from text based configuration files.

`bytes` are included to allow the storage of binary blobs.
It's use is not recommended but can be very convenient, and explicitly supporting it is better than having people try to abuse other datatypes such as strings.

### Core API

Each node is responsible for providing the following functionality.

- **Get Parameters**
  Given a list of parameter names it will return the parameter values.
- **Set Parameters**
  Given a list of parameter names, it will request an update of the values subject to validation of the values.
  The updated values can include unsetting the value.
  It will provide an API that can atomically update a set of values such that if any of the values fail validation, none of the values are set.
  The success or failure of this call will be available to the client.
- **List Parameters**
  Provide a list of parameter names which are currently set.
- **Describe Parameters**
  Given a list of parameter names, return their datatype.

This functionality will be exposed through local API calls as well as a ROS Service call API for remote operations.

### Parameter update validation

The node will allow for the registration of a callback for custom parameter value validation.

### Backwards compatibility Parameter Server like behavior

There are use cases where the older behavior with parameters persisting beyond the duration of a specific node are valuable.
To this end we propose to write a simple node which emulates the policy of the ROS 1.0 parameter server: it runs in namespace `/` and simply accepts all change requests.

### Parameter API

The client libraries will provide an API for interfacing with the Core Parameter API for both local and remote nodes including return codes.

### Parameter Change notification

Each node will provide a topic on which parameter changes will be published.
This topic is to support monitoring parameters for change.
It is expect that client libraries will implement the ability to register callbacks for specific parameter change notifications using this topic.

### Logging and playback

When logging an entire system, the parameter changes can be logged via standard topic recording of the events channel.
An implementation of the playback mechanism could listen to the parameter event streams and invoke the set parameter calls via the remote API.

## Current implementation

The above specification has been prototyped; the implementation can be found in [rclcpp](https://github.com/ros2/rclcpp).
The defintion of the services to use for interacting remotely are contained in the [rcl_interfaces package](https://github.com/ros2/rcl_interfaces)

### Unimplemented

Currently there a few parts of the specification unimplemented.

- No parameter subscription registration.
  The events are published, but there is not way to register a callback for changes to a specific parameter.
  You can currently register for a callback on all changes for parameters of a node.
- The ability to register callback to validate parameter updates prior to them being updated is not available.
- There has been no work on logging and playback of logged parameter changes.
- The ability to list and get expected validation policy has not been implemented.
  It is expected to operate at a slightly higher level than parameters, and it possibly will be related to the component life cycle.

## Topics not covered at the moment

Going forward, there are still topics to discuss and flesh out either in this document or others. A few to highlight.

### Parameter initialization

There are several ways to load parameters at startup including command line arguments, roslaunch arguments, and potentially parameter files.
This is something which should be addressed in conjunction with the new launch system.

### Support for arrays of primatives

During the API discussions supporting arrays of primatives was discussed and deferred.
Adding support for arrays in the interface is relatively straight forward.
It slightly increases the complexity of the API for users, but can support several use cases.
A use case for arrays of numbers is expressing a matrix or vector, addressing each position in a matrix by some sort of row-column naming scheme can get very cumbersome.
