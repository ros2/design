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
A tool called dynamic_reconfigure was developed to address this use case.
It provided a service based interface to interact with parameters of other nodes.

### Other resources

Other resources related to the parameter design process for ROS 2.0 include:

- Gonzalo's research on parameters.

  - Discussion: <https://groups.google.com/forum/#!topic/ros-sig-ng-ros/YzCmoIsN0o8> and <https://groups.google.com/forum/#!searchin/ros-sig-ng-ros/parameter/ros-sig-ng-ros/fwDBcei5Ths/L6ORPfjUDXYJ>
  - Prototype: <https://github.com/abellagonzalo/dynamic_config>
  - Final Notes:  [http://wiki.ros.org/sig/NextGenerationROS/Parameters](http://wiki.ros.org/sig/NextGenerationROS/Parameters)

- Thibault's nodeparam draft REP: <https://github.com/tkruse/rep/blob/nodeparam/nodeparam-REP.rst>

## Ideal System

It is useful to consider an ideal system to understand how it would work and how it would relate to the current system.
An ideal system would support for the combined use cases of ROS 1.0's built-in parameters as well as ROS 1.0's dynamic parameters system.
Based on that criteria an ideal system would be able to:

- Set parameter values

  This includes setting groups of parameter values atomically.

- Get parameter values

  This includes getting groups of parameter values atomically.

- Unset parameter values

  This includes unsetting groups of parameter values atomically, but may be a special case of setting groups of parameters atomically.

- List currently set parameters

  Since the number of parameters might be large this needs to be able to provide subsets of the parameters.
  E.g. the parameters could be queried incrementally for a tree-like GUI.

- Provide notifications when parameters are added and removed or their value has been changed

- Reject changes to parameter values

  This implies that some entity has the authority to reject or accept a change based on arbitrary criteria.
  This would also include the ability to convey at least part of the criteria for the acceptance of a change to external actors.
  For example, communicating the range for an integer or a few choices for a string.
  This type of information would be used to generate generic user interfaces, but might not capture all criteria.
  Since the validation criteria can be arbitrary complex and can potentially not be communicated to a client the parameter server could offer to validate an atomic set of parameters and respond with a boolean flag if the values would be accepted (based on the current criteria).
  Obviously the result might be different when the values are set shortly after but it would allow to implement validators in e.g. a GUI generically.

- Provide visibility into what parameters are expected to pass validation vs be rejected

  When updating a value it can be valuable to know if the parameter update would be accepted without actually requesting the change to happen.

- Provide clear rules on the lifetime of a parameter

  These rules would define what the lifetime of the parameter will be and what conditions will clear its value.

- Address all parameters without ambiguity in the names

  For example, one of the challenges of the current system is that there is a naming ambiguity between nodes and parameters `/foo/bar/baz` could be a node `/foo/bar/baz` or a private parameter `baz` on node `/foo/bar`.

- Be logged for playback and analysis

## Proposed Approach

To cover the feature set above, the ROS 2.0 parameter system is proposed as follows.

### Parameters Hosted in Nodes

For the sake of validating parameter lifecycle, all parameters will be hosted on a node.
Their lifetime will be implicitly tied to the nodes lifetime.
The node will be responsible for validating current values.
The node could also implement persistence of the parameters to reload the previous values after being restarted.

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

The datatypes are chosen as non-complex datatypes, as defined in the [interface definitions article](articles/interface_definition.html)
The full complement of datatypes of different bitdepth and unsigned types are avoided to allow interpretation from text based configuration files.

`bytes` are included to allow the storage of binary blobs.
It's use is not recommended but can be very convenient, and explicitly supporting it is better than having people try to abuse other datatypes such as strings.

### Required functionality

Each node is responsible for providing the following functionality.

- Get Parameters

  Given a list of parameter names it will return the parameter values.

- Set Parameters

  Given a list of parameter names, it will request an update of the values subject to validation of the values.
  The updated values can include unsetting the value.
  It will provide an API that can atomically update a set of values such that if any of the values fail validation, none of the values are set.
  The success or failure of this call will be available to the client for each update unit.
  Validation of the values is expected to return as quickly as possible and only be related to accepting or rejecting the set request.
  It should not take into account how the changed value may or may not affect ongoing system performance of the node or the greater system.

- List Parameters

  Provide a list of parameter names which are currently set.

- Describe Parameters

  Given a list of parameter names, return their datatype.

This functionality will be exposed through a user API which will support both local API calls as well as invocations on remote nodes via a ROS Service API.

### Parameter update validation

The node can validate incoming parameter changes and either accept or reject them.

### Backwards compatibility Parameter Server like behavior

There are use cases where the older behavior with parameter server was useful.
Both persisting beyond the duration of a specific node is valuable as well as having parameters with no specific association to a node which would potentially own or validate the values.
To this end we propose to write a simple node which emulates the policy of the ROS 1.0 parameter server: it runs in namespace `/` and simply accepts all changes requested.
The parameters held by this parameter server node would persist for the lifetime of the parameter server node.
Specific instances could be launched in different namespaces to support different parameter persistence models.

### Search parameter behavior

A pattern developed in ROS 1.0 was the `searchParam` mode where a parameter could be set in a namespace and the parameter query would walk up the namespace to search for the parameter.
A similar behavior can be implemented by allowing the search parameter implementation to walk across the different nodes in hierarchical order.

### Parameter API

The client libraries will provide the following API for interfacing with the Core Parameter API for both local and remote nodes including return codes.

### Parameter Events

Each node will provide a topic on which parameter events will be published.
This topic is to support monitoring parameters for change.
It is expected that client libraries will implement the ability to register callbacks for specific parameter changes using this topic.

### Logging and playback

When logging an entire system, the parameter changes can be logged via standard topic recording of the events channel.
An implementation of the playback mechanism could listen to the parameter event streams and invoke the set parameter calls via the remote API.

## Current implementation

The above specification has been prototyped; the implementation can be found in [rclcpp](https://github.com/ros2/rclcpp).
The definition of the services to use for interacting remotely are contained in the [rcl_interfaces package](https://github.com/ros2/rcl_interfaces)

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

Going forward, there are still topics to discuss and flesh out either in this document or others.
A few to highlight:

### Parameter initialization

There are several ways to load parameters at startup including command line arguments, roslaunch arguments, and potentially parameter files.
This is something which should be addressed in conjunction with the new launch system.

### Support for arrays of primatives

During the API discussions supporting arrays of primatives was discussed and deferred.
Adding support for arrays in the interface is relatively straight forward.
It slightly increases the complexity of the API for users, but can support several use cases.
A use case for arrays of numbers is expressing a matrix or vector, addressing each position in a matrix by some sort of row-column naming scheme can get very cumbersome.

### Predeclared interface to support static checking/validation

The ability to declare an API which can help with static checks and prevent logical errors which arrise from setting the wrong parameter based on a typo.
The node could enforce this by rejecting unexpected names, but there are some cases where knowing the expected parameter names would be useful for developer tools.
