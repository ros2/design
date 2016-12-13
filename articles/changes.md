---
layout: default
title: Changes between ROS 1 and ROS 2
permalink: articles/changes.html
abstract:
  This article provides an overview about the changes being made in ROS 2 compared to ROS 1.
published: true
author: '[Dirk Thomas](https://github.com/dirk-thomas)'
categories: Overview
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Preface

Each change is described as briefly as possible but giving enough context and rationale for a reader familiar with ROS 1.
If further external information is available (e.g. other articles) it should be linked to.

<div class="alert alert-info" markdown="1">
  Some of the described features are not yet available and are marked with ⏳.
</div>

## Platforms and dependencies

### Platforms

ROS 1 is only being CI tested on Ubuntu.
It is actively supported by the community on other Linux flavors as well as OS X.

ROS 2 is currently being CI tested and supported on Ubuntu Xenial, OS X El Capitan as well as Windows 10 (see [ci.ros2.org](http://ci.ros2.org/)).

### Languages

#### C++ standard

The core of ROS 1 is targeting C++03 and doesn't make use of C++11 features in its API.
ROS 2 uses C++11 extensively and uses some parts from C++14.
In the future ROS 2 might start using C++17 as long as it is supported on all major platforms.

#### Python

ROS 1 is targeting Python 2.
ROS 2 requires at least Python version 3.5.

### Reusing existing middleware

ROS 1 uses a custom serialization format, a custom transport protocol as well as a custom central discovery mechanism.
ROS 2 has an [abstract middleware interface](http://design.ros2.org/articles/ros_middleware_interface.html), through which serialization, transport, and discovery is being provided.
Currently all implementations of this interface are based on the DDS standard.
This enables ROS 2 to provide various [Quality of Service policies](http://design.ros2.org/articles/qos.html) which improve communication over different networks.

## Build system

For more information about the build system please see the [ament](http://design.ros2.org/articles/ament.html) article.

### Support other build systems beside CMake

Every ROS package is a CMake project.
In ROS 2 other build systems can be easily supported.
For now the build tool supports plain Python packages beside CMake.

### Python packages

In ROS 1 a package with Python code can only use a small subset of the features available in setup.py files since the setup.py file is being processed by custom logic from within CMake.
In ROS 2 a Python package can use anything in setup.py files, e.g. entry points since they are being invoked with `python3 setup.py install`.

### Environment setup

In ROS 1 the build tool generates scripts which must be sourced in order to setup the environment before being able to use the built ROS packages.
This approach only works when the ROS packages are being built with ROS specific build tool.

In ROS 2 the environment setup is separated into package-specific scripts and workspace-specific scripts.
Each package provides the necessary scripts to make itself usable after being built.
The build tool only invokes the workspace-specific scripts which then call the package-specific scripts.

### No non-isolated build

In ROS 1 multiple packages can be built in a single CMake context.
While this speeds up the build step, every package needs to ensure that cross package target dependencies are defined correctly.
Additionally all packages share the same namespace which leads to colliding target names, etc.

In ROS 2 only isolated builds are supported, i.e. every package is built separately on its own.
The install spaces can be either isolated or merged.

### No devel space

In ROS 1 packages can be built without installing them.
From the devel space in combination with the source space the system is already usable.
But every package has to actively support the devel space, e.g. in environment hooks and CMake code.

In ROS 2 a package must be installed after building it before it can be used.

One reason for the devel space in ROS 1 is to enable the developer to change files, e.g. Python code or launch files, and use the modified code directly without the need to rebuild the package.
This benefit is preserved in ROS 2 by optionally replacing copy operations in the install steps with symlinks.

### Support catkin_simple use case

In ROS 1 the package catkin_simple is aiming to make writing the CMake code of ROS packages easier.
In many cases it is not achieving this goal which is often due to restrictions of the design necessary for support features like the devel space.

In ROS 2 the CMake API was restructured to support this use case.

### Minimal support for packages without manifest

In ROS 1 only packages with a manifest file are considered by the build system.
In ROS 2 it is possible to detect packages with supported build system in folders without a manifest file.
If the package follows common practice it might even be possible to detect some of the missing meta information (like dependencies).

## Messages, Services

For more information please see the [ROS interface definition](http://design.ros2.org/articles/interface_definition.html) article.

### Separated namespaces in C++

In ROS 1 .msg and .srv files can have the same name but the generated code collides.
The same is the case for the request and response parts of services.

In ROS 2 the generated code uses separate namespaces to guarantee it is collision-free.

### Same names in Python

The generated Python code for messages and service is currently using the same module and class names in ROS 1 and ROS 2.
Therefore they can not be imported both in a single application.
This decision might be revisited if required.

### Optional default values in message definitions

In ROS 2 primitive values in messages can now have default values, set when the message is constructed.
Default values for non-primitive fields, i.e. array of strings, nested messages, are not yet possible (⏳).

### Optional upper bounds for arrays and strings

This is necessary in order to calculate the maximum size of a message in memory, which allows for preallocation of messages with a dynamic sizes.
This is useful for performance and for use cases like real-time.

### Unify duration and time types

In ROS 1 the duration and time types are defined in the client libraries.
The member names of the data structures are different in C++ (sec, nsec) and Python (secs, nsecs).

In ROS 2 these types are defined as messages and therefore are consistent across languages.

### Remove sequence field from Header message

The field has been deprecated for a long time and was not set consistently in ROS 1.

## Client libraries

### Across languages

#### Topic namespaces (⏳)

Currently ROS 2 does not support namespaces in topic names.
This is mostly due to restrictions of valid characters in DDS topic names.
A [design document](http://design.ros2.org/articles/topic_and_service_names.html) describes how this should be added in the future.

#### Notifications

In ROS 1 all information about the ROS graph must be polled from the master.
In ROS 2 changes will be published instead, e.g. notifications if a parameter has been changed.

#### Components with life cycle

In ROS 1 every node usually has its own main function.
In ROS 2 it is recommended to subclass from a component which has a life cycle.

The life cycle can be used by tools like roslaunch to start a system composed of many components in a deterministic way (⏳).

For more information please see the [node life cycle](http://design.ros2.org/articles/node_lifecycle.html) article.

#### Parameters and Dynamic Reconfigure

In ROS 1 global parameters and node-specific dynamic reconfigure parameters are two separate concepts.
In ROS 2 a unified approach is being used.
It is similar to dynamic reconfigure and a node named "global parameter server" (⏳) will accept requests to set values unconditionally.
In ROS 1 all this information needs to be polled for changes in ROS 2 changes will be published to notify other entities.

For more information please see the [parameter design](http://design.ros2.org/articles/ros_parameters.html.html) article.

#### Actions (⏳)

ROS 2 currently doesn't have the concept of actions.
It will be added in the future as a combination of a preemptable server and a feedback publisher.

#### Threading model

In ROS 1 the developer can only choose between single-threaded execution or multi-threaded execution.
In ROS 2 more granular execution models are available in C++ (e.g. across multiple nodes) and custom executors can be implemented easily.
For Python the execution models haven't been implemented yet.

#### ROS Graph

In ROS 1 nodes and topics can be remapped at startup time only.
In ROS 2 support for remapping hasn't been implemented yet (⏳).
The goal is to enable remapping as well as aliasing not only during startup time but also during runtime.

In ROS 1 node names are unique and this is being enforced by shutting down existing nodes when a new node with the same name is started.
In ROS 2 the uniqueness of node names is not yet enforced.

### C and C++

#### Support for real-time

ROS 1 does not support writing real-time code but relies on external frameworks like Orocos.
In ROS 2 it will be possible to write real-time nodes when using a proper RTOS and with carefully written user code.

### C++

#### Node vs. Nodelet

In ROS 1 the API's for nodes and nodelets are different and requires the developer to decide the mapping of nodes to processes at programming time.
In ROS 2 it is recommended to compile each component into a shared library which can then be loaded in a separate process or share the same process with other components (like ROS 1 nodelets).
This enables to choose the process layout at deploy-time.

#### Allow multiple nodes per process

In ROS 1 it is not possible to create more than one node in a process.
This is due to the API itself but also because of internal implementation decisions.
In ROS 2 it it possible to create multiple nodes in a process.

## Tools

### roslaunch (⏳)

In ROS 1 roslaunch files are defined in XML with very limited capabilities.
In ROS 2 launch file are written in Python which enables to use more complex logic like conditionals etc.
The current state only provides minimal functionality to run tests using multiple processes.

## Resource lookup

In ROS 1 various resources (packages, messages, plugins, etc.) are looked up by crawling the file system based on the ROS_PACKAGE_PATH.
This can cause poor performance when the trees in the ROS_PACKAGE_PATH are large, and caching produces inconsistent state.

In ROS 2 resources can be registered at an index at compile time and then be queried efficiently at runtime.
For more information please see the documentation of the [resource index](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md).

## Packaging

### ABI versioning (⏳)

ROS 1 rebuilds all downstream packages since it assumes ABI incompatibility.
To avoid this significant overhead a ROS 2 package should be able to declare its ABI to avoid rebuilding downstream packages whenever possible.

### Binary packages for Windows (⏳)

ROS 1 can only be built from source on Windows (which also only works for a few ROS packages and is not supported).
ROS 2 will provide binary package based on Chocolatey.
