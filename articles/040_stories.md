---
layout: default
title: Stories driving the development
permalink: articles/stories.html
abstract:
  This article captures some stories which drive the direction of features in ROS.
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

The article enumerates a few stories which sketch what will be possible with ROS in the future.
The list is by no means exhaustive.

## Push ROS into the hardware nodes

A couple of design decisions in ROS 1 make it difficult to integrate hardware devices (e.g. a sensor or actuator) natively into the ROS graph.
The master is a central entity in the ROS graph and needs to be started before any of the nodes.
Also the communication between the nodes and the master is done using XML-RPC which poses a significant dependency when being implemented on small resource constraint systems / micro controllers due to its recursive unbounded nature.
Instead commonly a driver is being used which uses a custom protocol to communicate between the device and the computer and exposes a ROS interface on the computer.

In the future it should be possible to implement the ROS protocol directly on the devices embedded system.
A ROS-enabled device would then be able to discover other nodes automatically and expose a ROS interface (composed of publisher, services and parameters).
The adoption of an industry standard like DDS as well as the decentralized nature of the middleware are important pieces to enable this.

The advantage of this approach is twofold:

- The integration effort to build a new system is being reduced since no custom drivers are required anymore.
- By implementing a specific ROS-based interface the devices of specific class can easily be substituted without the need to spend time on integrating the software / hardware from e.g. a different vendor.

## Delay decision on process layout to deploy time

In ROS 1 nodes commonly use the `Node` API and implement their own `main` function.
Only a few packages leverage the `Nodelet` API and compile their components into shared libraries instead.
The developer has to choose between these two different APIs and converting from using one to the other requires some non trivial effort.

In ROS 2 the recommended way of writing nodes will be similar to nodelets.
This will enable the user to decide at deploy time how a set of nodes should be started.
On the one hand each node could be started in a separate process to ease debugging of them individually.
On the other multiple nodes can be aggregate in a single process to benefit from the performance advantages possible by in-process message passing.

## Deterministic launch

In ROS 1 the launch system is only able to start a set of processes.
It doesn't provide any more feedback beyond the information if a process has finished.
For complex systems this is often not sufficient.
It is quite common that developers write their processes in a way which either waits a fixed amount of time or waits for a custom flag which signals that "everything" is ready before starting to process data.
Also when "something" goes wrong during startup the attentive developer will notice and manually restart the launch process.
Obviously in use cases where the software is being used on a product this is not feasible either.

The goal is to enable the launch system to ensure a deterministic result.
In order to achieve this the launch system needs to be able to introspect the started processes (which are usually ROS nodes) and ensure that they have been started successfully, finished initializing, and have established all require communication channels with other entities.
This will require a minimalistic life cycle as well as the ability to introspect the state from the launch system.
It would even be possible to compare the started ROS graph with a "known good" state to ensure the system has been started according to the expectations.

## Introspection, Orchestration, and beyond

In complex systems the observation of the system and its dynamic configuration becomes more important.
In ROS 1 nodes do not have any specific state and only a few components (like the nodelet manager) provide an interface to get information or even manipulate the running system.

Once a ROS system is using the above features ([Nodelet-style nodes](#delay-decision-on-process-layout-to-deploy-time) and an [accessible life cycle](#delay-decision-on-process-layout-to-deploy-time)) the abilities for introspection as well as orchestration can be leverages to build more complicated systems.
The following a only a few example scenario enabled by the comprehensive introspection and debugging capabilities:

- The state of each node can be monitored and based on the information specific actions can be triggered.
  E.g. certain error conditions can be signaled to the user, or in case fall back behaviors are available they can be selected to provide a degraded continuation of the system.

- The resource usage of the system could be monitored at runtime.
  Based on the available information the system can be dynamically reconfigured e.g. by enabling / disabling specific nodes, altering any kind of parameter (e.g. frame rate, or any other threshold).

- In case of a single process containing multiple nodes crashing the system should decide to not only restart these nodes but also separate them into individual processes to isolate the problem in future cases.

- If the system load on a single computer exceeds a certain threshold an orchestration entity can trigger the following steps: pause all nodes through the life cycle interface, shutdown a specific node, spawning the node on a separate machine and passing the same configuration / parameters, and once all communication channels have been established resume all nodes.
