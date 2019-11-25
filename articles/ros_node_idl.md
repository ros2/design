---
layout: default
title: ROS 2 Node Interface Definition Language
permalink: articles/ros2_node_interface_definition_language.html
abstract:
  This article specifies the ROS 2 Interface Definition Language, a simple and standardized manner to export the complete interface (action/message/parameter/service) of node(s) in a package.
author: >
  [Jérémie Deray](https://github.com/artivis),
  [Kyle Fazzari](https://github.com/kyrofa)
published: true
categories: Interfaces
---

{:toc}


# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}


## Background

Every ROS 2 node has an associated interface that describes how it communicates with other nodes, as well as how it is to be configured.
This interface is defined in code, and consists of:

- Actions (server or client)
- Parameters
- Services (server or client)
- Topics (publisher or subscriber)

The information contained within that interface is obviously very valuable on different levels and from different perspectives.
While it is usually readily available to a developer looking at the code, it cannot reliably be automatically extracted.
It therefore calls for the creation of a standardized way to explicitly define and export this information.

This article defines a high-level abstraction allowing upstream packages to specify the communication requirements of the nodes in the package, such that the final user, be it a developer or a static analysis tool, can benefit from it.
The Interface Definition Language (IDL) specified in the next section is meant to be distributed alongside its associated package.
This is true for the package source and also for the generated release. Whether the interface is declared or not is up to the package author and should not prevent the correct execution of any system pre-existing the IDL.
Similarly, the declared interface may be only partial and allow for the full use of pre-existing systems and the use of dependent systems on the parts covered by the partial interface.

## Motivation

While initially being approached from a ROS 2 Security perspective, the abstraction level of the IDL allows for the developments of other functionalities and tools.

### Security motivation

Thanks to [DDS-Security][dds_security] and [SROS 2][sros2_design], security is at the heart of ROS 2.
DDS enforces access control using a signed permissions document for each domain participant (see the [DDS-Security spec][dds_security], section 9.4).
In SROS 2, that permissions document is generated from a ROS-specific [XML][xml_wiki] policy file that may include the permissions for one or many nodes.

Currently policy files can be created in one of two ways:
- Written by hand.
- A snapshot of the live ROS 2 graph can be taken and written into a policy that covers its current state via `ros2 security generate_policy`.

While the first option is obviously very tedious and error-prone, the second only partially alleviates the burden due to the fact that it cannot fully cover the dynamic nature of a ROS 2 graph and all of its interactions.
More problematic than these issues, though, is that both options put the onus of security squarely on the shoulders of end users.
This introduces two problems:

While developers will be able to define the set of rules securing their own ROS 2 nodes, the nodes developed in-house are often outnumbered by upstream components when it comes to the entire node graph, and the developers are typically not experts in every component being used.
Without that expertise, the entire node graph cannot be properly locked down.
Consider a complex and popular upstream component, perhaps parts of the navigation stack.
**Every end user** of this component must duplicate the effort of attempting to properly lock it down.

If ROS 2 provided a way for upstream package authors to specify the interface required by the nodes in their package, and if the tools to generate security policies from that interface existed, neither of these problems would exist.

### Other motivations

Outside of security, there are several fascinating possibilities unlocked by having such an interface.
For example, consider how this could impact [ROS 2 launch][launch_ros].
It would be able to statically (i.e. before running anything) determine if parameter names or remappings are incorrect, among other similar sanity checks.
Another example of the usefulness of having a static interface is the ability to create graphical tools for putting a ROS system together.
Yet another example would be an additional feature in `ros2 pkg create` that would allow a developer to hand it an IDL and have it generate scaffolding for a node with that interface.

These examples are only a subset of use-cases made possible by such an interface.
It's clear that this is useful well beyond security.

## Package interface

How do upstream packages specify their interface requirements?
Through a high-level description of all the actions, parameters, services and topics, provided or required, by each node within the package.

The package interface is defined in a separate [XML][xml_wiki] file and exported from the `package.xml` using [REP 149's export mechanism][rep149_export], thereby avoiding pollution of the package manifest.
The interface may cover only a subset of nodes in a package, as long as the nodes that _are_ covered are done so completely.

Here is an example IDL for a package containing two nodes:

{% include_relative ros2_node_idl/interface_declaration.xml %}

Once an IDL file is written, it is exported from the package manifest:

{% include_relative ros2_node_idl/package.xml %}

### Schema for `package.xml`'s export tag

#### `interface`

This is how the package exports its defined IDL for other tools to consume.

Attributes:

**path**:  Path to XML file containing IDL

Note that the introduction of the `interface` tag within the `export` tag of the package manifest raises a small difficulty with regards to the [REP 149][rep149_export].
The REP specifies:

> To avoid potential collisions, an export tag should have the same name as the package which is meant to process it. The content of that tag is up to the package to define and use.

Considering the high level abstraction of the IDL, the `interface` tag is not meant for a specific package but rather declares intrinsic properties for anyone to process it.
However, the keyword is likely solidly descriptive enough to either be accepted as falling under [REP 149][rep149_export] or else to motivate an amendment of the REP.

### IDL Schema

#### `interface`

This tag contains the package interface, made up of a collection of node interfaces.
Root tag of the IDL file. There must be only one <interface> tag per IDL file.

Attributes:
- **version**: version of schema in use, allowing for future revisions.

#### `node`

Encapsulate a sequence of ROS entity interfaces.
It is specific to a node as determined by its associated attributes.

Attributes:
- **name**: The base name of the node.

#### `action`

Define the interface for a given action.

Attributes:
- **name**: The name of the action.
- **type**: The type of the action.
Valid values are any ROS action types.
- **server**: Whether or not the node provides an action server for the action.
Valid values are "true" or "false". Defaults to "false".
- **client**: Whether or not the node provides a client for the action.
Valid values are "true" or "false". Defaults to "false".

#### `parameter`

Define the interface for a given parameter.

Attributes:
- **name**: The name of the parameter.
- **type**: The type of the parameter.
Valid values are any ROS parameter types.

#### `service`

Define the interface for a given service.

Attributes:
- **name**: The name of the service.
- **type**: The type of the service.
Valid values are any ROS service types.
- **server**: Whether or not the node provides a server for the service.
Valid values are "true" or "false". Defaults to "false".
- **client**: Whether or not the node provides a client for the service.
Valid values are "true" or "false". Defaults to "false".

#### `topic`

Define the interface for a given topic.

Attributes:
- **name**: The name of the topic.
- **type**: The type of the message.
Valid values are any ROS message types.
- **publisher**: Whether or not the node publishes on the topic.
Valid values are "true" or "false". Defaults to "false".
- **subscription**: Whether or not the node subscribes to the topic.
Valid values are "true" or "false". Defaults to "false".

[dds_security]: https://www.omg.org/spec/DDS-SECURITY/1.1/PDF
[sros2_design]: /articles/ros2_dds_security.html
[launch_ros]: https://github.com/ros2/launch_ros
[xml_wiki]: https://en.wikipedia.org/wiki/xml
[rep149_export]: http://www.ros.org/reps/rep-0149.html#export
