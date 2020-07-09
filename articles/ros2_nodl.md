---
layout: default
title: ROS 2 Node Definition Language
permalink: articles/ros2_node_definition_language.html
abstract:
  This article specifies the ROS 2 Node Definition Language, a simple and standardized manner to export the complete interface (action/message/parameter/service) of node(s) in a package.
author: >
  [Jérémie Deray](https://github.com/artivis),
  [Kyle Fazzari](https://github.com/kyrofa)
  [Ted Kern](https://github.com/arnatious)
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
The Node Definition Language (NoDL) specified in the next section is meant to be distributed alongside its associated package, be it in the source code or a generated release packaging format (e.g. debian).
Whether the interface is declared or not is up to the package author and should not prevent the correct execution of any system pre-existing the NoDL.
Similarly, the declared interface may be only partial and allow for the full use of pre-existing systems and the use of dependent systems on the parts covered by the partial interface.

## Motivation

While initially being approached from a ROS 2 Security perspective, the abstraction level of the NoDL allows for the developments of other functionalities and tools.

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
Benefiting from the declared interface(s), it would be able to execute many kind of static assertions (i.e. at launch-time, before running anything) upon the whole system to be launched.
Such assertions could include:
- Check for duplicates.
- Check for multiple publishers on a single topic.
- Check for message type mismatch.
- Check for qos mismatch.
- Check for orphan connections (e.g. a listener is connected to a topic with no publisher).
- Determine if remappings are incorrect.
- Determine if parameter names are incorrect.

These assertions results would then be summarized in a logging file for later debugging.

Another example of the usefulness of having a static interface is the ability to create graphical tools for putting a ROS system together.
Yet another example would be an additional feature in `ros2 pkg create` that would allow a developer to hand it a NoDL and have it generate scaffolding for a node with that interface.

These examples are only a subset of use-cases made possible by such an interface.
It's clear that this is useful well beyond security.

## Challenges to overcome

This proposal has a number of potential upsides, but it also has some downsides worthy of discussion.

### This is only really useful if it gains significant adoption in upstream packages

It's true that, if not all of the packages in one's system have adopted this, its gains are incomplete.
However, it's still useful even if only a subset of the packages adopt it (e.g. one's own packages), which means even without significant upstream adoption it will still be useful to individuals or organizations.
Upstream packages that haven't adopted this simply won't benefit from it.
Also, its usefulness hopefully outweighs the work required to implement it upstream, and it's certainly something that can be contributed by community members given that the interface would be reviewed by the experts in the package.

### Declared and actual interface can get out of sync

This is certainly a concern: an out-of-date interface is debatably less useful than having no interface at all.
There are a number of possibilities that will help with this issue.
One possibility is to more tightly couple the declared and actual interface by creating a library that consumes the declared interface and creates the corresponding ROS entities.
Another is the fact that, as soon as the node is running, RCL itself (or another `ros2` command) can verify that the actual interface properly corresponds to the declared interface, and can act appropriately.

## Package interface

How do upstream packages specify their interface requirements?
Through a high-level description of all the actions, parameters, services and topics, provided or required, by each node within the package.

The package interface is defined in a separate [XML][xml_wiki] file with suffix `.nodl.xml`.
This XML file is exported to the [ament index][ament_index], either manually in the case of python projects or with a helper CMake macro.
The interface may cover only a subset of nodes in a package, as long as the nodes that _are_ covered are done so completely.

Here is an example NoDL for a package containing two nodes:

{% include_relative ros2_nodl/interface_declaration.xml %}

Once an NoDL file is written, it is exported from either `CMakeLists.txt` or `setup.py` (more details below).
Note that several NoDL files can be exported, allowing for writing one NoDL file per node if desired.

### Exporting a NoDL to the Ament Index

Per the design philosophy of the [ament index][ament_index], files will be installed in two locations.
In the case of a package named `Foo`, the NoDL file `foo.nodl.xml` should be placed in the package share directory, `share/foo/foo.nodl.xml`.
A corresponding marker file, `share/ament_index/nodl_desc/foo`, should be created.
It is either empty or contains the relative path to the `foo.nodl.xml` file.

#### CMake Macro

For packages using ament_cmake, the package `ament_nodl` provides the macro `nodl_export_node_description_file` which performs the export described in [Exporting a NoDL to the Ament Index](#exporting-a-nodl-to-the-ament-index).

For a package `Foo`, containing `foo.nodl.xml`, the following lines are added to `CMakeLists.txt`:

```cmake
find_package(ament_nodl REQUIRED)
nodl_export_node_description_file(foo.nodl.xml)
```

#### setup.py

In the case of setup.py, placement of the NoDL file and marker in the index must be done manually alongside the placement of the package's marker in the ament index.
One can re-use the same empty marker file placed in the package index.
An example `data_files` argument to `setuptools.setup()` in `setup.py` follows:

```python
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/nodl_desc',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml', package_name + '.nodl.xml']),
    ],
```

### NoDL Schema

An `.xsd` xml schema is provided alongside the NoDL implementation.
This can be used to programmatically validate the NoDL's `.xml` document.
There are some semantics that cannot be expressed in this schema, so the the `.xsd` is not authoritative.
Rather, it is a heuristic, and the [NoDL reference implementation][nodl-reference] can reject a document that does not conform to other requirements.

#### `interface`

Root tag of the NoDL file, it is made up of a collection of node interfaces.
There must be only one tag per NoDL file.

Attributes:
- **version**: version of schema in use, allowing for future revisions.

#### `node`

Encapsulate a sequence of ROS entity interfaces.
It is specific to a node as determined by its associated attributes.

Attributes:
- **name**: The base name of the node.
- **executable**: The name of the generated executable that contains the node.

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
[ament_index]: https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md#integration-with-other-systems
[nodl_reference]: https://github.com/ubuntu-robotics/nodl
