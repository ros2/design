---
layout: default
title: ROS 2 Interface Description Language
permalink: articles/ros2_node_interface_declarations.html
abstract:
  This article specifies the ROS 2 Interface Description Language, a simple and standardized manner to export the complete interface (action/message/parameter/service) of node(s) in a package.
author: >
  [Kyle Fazzari](https://github.com/kyrofa),
  [Jérémie Deray](https://github.com/artivis)

published: true
categories: Security
---

{:toc}


# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}


## Background

Every ROS 2 node has an associated interface that describes how it communicates with other nodes, as well as how it is to be configured. This interface is defined in code, and consists of:

-  Actions
-  Parameters
-  Services
-  Topics

That interface is obviously very valuable on different levels and from different perspectives. While it is usually readily available to a developer looking at the code, it cannot be automatically extracted. The main barrier to this is the lack of introspection in C++. It therefore calls for the creation of a standardized way to explicitly define and export this information.

This article defines a high-level abstraction allowing upstream packages to specify the communication requirements of the nodes in the package, such that final user, be it a developer or a static analysis tool, can benefit from it. The Interface Description Language specified in the next section is meant to be distributed alongside its associated package. This is true for the package source and also for the generated release.

### Initial motivation

Thanks to [DDS-Security][dds_security] and [SROS2][sros2_design], security is at the heart of ROS 2. DDS enforces access control using a signed permissions document for each domain participant (see the [DDS-Security spec][dds_security], section 9.4). In SROS 2, that permissions document is generated from a ROS-specific [XML][xml_wiki] policy file that may include the permissions for one or many nodes.

Currently policy files can be created in one of two ways:
-  Written by hand.
-  A snapshot of the live ROS 2 graph can be taken and written into a policy that covers its current state via `ros2 security generate_policy`.

While the first option is obviously very tedious and error-prone, the second only partially alleviates the burden due to the fact that it cannot fully cover the dynamic nature of a ROS 2 graph and all of its interactions. More problematic than these issues, though, is that both options put the onus of security squarely on the shoulders of end users.

This introduces two problems:

-  While developers will be able to define the set of rules securing their own ROS 2 nodes, the nodes developed in-house are often outnumbered by upstream components when it comes to the entire node graph, and the developers are typically not experts in every component being used. Without that expertise, the entire node graph cannot be properly locked down.
-  Consider a complex and popular upstream component, perhaps parts of the navigation stack. **Every end user** of this component must duplicate the effort of attempting to properly lock it down.

While initially being approached from a ROS 2 Security perspective, the abstraction level of the Interface Description Language allows for the developments of other functionalities and tools. Such developments are briefly speculated in the last section of this article.

## Package interface declaration

How do upstream packages specify their interface requirements? Through a high-level description of all the actions, parameters, services and topics, provided or required, by each node within the package.

On the model of the [`pluginlib` export mechanism][rep149_export], the package interface is defined in a separate [XML][xml_wiki] file and exported from the package manifest `package.xml`. This scheme avoids polluting the package manifest and allows for dispatching the interface definition of several nodes to several files.

This article standardizes the Interface Description Language XML format hereafter. The following example illustrates it,

``` xml
{% include_relative ros2_node_idl/interface_declaration.xml %}
```

Once an interface description file being defined, it is exported from the package manifest as shown in the following example,

``` xml
{% include_relative ros2_node_idl/package.xml %}
```

### Schema

This section is a description of a ROS 2 Interface Description Language schema. Relying on XML syntax, it is expressed in terms of XML tags.

#### `interface`

The introduction of the `interface` tag within the `export` tag of the package manifest raises a small difficulty with regards to the [REP-149][rep149_export].
The REP specifies,

“To avoid potential collisions, an export tag should have the same name as the package which is meant to process it. The content of that tag is up to the package to define and use.”

Considering the high level abstraction of the Interface Description Language, the `interface` tag is not meant for a specific package but rather declares intrinsic properties for anyone to process it. However, the keyword is likely solidly descriptive enough to either be accepted as falling under [REP-149][rep149_export] or else to motivate an amendment of the REP.

#### `node`

Encapsulate a sequence of interfaces. It is specific to a unique node instance as determined by its associated attributes.

Attributes:
- **name** The name of the node
- **ns** The namespace of the node

#### `action`

Attributes:
- **name** The name of the action
- **type** The type of the action
  - Valid values are any ROS action types as defined in <link>
- **server** Whether or not the node provides an action server for the action
  - Valid values are “true” or “false”
- **client** Whether or not the node provides a client for the action
  - Valid values are “true” or “false”

#### `message`

Attributes:
- **name** The name of the topic
- **type** The type of the message
  - Valid values are any ROS message types as defined in <link>
- **publisher** Whether or not the node publishes on the topic
  - Valid values are “true” or “false”
- **subscription** Whether or not the node subscribes to the topic
  - Valid values are “true” or “false”

#### `parameter`

Attributes:
- **name** The name of the parameter
- **type** The type of the parameter
  - Valid values are any ROS parameter types as defined in <link>

#### `service`

Attributes:
- **name** The name of the service
- **type** The type of the service
  - Valid values are any ROS service types as defined in <link>
- **server** Whether or not the node provides a server for the service
  - Valid values are “true” or “false”
- **client** Whether or not the node provides a client for the service
  - Valid values are “true” or “false”

## Fostering new use cases

While being initiated from a ROS 2 Security stance, the Interface Description Language fosters other use cases and tools briefly speculated here,

-  Interface static analysis (launch file validation, remapping assertion etc)
  -  e.g. "You're remapping from a topic that isn't being published!"
  -  e.g. Facilitate the understanding of system-wide access control
-  Auto generation of a node skeleton code
  -  e.g. `$ ros2 pkg create --from-interface interface_declaration.xml` generate a fully functional node given an interface
-  Interface documentation auto generation
  -  e.g. To be published in ROS 2 wiki, maintaining it up-to-date and guaranteeing the consistency between the actual interface and its documentation
-  System-wide interface-based design using graphical tools
-  End-user permissions validation
  -  e.g. Think smartphone apps permissions upon installation

[dds_security]: https://www.omg.org/spec/DDS-SECURITY/1.1/PDF
[sros2_design]: /articles/ros2_dds_security.html
[xml_wiki]: https://en.wikipedia.org/wiki/xml
[rep149_export]: http://www.ros.org/reps/rep-0149.html#export
