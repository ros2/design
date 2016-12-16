---
layout: default
title: ROS 2 middleware interface
permalink: articles/ros_middleware_interface.html
abstract:
  This article describes the rationale for using an abstract middleware interface between ROS and a specific middleware implementation.
  It will outline the targeted use cases as well as their requirements and constraints.
  Based on that the developed middleware interface is explained.
author: '[Dirk Thomas](https://github.com/dirk-thomas)'
published: true
categories: Middleware
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## The *middleware interface*

### Why does ROS 2 have a *middleware interface*

The ROS client library defines an API which exposes communication concepts like publish / subscribe to users.

In ROS 1 the implementation of these communication concepts was built on custom protocols (e.g., [TCPROS](http://wiki.ros.org/ROS/TCPROS)).

For ROS 2 the decision has been made to build it on top of an existing middleware solution (namely [DDS](http://en.wikipedia.org/wiki/Data_Distribution_Service)).
The major advantage of this approach is that ROS 2 can leverage an existing and well developed implementation of that standard.

ROS could build on top of one specific implementation of DDS.
But there are numerous different implementations available and each has its own pros and cons in terms of supported platforms, programming languages, performance characteristics, memory footprint, dependencies and licensing.

Therefore ROS aims to support multiple DDS implementations despite the fact that each of them differ slightly in their exact API.
In order to abstract from the specifics of these APIs, an abstract interface is being introduced which can be implemented for different DDS implementations.
This *middleware interface* defines the API between the ROS client library and any specific implementation.

Each implementation of the interface will usually be a thin adapter which maps the generic *middleware interface* to the specific API of the middleware implementation.
In the following the common separation of the adapter and the actual middleware implementation will be omitted.

    +-----------------------------------------------+
    |                   user land                   |
    +-----------------------------------------------+
    |              ROS client library               |
    +-----------------------------------------------+
    |             middleware interface              |
    +-----------------------------------------------+
    | DDS adapter 1 | DDS adapter 2 | DDS adapter 3 |
    +---------------+---------------+---------------+
    |    DDS impl 1 |    DDS impl 2 |    DDS impl 3 |
    +---------------+---------------+---------------+

## Why should the *middleware interface* be agnostic to DDS

The ROS client library should not expose any DDS implementation specifics to the user.
This is primarily to hide the intrinsic complexity of the DDS specification and API.

While ROS 2 only aims to support DDS based middleware implementations it can strive to keep the *middleware interface* free of DDS specific concepts to enable implementations of the interface using a different middleware.
It would also be feasible to implement the interface by tying together several unrelated libraries providing the necessary functions of discovery, serialization and publish / subscribe.

    +-----------------------------------+
    |             user land             |   no middleware implementation specific code
    +-----------------------------------+
    |        ROS client library         |   above the interface
    +-----------------------------------+
    |       middleware interface        |   ---
    +-----------------------------------+
    | mw impl 1 | mw impl 2 | mw impl 3 |
    +-----------+-----------+-----------+

## How does the information flow through the *middleware interface*

One goal of the *middleware interface* is to not expose any DDS specific code to the user land code.
Therefore the ROS client library "above" the *middleware interface* needs to only operate on ROS data structures.
ROS 2 will continue to use ROS message files to define the structure of these data objects and derive the data structures for each supported programming language from them.

The middleware implementation "below" the *middleware interface* must convert the ROS data objects provided from the client library into its own custom data format before passing it to the DDS implementation.
In reverse custom data objects coming from the DDS implementation must be converted into ROS data objects before being returned to the ROS client library.

The definition for the middleware specific data types can be derived from the information specified in the ROS message files.
A defined mapping between the primitive data types of ROS message and middleware specific data types ensures that a bidirectional conversion is possible.

    +----------------------+
    |      user land       |   1) create a ROS message
    +----------------------+      v
    |  ROS client library  |   2) publish the ROS message
    +----------------------+      v
    | middleware interface |      v
    +----------------------+      v
    |      mw impl N       |   3) convert the ROS message into a DDS sample and publish the DDS sample
    +----------------------+

<!--- separate code blocks -->

    +----------------------+
    |      user land       |   3) use the ROS message
    +----------------------+      ^
    |  ROS client library  |   2) callback passing a ROS message
    +----------------------+      ^
    | middleware interface |      ^
    +----------------------+      ^
    |      mw impl N       |   1) convert the DDS sample into a ROS message and invoke subscriber callback
    +----------------------+

Depending on the middleware implementation the extra conversion can be avoided by implementing serialization functions directly from ROS messages as well as deserialization functions into ROS messages.

## Considered use cases

The following use cases have been considered when designing the middleware interface:

### Single middleware implementation

ROS applications are not built in a monolithic way but distributed across several packages.
Even with a *middleware interface* in place the decision of which middleware implementation to use will affect significant parts of the code.

For example, a package defining a ROS message will need to provide the mapping to and from the middleware specific data type.
Naively each package defining ROS messages might contain custom (usually generated) code for the specific middleware implementation.

In the context of providing binary packages of ROS (e.g., Debian packages) this implies that a significant part of them (at least all packages containing message definitions) would be specific to the selected middleware implementation.

                    +-----------+
                    | user land |
                    +-----------+
                         |||
          +--------------+|+-----------------+
          |               |                  |
          v               v                  v
    +-----------+   +-----------+   +-----------------+   All three packages
    | msg pkg 1 |   | msg pkg 2 |   | middleware impl |   on this level contain
    +-----------+   +-----------+   +-----------------+   middleware implementation specific code

### Static vs. dynamic message types with DDS

DDS has two different ways to use and interact with messages.

On the one hand the message can be specified in an IDL file from which usually a DDS implementation specific program will generate source code.
The generated code for C++, e.g., contains types specifically generated for the message.

On the other hand the message can be specified programmatically using the DynamicData API of the [XTypes](http://www.omg.org/spec/DDS-XTypes/) specification.
Neither an IDL file nor a code generation step is necessary for this case.

Some custom code must still map the message definition available in the ROS .msg files to invocations of the DynamicData API.
But it is possible to write generic code which performs the task for any ROS .msg specification passed in.

                    +-----------+
                    | user land |
                    +-----------+
                         |||
          +--------------+|+----------------+
          |               |                 |
          v               v                 |
    +-----------+   +-----------+           |            Each message provides its specification
    | msg pkg 1 |   | msg pkg 2 |           |            in a way which allows a generic mapping
    +-----------+   +-----------+           |            to the DynamicData API
          |               |                 |
          +-------+-------+                 |
                  |                         |
                  v                         v
     +-------------------------+   +-----------------+   Only the packages
     | msg spec to DynamicData |   | middleware impl |   on this level contain
     +-------------------------+   +-----------------+   middleware implementation specific code

However the performance using the DynamicData API will likely always be lower compared to the statically generated code.

### Switch between different implementations

When ROS supports different middleware implementations it should be as easy and low effort as possible for users to switch between them.

One obvious way will be for a user to rebuild all ROS packages from source selecting a different middleware implementation.
While the workflow won't be too difficult (probably half a dozen command-line invocations), it still requires quite some build time.

To avoid the overhead of rebuilding everything a different set of binary packages could be provided.
While this would reduce the effort for the user the buildfarm would need to build a completely separate set of binary packages.
The effort to support N packages with M different middleware implementation would require significant resources for maintaining the service as well as the necessary computing power.

#### Reduce the number of middleware implementation specific packages

One way to at least reduce the effort for building for different middleware implementations is to reduce the number of packages depending on the specific middleware implementation.
This can, for example, be achieved using the DynamicData API mentioned before.
Since only a few packages need to be built for each middleware implementation it would be feasible to generate binary packages for them on the buildfarm.

The user could then install (one | a few) binary package(s) for a specific middleware implementation together with its specific implementation of the mapping between the message specification and the DynamicData API.
All other binary packages would be agnostic to the selected middleware implementation.

            +-----------+
            | user land |
            +-----------+
                  |
                  v
            +-----------+           Generic binary packages
            | msg pkgs  |           agnostic to selected
            +-----------+           middleware implementation
                  |
                  ?                 Select middleware implementation
                /   \               by installing (one | a few)
              /       \             binary package(s)
    +-----------+   +-----------+
    | mw impl 1 |   | mw impl 2 |
    +-----------+   +-----------+

#### Generate "fat" binary packages

Another way to enable the user to switch between middleware implementations without the need to use the DynamicData API is to embed the middleware specific generated code for all supported implementations into each binary package.
The specific middleware implementation would then be selected, for example, at link time and only the corresponding generated code of that middleware implementation would be used.

### Using single middleware implementation only

When building ROS with a single middleware implementation the result should follow the design criteria:

> Any features that you do not use you do not pay for.

This implies that there should be no overhead for neither the build time nor the runtime due to the ability to support different middleware implementations.
However the additional abstraction due to the middleware interface is still valid in order to hide implementation details from the user.

## Design of the *middleware interface*

The API is designed as a pure function-based interface in order to be implemented in C.
A pure C interface can be used in ROS Client Libraries for most other languages including Python, Java, and C++ preventing the need to reimplement the core logic.

### Publisher interface

Based on the general structure of ROS nodes, publishers and messages for the case of publishing messages the ROS client library need to invoke three functions on the middleware interface:

- `create_node()`
- `create_publisher()`
- `publish()`

#### Essential signature of `create_node`

Subsequent invocations of `create_publisher` need to refer to the specific node they should be created in.
Therefore the `create_node` function needs to return a *node handle* which can be used to identify the node.

    NodeHandle create_node();

#### Essential signature of `create_publisher`

Besides the *node handle* the `create_publisher` function needs to know the *topic name* as well as the *topic type*.
The type of the *topic type* argument is left unspecified for now.

Subsequent invocations of `publish` need to refer to the specific publisher they should send messages on.
Therefore the `create_publisher` function needs to return a *publisher handle* which can be used to identify the publisher.

    PublisherHandle create_publisher(NodeHandle node_handle, String topic_name, .. topic_type);

The information encapsulated by the *topic type* argument is highly dependent on the middleware implementation.

##### Topic type information for the DynamicData API

In the case of using the DynamicData API in the implementation there is no C / C++ type which could represent the type information.
Instead the *topic type* must contain all information needed to describe the format of the message.
This information includes:

- the name of the package in which the message is defined

- the name of the message

- the list of fields of the message where each includes:

  - the name for the message field
  - the type of the message field (which can be either a built-in type or another message type), optionally the type might be an unbounded, bounded or fixed size array
  - the default value

- the list of constants defined in the message (again consisting of name, type and value)

In the case of using DDS this information enables one to:

- programmatically create a *DDS TypeCode* which represents the message structure
- register the *DDS TypeCode* with the *DDS Participant*
- create a *DDS Publisher*, *DDS Topic* and *DDS DataWriter*
- convert data from a *ROS message* into a *DDS DynamicData* instance
- write the *DDS DynamicData* to the *DDS DataWriter*

##### Topic type information for statically generated code

In the case of using statically generated code derived from an IDL there is are C / C++ types which represent the type information.
The generated code contains functions to:

- create a *DDS TypeCode* which represents the message structure

Since the specific types must be defined at compile time the other functionalities can not be implemented in a generic (not specific to the actual message) way.
Therefore for each message the code to perform the following tasks must be generated separately:

- register the *DDS TypeCode* with the *DDS Participant*
- create a *DDS Publisher*, *DDS Topic* and *DDS DataWriter*
- convert data from a *ROS message* into a *DDS DynamicData* instance
- write the *DDS DynamicData* to the *DDS DataWriter*

The information encapsulated by the *topic type* must include the function pointers to invoke these functions.

#### `get_type_support_handle`

Since the information encapsulated by the *topic type* argument is so fundamentally different for each middleware implementation it is actually retrieved through an additional function of the middleware interface:

    MessageTypeSupportHandle get_type_support_handle();

Currently this function is a template function specialized on the specific ROS message type.
For C compatibility an approach using a macro will be used instead which mangles the type name into the function name.

#### Essential signature of `publish`

Beside the *publisher handle* the `publish` function needs to know the *ROS message* to send.

    publish(PublisherHandle publisher_handle, .. ros_message);

Since ROS messages do not have a common base class the signature of the function can not use a known type for the passed ROS message.
Instead it is passed as a void pointer and the actual implementation reinterprets it according to the previously registered type.

#### Type of the returned handles

The returned handles need to encapsulate arbitrary content for each middleware implementation.
Therefore these handles are just opaque objects from a user point of view.
Only the middleware implementation which created it knows how to interpret it.

In order to ensure that these information are passed back to the same middleware implementation each handle encodes a unique identifier which the middleware implementation can check before interpreting the handles payload.

### Subscriber interface

The details of the interface necessary for the subscriber side are not (yet) described in this document.

## Current implementation

The described concept has been implemented in the following packages:

- the package [rmw](https://github.com/ros2/rmw/tree/master/rmw) defines the middleware interface

  - the functions are declared in [rms/rmw.h](https://github.com/ros2/rmw/blob/master/rmw/include/rmw/rmw.h)
  - the handles are defined in [rmw/types.h](https://github.com/ros2/rmw/blob/master/rmw/include/rmw/types.h)

- the package [rmw_connext_cpp](https://github.com/ros2/rmw_connext/tree/master/rmw_connext_cpp) implements the middleware interface using [RTI Connext DDS](http://www.rti.com/products/dds/index.html) based on statically generated code

  - the package [rosidl_typesupport_connext_cpp](https://github.com/ros2/rmw_connext/tree/master/rosidl_typesupport_connext_cpp) generates

    - the DDS specific code based on IDL files for each message

      - the package [rosidl_generator_dds_idl](https://github.com/ros2/rosidl_dds/tree/master/rosidl_generator_dds_idl) generates DDS IDL files based on ROS msg files

    - additional code to enable invoking the register/create/convert/write functions for each message type

- the package [rmw_connext_dynamic_cpp](https://github.com/ros2/rmw_connext/tree/master/rmw_connext_dynamic_cpp) implements the middleware interface using *RTI Connext DDS* based on the DynamicData API

  - the package [rosidl_typesupport_introspection_cpp](https://github.com/ros2/rosidl_dds/tree/master/rosidl_typesupport_introspection_cpp)

    - generates code which encapsulated the information from each ROS msg file in a way which makes the data structures introspectable

- the package [rmw_opensplice_cpp](https://github.com/ros2/rmw_opensplice/tree/master/rmw_opensplice_cpp) implements the middleware interface using [PrismTech OpenSplice DDS](http://www.prismtech.com/opensplice) based on statically generated code

  - the package [rosidl_typesupport_opensplice_cpp](https://github.com/ros2/rmw_opensplice/tree/master/rosidl_typesupport_opensplice_cpp) generates

    - the DDS specific code based on IDL files for each message
    - additional code to enable invoking the register/create/convert/write functions for each message type

### One or multiple type support generators

The packages contributing to the message generation process of each message are called *type support generators*.
Their package name starts with the prefix `rosidl_typesupport_`.

Each message package will contain the generated code from all type support generators which are available when the package is configured.
This can be only one (when building against a single middleware implementation) or multiple type support generators.

### User land code decides at link time

The packages implementing the middleware interface are called *middleware implementations*.
Their package name starts with the prefix `rmw_` followed by the name of the used middleware (e.g., *connext* or *opensplice*).

A user land executable using ROS nodes, publishers and subscribers must link against one specific middleware implementation library.
The used middleware implementation library will only use the corresponding type support from each message package and ignore any additionally available type supports.
Only middleware implementations can be selected for which the corresponding type support generators have been available when building the message packages.

### Mapping between DDS and ROS concepts

Every ROS node is one DDS participant.
If multiple ROS nodes are being run in a single process they are still mapped to separate DDS participants.
If the containing process exposes its own ROS interface (e.g. to load nodes into the process at runtime) it is acting as a ROS node itself and is therefore also mapped to a separate DDS participant.

The ROS publishers and subscribers are mapped to DDS publishers and subscribers.
The DDS DataReader and DataWriter as well as DDS topics are not exposed through the ROS API.

The ROS API defines queue sizes and a few [Quality of Service parameters](http://design.ros2.org/articles/qos.html) which are being mapped to their DDS equivalent.
The other DDS QoS parameters are not being exposed through the ROS API.
