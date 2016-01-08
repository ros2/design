---
layout: default
title: Topological Discovery and Communication Negotiation
abstract:
  This article lays out the logical components and possibilities within a discovery and transport negotiation system.
  This article was written to try and understand the different possibilities for how the middleware could be implemented.
published: true
author: '[William Woodall](https://github.com/wjwwood)'
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

> For context, this article explores the ideal and theoretical aspects of discovery and negotiation.
> It does not aim to answer all implementation questions or suggest implementation strategies.
> It simply tries to capture the concepts in the design space and identify trade-offs and relationships between design elements.

Original Author: {{ page.author }}

## Problem Space

ROS systems tend to be implemented as a **computational graph**, where there are graph **node**'s connected by **topic**'s and **service**'s.
The graph which models the computational nodes with their topics and services can be different from the graph which represents the physical groupings and connections which implement the behavior modeled in the topics and services graph.

For the purposes of this document, the graph which defines the computational nodes and how they are connected using topics and services will be referred to as the **computational graph**.
The graph which models processes around nodes and physical layers between nodes will be referred to as the **data layer graph**.
A **node** is any addressable participant in the **computational graph**, it does not imply how nodes are organized into system processes, i.e. a node is neither necessarily a single process, nor does it necessarily share a process with other nodes.

To demonstrate the difference between the **computational graph** and the **data layer graph**, consider the following:

- A computational graph has nodes N1, N2, N3, and N4.
- N1 is on machine M1 and in process P1.
- N2 is on machine M1 and in process P1.
- N3 is on machine M1 and in process P2.
- N4 is on machine M2 and in process P3.
- N1, N2, N3, and N4 are all publishing and subscribing to topic T1.
- N1 is publishing to topic T2, while N2, N3, and N4 are subscribing to topic T2.

For the purposes of this example, assume these computational nodes can optionally share processes and be on different machines.
How this is accomplished is not relevant to this document, but might be covered in another document.

The above describes how the **computational graph** is organized from a conceptual point of view.
The existence of a node, its host machine and host process are products of the graph's execution, which would be done by the user executing each in turn, or by a process management system.
The topic publications and subscriptions are defined at runtime by the user code in each of the nodes, though for some use cases the topics might also have to be captured externally and statically.

The above constraints do not at all describe the method by which messages over topics are delivered to various nodes of the graph, this is known as the **data layer graph**.
It is the responsibility of some entity, or entities, to use this **computational graph** (and optionally the information about process and machine layout) to decide how the **data layer graph** should be implemented and then execute that implementation.

In the current ROS system, each node reports its address and configuration to a master process which coordinates the graph.
The master is responsible for maintaining the graph state and notifying nodes of relevant changes to the graph, e.g. a new publisher of a topic which this node subscribes to was created.
The nodes will then initiate connections to other nodes where appropriate, so in this sense the nodes are somewhat autonomous.
One could argue that ROS should have multiple masters rather than sharing one master amongst multiple machines, or that there should be no master and all of the nodes should be completely autonomous.The point here is that there is no one-size fits all solution, therefore this paper will try to identify ways that these discovery and negotiation steps can be abstracted such that many different implementations may be supported.

## Use Cases

For the proposed solution to this design space, this paper will build up the interfaces in layers, such that for different use cases the process can be gracefully degraded.
Therefore, this paper will list the use cases in order of complexity.

### Statically Configured Nodes

The most basic use case is where each node is given a list of declarative instructions on how to connect to the data layer.
These instructions would likely come in the form of url's, but would be of the notion "Connect to Topic `<topic>` of type `<msg_type>` via `<protocol>://<address>:<port>/` using `<transport>` and `<serialization>`".
A more concrete example might be "Connect to Topic `/foo` of type `pkg/Foo` via `udpm://225.82.79.83:11311/` using `<zmq_pgm>` and `<protobuf>`".

The basic building block that this requires is that nodes provide an API which allows something to instruct the node to establish some connection ("connect_to") and to map publishers and subscribers to a given connection ("map_to").
The program parsing the declarative instructions would just iterate over the instructions, calling this "connect_to" function for each url and then the "map_to" function on each publisher and subscriber.

It should be noted that at this point, this "map_to" call should fail if the node is asked to do something which it has not previously set itself up to do, e.g. a node is asked to map a topic and subscriber to a connection for which it has not created a Subscriber instance.
This constraint implies the need for a life cycle where any publishers and subscribers are instantiated by the node in one step and then connections are made in another step.

Because the decisions about how to connect to the data layer are static, it also removes the possibility for dynamically creating publishers and subscribers on the fly because no entity will be watching for new publishers/subscribers and dynamically determining and executing data layer connections.

Another point is that when nodes loose connection with each other on the data layer (temporary loss of network), the node implementation which calls "connect_to" would be responsible for issuing a new "connect_to" after the connection dies.
This implies there should be away to introspect the node by polling it or by getting notifications about the state of the underlying connections which were created.

### Statically Configured Graph

The next more complicated system is one where each node starts and waits for an outside process to tell it how to make its connections to the data layer.
In this scenario a central authority has the static configuration of all node addresses and knows how they should be connected to each other in the data layer.
In order to execute this, the central authority needs to be able to call the previously described "connect_to" and "map_to" functions remotely.

This adds the necessity for a node to provide an RPC interface for the "connect_to" and "map_to" functions.

Like the previous use case, ideally someone should be monitoring the state of the connections, reactivating them if necessary.
If the central authority has this responsibility then it should have some way of introspecting the state of these connections.
This leads to a need for introspection of graph participants be available externally as well.

The main evolution of required functionality for this system over the previous one is that graph participant's interfaces must be remotely accessible.
The ability to remotely access the "connect_to" and "map_to" functions is a requirement, where as the ability to access the node's state remotely is only required when an external process is monitoring the connections that were established.

### A Fork in the Features

This is were the system features matrix forks.
There are two glaring limitations of the previous use cases:

- Nodes have no general way to notify the rest of the graph about events, events like:

  - node life cycle state changed
  - heartbeat
  - connection_established/connection_lost
  - mapping_established/mapping_lost
  - etc...

- The list of nodes and their addresses, publishers, subscribers, etc. are statically maintained, which prevents:

  - dynamically computing the data layer connections
  - dynamically adding nodes
  - dynamically adding publishers and subscribers

The first limitation is about having the ability to introspect the changes in the **data layer graph** so that the system can react dynamically to things like dropped connections.
The second limitation is about having the ability to dynamically discover and manipulate the layout of the **computational graph** which might in turn change the **data layer graph**.

First this paper will look at how the system can be dynamically configured.

### Dynamically Configured Graph with Static Discovery

One of the issues with the "Statically Configured Graph" system described above is that the topics and/or services each nodes provides or uses must be statically defined, either as part of the configuration for each node, or as part of the centralized authority's configuration.
This does not allow for dynamically defined topic subscriptions and publications nor dynamically provided services.

In order to enable these type of flexible or dynamic node configurations, each node must provided an externally accessible function for getting the configuration of itself.
This would allow a central authority to periodically check each node for new topic subscriptions or publications and/or new service providers and dynamically change the **data layer graph** layout to reflect these changes.

This also allows for a system where the list of graph participants and their location is known, but the data layer graph is unknown and can be determined at runtime.
This would allow a small additional flexibility over a completely statically configured system.

### Statically Configured Graph with Data Layer Events

The other issue with the "Statically Configured Graph" system above is that it cannot easily monitor the state of each of the graph participants and their connections because it would require some form of polling or point to point event systems.
In order to better facilitate use cases where graph state would be maintained in a decentralized manner, point to point events should be avoided (at least conceptually), and instead participants in the graph should be able to send messages (events) to the "graph" notifying the rest of the graph participants of changes to their own state, and anything which wants to monitor the state of the graph should be able to maintain a consistent state of the graph by listening to messages sent to the graph by its participants.
This assumes that nodes are the authority of their state in the graph.

This begins to outline the need for a graph interface, which allows a user to maintain the state of the graph, get asynchronous notifications of graph changes, and send events to the graph.
What the graph interface looks like is discussed in a later section.

With a graph interface anyone, either a graph participant or an observer, can monitor the state of the graph and potentially react to changes in the graph.
This allows for scenarios like a long running central authority which can setup the graph initially, and restore any connections when necessary at runtime.

Along with data layer events, comes the notion of liveliness.
When a connection is terminated for some reason an event should be sent to the graph, but often the reason for a disconnect will be that one end of the connection has dropped off the graph unexpectedly and therefore an event is unlikely to reach the graph.
For this reason, it makes sense to include liveliness into the system when data layer events are added.
Liveliness is not required, but could be added to any system which has a notion of the graph interface and is able to send and receive messages to the graph, these messages would be some form of heartbeat.

### Dynamically Configured Graph with Data Layer Events and Static Discovery

This system simply adds the data layer events (connection established/lost, heartbeat, etc...) on top of the "Dynamically Configured Graph with Static Discovery".
This system would be able to take a static set of nodes, with addresses, and dynamically detect their configuration, determine an appropriate data layer graph, and execute it.
It would also be able to adapt to a change in the configuration of the node and adapt to data layer events, like temporarily lost connections, or connection state introspection.

### Dynamically Configured Graph with Dynamic Discovery

The obvious next step is a system where the participants of the graph and their locations (addresses) are discovered at runtime.
This information along with the ability for nodes to dynamically configure their topics and services, allows for a lot of the tools which exist currently in ROS.
Being able to add a node to the graph in an ad-hoc manner and then dynamically create publishers and subscribers, allows for dynamic introspection of the graph as well as dynamic development of the graph.

Implementation of this system requires the notion of the graph interface, so that on node creation and termination, the node can send messages to the graph, notifying the rest of the graph of their participation in the graph or their leaving of the graph.

This system does not have the data layer events described in previous systems, though it is likely that once a system is capable of dynamic discovery and dynamic configuration, then the data layer events will likely also be present.

### Dynamically Configured Graph with Data Layer Events and Dynamic Discovery

This is the most fully featured system covered in this paper, as it combines dynamic configuration of nodes (topics and services), dynamic discovery of nodes, and data layer events.

This system is capable of supporting dynamic insertion and removal of nodes in the graph.
Each of those nodes can dynamically change their configurations at will.
One or more entities can monitor the state of the nodes and their **computational graph** layout, determine part of or a whole **data layer graph** layout, and execute the **data layer graph** layout.
Further more these entities can get event driven notifications of changes to the nodes in the graph, changes to their computational graph connections amongst each other, or changes to their data layer connections.

All of these capabilities together allows for complex systems which are capable of dynamic behavior.

### Summary of Use Cases

Below is a table summarizing the above mentioned use cases and what interfaces/features each of them need to be implemented.
All of the systems in the table below require the basic local node API with the "connect_to" and "map_to" functions as well as basic connection introspection.

<div class="table" markdown="1">

| System Name                                                    | Remote Node API | Node Configuration API | Data Layer Events | Dynamic Discovery | Requires Graph API |
| ---                                                            | ---             | ---                    | ---               | ---               | ---                |
| Statically Configured Nodes                                    | .               | .                      | .                 | .                 | .                  |
| Statically Configured Graph                                    | &#x2713;        | .                      | .                 | .                 | .                  |
| Dynamically Configured Graph with Static Discovery             | &#x2713;        | &#x2713;               | .                 | .                 | .                  |
| Statically Configured Graph with Events                        | &#x2713;        | .                      | &#x2713;          | .                 | &#x2713;           |
| Dynamically Configured Graph with Events and Static Discovery  | &#x2713;        | &#x2713;               | &#x2713;          | .                 | &#x2713;           |
| Dynamically Configured Graph with Dynamic Discovery            | &#x2713;        | &#x2713;               | .                 | &#x2713;          | &#x2713;           |
| Dynamically Configured Graph with Events and Dynamic Discovery | &#x2713;        | &#x2713;               | &#x2713;          | &#x2713;          | &#x2713;           |

</div>

## Node Life Cycles

Something that wasn't covered in the previous section of use cases was verifiability of the system.
There are different levels of the system that may need to be verified, depending on the use case.
In the most flexible and dynamic system, one could imagine that the user develops the system using all of the freedom which the dynamic system gives him/her at development time, but also wanting to ensure that his/her system is up and connected the way he expects.
In a simpler system, like the "Statically Configured Graph" system, one could imagine that a centralized authority would want all nodes up and connected before trying to use the system.
In both of these cases, the system needs each participant in the graph to report something about their state.

One option is to provide a Life Cycle to nodes in the graph.
Each node would have a set series of states in which it could be, something like starting, establishing configuration, establishing connections, running, error, and stopping.
The exact states are not as important as the ability for the system to introspect the life cycle state of all of the nodes.

Combining the life cycle information of the nodes with the state of graph could allow for various levels of system verifiability.
It would allow something to ask the question, "Is the system up and running?".
Currently in ROS, a user would run a launch file and hope that all of their nodes start without crashing and then make the correct connections, and it is difficult to verify if the thing the user designed in the launch file is what the user got at launch time.

Life cycle information could be presented to the user as an API, which the user should call at various points in the code to signify transitioning to different states.
If users are fine with design their code in a structured component like construct, the system could provide life cycle reporting automatically.
Life cycle reporting should not be required by default, otherwise it might discourage, or make it more difficult, to debug and experiment using a scripting language.

Life cycle state information could be provided by nodes in any above described use case except the "Statically Configured Nodes" system.
For systems with a graph interface, the life cycle state changes can be sent as a graph message, otherwise a simple remote procedure call interface could be provided.

## Proposed Interfaces

In this section of the paper, some more details about interfaces which were eluded to in the use cases above are provided.

Note that these interfaces are not necessarily what the end user would use, but rather they are the required interfaces which would be used to implement the desired fully featured system which the end user would use.
The added benefit for clearly defining these interfaces is so that systems can be gracefully degraded or adapted for users who have different needs.

### Node Interface

The most basic interface used above is the Node API.
It was briefly described as having "connect_to" and "map_to" functions along with basic connection introspection.

The "connect_to" function is necessary in order to execute the data layer connections.
This function might cause the node to connect to a remote TCP/IP server, setup a local TCP/IP server, join a UDP Multicast Group, set a shared memory block, or something else.

In order to better support non point-to-point communication transport types like UDP Multicast, the system needs to be able to differentiate between transport and topic, which is why the "map_to" function is described as a separate function.
The topic information could be included in the "connect_to" call, but that would make some use cases more difficult.
For example, if a system wanted to send multiple types of topics over a single connection, possibly a single TCP or UDP connection, then it would not really make sense to call the "connect_to" function for a topic whose connection is already established.
Instead the proposed set of functions would prescribe that the "connect_to" function would be called once, returning a handle to the connection, and that handle would be reused in multiple calls to "map_to".

In addition to the "connect_to" and "map_to" functions, the basic node interface should provide introspection of the created connections.
This allows the entity which created the connections to keep tabs on the state of those connections.
This could be implemented using a handle for each connection, returned by "connect_to", allowing the user to poll the state of the connection, but an asynchronous interface could also be implemented, allowing for local events.

This API should always be provided locally, but in most systems described above it should also be exposed over RPC to allow for remote control and management of the node.

### Node Configuration Interface

The node configuration interface allows a user of the interface to get the configuration of the node.
This paper will not try to exhaustively list the contents of the node configuration, but it should at least contain some way to uniquely identify the node, probably a tuple of machine id, process id, and node id.
The configuration should probably also include publications, subscriptions, and services consumed or provided by the node.
In order for a negotiation algorithm to come up with "connect_to" and "map_to" instructions for each node, the configuration for each node will probably need to contain the supported communication paradigms, serialization wire formats, and transports.
The negotiator can then know what connections need to be made and what connections are possible, and then it can use some algorithm to decide how to wire nodes together.

The format for the configurations should probably be some extensible format rather than a statically defined data structure.
Though this might not scale well to small computers or embedded devices.
Obvious candidates would be something like JSON or XML, though a non-nestable solution like INI file might be sufficient for the use case while simultaneously being much easier to parse on embedded computers.

### Graph Interface

Several of the above systems described the need for a graph interface which would allow the graph participants to send and receive messages to the graph.
This graph transport abstraction is the most basic interface required for the graph.
On top of these send and receive graph message functions other functionality can be built.
This design allows the graph interface to be layered.

The first layer of the graph interface is the send and receive layer, which basically allows users to pass messages to and receive messages from the graph in an abstract sense.
The method by which these messages are delivered is not something that the users of the interface should be concerned with.
Systems can provide implementations of the graph transport to match their needs.
This layer of the interface enables nodes to dynamically join the graph, dynamically provide configuration data, send data layer events, send heartbeat messages, send life cycle state changes, and potentially provide other information.
In the case of a broker system each message a node sends to the graph will go directly to the master and the master will redistribute the messages to the appropriate nodes as it sees fit.
In the case of a completely distributed, master-less system each message sent by a node could be broadcast to every other node and each node could filter the incoming messages as it sees fit.

The nodes which will be coordinating with each other will have to agree on a graph transport implementation a priori.
Because there is no opportunity to negotiate the graph transport implementation, there must exist a simple "lingua franca" and the system wide graph transport implementation serves as that unifying language.
It would still be possible to write programs which could serve to transparently bridge networks which used different graph transports.

On top of the first layer an asynchronous graph event system could be created.
Users of this API would instantiate a system which monitors the graph using the receive message function described above and would notify the user when events which the user is interested in occurs.
For instance, the user could ask the system to notify them anytime a new publisher for the topic 'foo' is created.
This could be used by the implementation of a negotiation system, master or master-less, or it could be used by a diagnostics system or even user land code to build more robust systems.

With the graph event interface another interface which maintains the state of the graph and allows for queries on the graph can be built.
A user would use this interface by instantiating an object which would maintain the graph state by monitoring the graph using the graph event system and the node configuration interface.
This interface might be used by a master system or by a visualization tool.
A master system would use this interface in the master process to maintain the graph state, so that questions like "what nodes are publishing to topic 'foo'" can be answered efficiently.
A master-less system would have each node in the graph use this interface to maintain a graph state so that all nodes have a notion of the graph topology and could make distributed decisions about how to connect to their peers.
A graph visualization tool would also likely use this interface in order to visualize the current state of the graph.

## Communication Negotiation

The previous paragraphs have not discussed the negotiation of the communications at all.
It has been described how nodes may provide their configurations dynamically, or the configurations might be captured statically and it has been described how nodes can be instructed to establish connections on the data layer either internally or externally, but not much has been said about determining the appropriate **data layer graph** layout based on the node's configurations and the machine/network topology.
It is left to the implementor of the the negotiation system to use the configurations of the nodes and potentially other information to design and execute a **data layer graph**.
This method of negotiation implies that when determining the **data layer graph**'s layout, all of the required information can be retrieved from the node, i.e. a node should be able to answer "what transports do you support?" through the node configuration API.

What the above set of use cases does do is try to ensure that most conceivable negotiation systems could be implemented on top of these interfaces.
To illustrate, this paper will describe some theoretical systems.

### Client-Server Master System with Point to Point TCP Data Only

This system is very similar to the existing ROS system in that each node on startup contacts a centralized master.
The node reports its existence to the master and notifies the master any time a new publisher or subscriber is created in the node.
The master notifies nodes when a publisher exists for their subscribers and the nodes initiate a TCP connection to the publishing node.
There is no point at which some more sophisticated **data layer graph** layout is chosen.

In this system the graph transport implementation is a connection to the master for each node.
The state of the graph is maintained in the master process only, and each node calls its own interface in order to execute the data layer connections between nodes.
All events and configurations for each node are sent to the master and relayed to the correct nodes by the master.

### Distributed System with Intelligent Multicast

This theoretical system has each node maintain the graph's state and make connections to different UDP multicast groups based on the topics on which it is publishing or subscribing.
But the TTL for the multicast is set to 0, so multicast datagrams do not leave the local machine.
Each node, however, is intelligent enough to know that it needs to start a TCP server when a node on another machine is subscribing to a topic on which it is publishing.
And the node on the other machine is intelligent enough to open a TCP connection to the publishing node rather than joining the multicast group for that topic on its local machine.

In this system the graph transport system might be a udp multicast group, with a higher TTL level or which bridges machines with the help of udp bridges.
The state of the graph is maintained in each node, and each node uses the configuration checksum in the heartbeat message from other nodes to know when it needs to get updated configurations from a node using its node configuration interface.
The nodes each have negotiation logic which allows them to make cooperative networking decisions and use their own node interface to initiate connections and topic-connection mappings.

## Open Questions

There exist still some open questions surrounding Discovery and Negotiation.

### User Hints to Data Layer Implementation

One use case not addressed above is how to allow user code to hint or constrain the generation of the **data layer graph** layout.
Ideally, a user could indicate that data on a certain topic is or is not suitable for unreliable transportation, but as it stands there is no direct way for a node to effect change on the data layer.

One option might be to allow certain hints or constraints to be added to the node configuration, which the negotiation system could use to make more ideal decisions when generating the **data layer graph** layout.
However, this implies that the **data layer graph** is determined at runtime and not static.

Another option is not allow constraints at all, because there will always be the scenario where an end-users wants to reuse a node in a manner for which it was not originally designed and constraints which are not overridable would make the node less reusable.

### Graph Communication

What RPC system should nodes use to expose their API's?

What system should be used for serialization or graph messages?

What transport should be used for graph messages?

One option is just pick one of the available systems that are used by the data layer.
Another option is pick a simple set which must always be available.

### Topic Tools

How could runtime topic remapping and/or aliasing work?

Currently ROS can remap any "ROS Name" at runtime, would it be possible to do this during runtime as well?

One option might be to allow some level of setting of configurations through the Node Configuration API described above.
