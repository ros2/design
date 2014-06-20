---
layout: default
title: ROS on ZeroMQ and Friends
html_title: ZeroMQ and Friends
permalink: articles/ros_with_zeromq.html
abstract:
  This article makes the case for using ZeroMQ and other libraries to implement a new, modern middleware for ROS. This article also covers the results of the ZeroMQ based prototype made by OSRF.
published: true
author: '[William Woodall](https://github.com/wjwwood)'
---

* This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

> This article could use additional details, feel free to propose changes.

Original Author: {{ page.author }}

While this article covers proposals and related experiments for building a new middleware specifically around ZeroMQ, it also generally captures the idea of building a new middleware out of a few component libraries.
This strategy of composing existing libraries into a middleware is in contrast to wrapping up an existing end-to-end middleware which provides most if not all of the middleware needs for ROS out of the box.

## Building a Prototype Middleware from Scratch

In order to meet the needs of ROS, a middleware needs to provide a few key services.
First it needs to provide a way for parts of the system to discover each other and make connections dynamically at run time.
Then the middleware needs to provide one or more transport paradigms for moving information between parts of the system, and for ROS specifically the publish-subscribe pattern is required at a minimum.
Finally, the middleware should provide a means of defining messages and then preparing them for transport, i.e. serialization.
Since ROS 1.x was design there have been several new libraries in these component fields to gain popularity.

### Discovery

For discovery the first solution that was investigated was [Zeroconf](http://en.wikipedia.org/wiki/Zero_configuration_networking) with Avahi/Bonjour.
Some simple experiments were conducted which used [pybonjour](https://code.google.com/p/pybonjour/) to try out using the zeroconf system for discovery.
The core technology here is `mDNSresponder`, which is provided by Apple as free software, and is used by both bonjour (OS X and Windows) and avahi (Linux, specifically avahi-compat).

These zeroconf implementations, however, proved to not be so reliable with respect to keeping a consistent graph between machines.
Adding and removing more than about twenty items at a time from subprocesses typically resulted in inconsistent state on at least one of the computers on the network.
Particularly bad was the experiment of removing items from zeroconf, where several "nodes" registered on machine A and then after a few seconds shutdown cleanly.
Then observed behavior on remote machines B and C is that the zeroconf browser would show all "nodes" as registered, but after being shutdown only some would be removed from the list, resulting in "zombie nodes".
Worse still is that the list of "zombie nodes" were different on B and C.
This problem was only observed between machines using avahi as a compatibility layer, which lead into a closer look into avahi and its viability as a core dependency.
This closer look at avahi raised some concerns about the quality of the implementation with respect to the [Multicast DNS](http://en.wikipedia.org/wiki/Multicast_DNS) and [DNS Service Discovery](http://en.wikipedia.org/wiki/Zero_configuration_networking#Service_discovery) technology.

Further more DNS-SD seems to prefer the trade-off of light networking load for eventual consistency.
This works reasonably well for something like service name look up, but it did not work well for quickly and reliably discovering the proto-ROS graph in the experiments.
This lead to the development of a custom discovery system which is implemented in a few languages as part of the prototype here:

[https://bitbucket.org/osrf/disc_zmq/src](https://bitbucket.org/osrf/disc_zmq/src)

The custom discovery system used multicast UDP packets to post notifications like "Node started", "Advertise a publisher", and "Advertise a subscription", along with any meta data required to act, like for publishers, an address to connect to using ZeroMQ.
The details of this simple discovery system can be found at the above URL.

This system, though simple, was quite effective and was sufficient to prove that implementing such a custom discovery system, even in multiple languages is a tractable problem.

### Data Transport

For transporting bytes between processes, a popular library is [ZeroMQ](http://zeromq.org/), but there are also libraries like [nanomsg](http://nanomsg.org/) and [RabbitMQ](http://www.rabbitmq.com/).
In all of those cases the goal of the library is to allow you to establish connections, explicitly, to other participants and then send strings or bytes according to some communication pattern.
ZeroMQ is an LGPL licensed library which has recently become very popular, is written in C++ with a C API, and has bindings to many languages.
nanomsg is a new MIT licensed library which was created by the original author of ZeroMQ, is written in C with a C API, but is far less mature than ZeroMQ.
RabbitMQ is a brokered system, which is different from ZeroMQ and nanomsg, so it meets some of the discovery needs as well as the transport needs for ROS.
RabbitMQ is licensed under the Mozilla Public License.
All of these libraries could probably be used to replace the ROSTCP transport, but for the purposes of this article we will use ZeroMQ.

In this prototype:

[https://bitbucket.org/osrf/disc_zmq/src](https://bitbucket.org/osrf/disc_zmq/src)

ZeroMQ was used as the transport, which conveniently has bindings in C, C++, and Python.
After making discoveries using the above described simple discovery system, connections were made using ZeroMQ's `ZMQ_PUB` and `ZMQ_SUB` socket's.
This worked quite well, allowing for communication between process in an efficient and simple way.
However, in order to get more advanced features, like for instance latching, ZeroMQ takes the convention approach, meaning that it must be implemented by users with a well known pattern.
This is a good approach which keeps ZeroMQ lean and simple, but does mean more code which must be implemented and maintained for the prototype.

Additionally, ZeroMQ, in particular, relies on reliable transports like TCP or [PGM (Pragmatic General Multicast)](http://en.wikipedia.org/wiki/Pragmatic_General_Multicast), so using it in soft real-time scenarios is sort of a non-starter.

### Message Serialization

In ROS 1.x, messages are defined in `.msg` files and code, which can instantiate and then later serialize the data in that message, is generated at build time for each of the supported languages.
Since ROS was created, several popular libraries which take care of this responsibility have come about.
Google's [Protocol Buffers (Protobuf)](https://code.google.com/p/protobuf/), [MessagePack](http://msgpack.org/), [BSON](http://bsonspec.org/), and [Cap'n Proto](http://kentonv.github.io/capnproto/) are all examples of serialization libraries which have come to popularity since ROS was originally written.
An entire article could be devoted to the pros and cons of different message definition formats, serialization libraries, and their wire formats, but for the purposes of this prototype we worked with either plain strings or protobuf.

## Conclusions

After implementing the custom middleware prototype, some points worth noting were made.
First, there isn't any existing discovery systems which address the needs of the middleware which are not attached to other middlewares.
Implementing a custom discovery system is a possible but time consuming.

Second, there is a good deal of software that needs to exist in order to integrate discovery with transport and serialization.
For example, the way in which connections are established, whether using point to point or multicast is a piece of code which lives between the transport and discovery systems.
Another example is the efficient intra-process communications, ZeroMQ provides an INPROC socket, but the interface to that socket is bytes, so you cannot use that without serialization without constructing a system where you pass around pointers through INPROC rather than serialized data.
At the point where you are passing around points rather than serialized data you have to start to duplicate behavior between the intraprocess and interprocess communications which are abstracted at the ROS API level.
One more piece of software which is needed is the type safety system which works between the transport and the messages serialization system.
Needless to say, even with these component libraries solving a lot of the problems with implementing a middleware like ROS's, there still exists quite a few glue pieces which are need to finish the implementation.

Even though it would be a lot of work to implement a middleware using component libraries like ZeroMQ and Protobuf, the result would likely be a finely tuned and well understood piece of software.
This path would most likely give the most control over the middleware to the ROS community.

In exchange for the creative control over the middleware, comes the responsibility to document its behavior and design to the point that it can be verified and reproduced.
This is a non-trivial task which ROS 1.x did not do very well because it had a relatively good pair of reference implementations.
Many users which wish to put ROS into mission critical situations and into commercial products have lamented that ROS lacks this sort of governing design document which allows them to certify and audit the system.
It would be of paramount importance that this new middleware be well defined, which is not a trivial task and almost certainly rivals the engineering cost of the initial implementation.
