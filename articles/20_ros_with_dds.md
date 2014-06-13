---
layout: default
title: ROS on DDS
permalink: articles/ros_on_dds/
abstract:
  This article makes the case for using DDS as the middleware for ROS, outlining the pros and cons of this approach, as well as considering the impact to the user experience and code API that using DDS would have. The results of the ros_dds prototype are also summarized and used in the exploration of the issue.
author: William Woodall
---

* This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

In this article, the case will be made for using vendor implementation of the [Data Distribution Service (DDS)](http://en.wikipedia.org/wiki/Data_Distribution_Service) as the middleware for ROS.
One goal of this article is to educate the audience on what DDS is, and how it would fit into the ROS picture.

## Why Consider DDS?

When exploring options for the next generation communication system of ROS, the initial list of candidates were: starting with ROS 1.x transport and improving it or building a new transport very much like the ROS 1.x transport, but using component libraries such as ZeroMQ for transport, Protocol Buffers for message serialization, and ZeroConf (Bonjour or Avahi) for discovery.
However, in addition to those options, which both involved us building a middleware from parts or scratch, other end-to-end middlewares were considered.
One middleware that was always coming up in the research was DDS.

### An End-to-End Middleware

The benefit of using an end-to-end middleware, like DDS, is that there is much less code to maintain and the the behavior and exact specifications of the middleware have already been distilled into documentation.
By using something like DDS, ROS can build on a standard which not only has documents describing the system, but also clearly defines the way in which it should be used and even specifies the software API.
Because of this concrete specification, other organizations can review and audit the design of the middleware and different vendors can make implementations that, in varying degrees, can inter-operate with each other.
This is something that ROS has never had, besides a few basic descriptions in a wiki and a reference implementation.
If eventually the decision is to build a middleware from existing libraries, then this type of specification is something that will also need to be created.

The draw back of using an end-to-end middleware is that ROS must work within the design of that middleware.
If the middleware was designed for a different use case or is not flexible enough for ROS, then it might be necessary to work around the design.
So at some level, by adopting an end-to-end middleware ROS is adopting many of the philosophies and culture of that tool, and that is something that should not be taken lightly.

## What is DDS?

DDS provides a publish-subscribe transport which is very similar to ROS's publish-subscribe transport.
DDS uses the "Interface Description Language (IDL)" as defined by OMG for message definition and serialization.
DDS does not yet provide a request-response style transport, which would be like ROS's Service call system, but a draft for that style of transport is being reviewed right now (2014).

Supporting the publish-subscribe transport, DDS provides a completely distributed discovery system by default.
Because of this, any two DDS programs can communicate without the need for an equivalent tool like the ROS master.
This makes the system more fault tolerant and flexible, but it is not required to use the dynamic discovery mechanism, different DDS vendors provide options for static discovery.

## Where did DDS come from?

DDS got its start as a group of companies which had similar middleware frameworks and became a standard when common customers wanted to get better interoperability between the vendors.
The DDS standard was created by the [Object Management Group](http://www.omg.org/), which are the same people that brought us UML, CORBA, SysML, and other generic software related standards.
Now, depending on your perspective, this may be a positive endorsement or a negative endorsement.
On the one hand you have a standards committee which is perennial and clearly has huge influence on the software engineering community, but on the other hand you have a slow moving body which is slow to adapt to changes in the software engineering community's ethos and therefore arguably doesn't always keep up with the latest trends.

DDS was originally several similar middlewares which eventually became so close to one another that writing a standard to unify them made sense.
So in this way, even though the DDS specification has been written by a committee, it has evolved to its current form by reacting to the needs of its users.
This type of organic evolution of the specification before it was ratified helps to alleviate the concern that the system was designed in a vacuum and that it does not perform well in real environments.
There are some examples of committees coming up with well intentioned and well described specifications that nobody wants to use or doesn't meet the needs of the community it serves, but this does not appear to be the case for DDS.

There is also a concern that DDS is a static specification which was defined and is used in "legacy" systems, but has not kept current.
This kind of stereotype comes from horror stories about things like UML and CORBA, which are also products of OMG.
On the contrary, DDS seems to have an active and organic specification, which in the recent past has added, or is adding, more specifications for things like websockets, security over SSL, extensible types, request and response transport, and a new, more modern C++11 style API specification for the core API to replace the existing C++ interface.
This type of evolution in the standard body for DDS is an encouraging thing to observe, and even though the body is relatively slow, as compared to software engineering technology trends, it is evolving to meet demands of its users.

## Technical Credibility

DDS has an extensive list of installations which are mission critical and impossibly varied.
DDS has been used in:

- battleships
- large utility installations like dams
- financial systems
- space systems
- flight systems
- train switchboard systems

And many other equally important and varied scenarios.
These successful use cases lend credibility to DDS's design being both reliable and flexible.

Not only has DDS met the needs of these use cases, but after talking with users of DDS (in this case government and NASA employees who are also users of ROS), they have all praised its reliability and flexibility.
Those same users will note that the flexibility of DDS comes at the cost of complexity.
The complexity of the API and configuration of DDS is something that ROS would need to address.

The DDS specification is extremely flexible, allowing it to run at the highest level of systems integration as well as on embedded devices.
Several of the DDS vendors have special implementations of DDS for embedded systems which boast specs related to library size and memory footprint on the scale of tens or hundreds of kilobytes.
Since DDS is implemented, by default, on UDP, it does not depend on a reliable transport or medium for communication.
This means that they have to reinvent the reliability wheel (basically TCP), but they gain portability (no reliance on a TCP stack) and they gain control.
Control over several parameters of reliability, what DDS calls Quality of Service (QoS), gives maximum flexibility in controlling the behavior of communication.
For example, if you are concerned about latency, like for soft realtime, you can basically tune DDS to be just a UDP blaster.
In another scenario you might need something that behaves like TCP, but needs to be more tolerant to long dropouts, and with DDS all of these things can be controlled by changing the QoS parameters.

## How would this work with ROS?

DDS provides discovery, message definition, message serialization, and publish-subscribe transport.
