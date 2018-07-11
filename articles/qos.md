---
layout: default
title: ROS 2.0 Quality of Service policies
permalink: articles/qos.html
abstract:
  This article describes the approach to provide QoS (Quality of Service) policies for ROS 2.0.
published: true
author: '[Esteve Fernandez](https://github.com/esteve)'
---

{:toc}

<div class="alert alert-warning" markdown="1">
  This is a work in progress.
  Please check for existing issues/pull requests for related discussion or open new issues/pull requests to propose changes.
</div>

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

With the advent of inexpensive robots using unreliable wireless networks, developers and users need mechanisms to control how traffic is prioritized across network links.

## Background

ROS 1.x uses TCP as the underlying transport, which is unsuitable for lossy networks such as wireless links.
With ROS 2.0 relying on DDS which uses UDP as its transport, we can give control over the level of reliability a node can expect and act accordingly.

## DDS Quality of Service policies

DDS provides fine-grained control over the Quality of Service (QoS) setting for each of the entities involved in the system.
Common entities whose behavior can be modified via QoS settings include: Topic, DataReader, DataWriter, Publisher and Subscriber.
QoS is enforced based on a Request vs Offerer Model, however Publications and Subscriptions will only match if the QoS settings are compatible.

## ROS 2.0 proposal

Given the complexity of choosing the correct QoS settings for a given scenario, it may make sense for ROS 2.0 to provide a set of predefined QoS profiles for common usecases (e.g. sensor data, real time, etc.), while at the same time give the flexibility to control specific features of the QoS policies for the most common entities.

## QoS profiles

A QoS profile defines a set of policies, including durability, reliability, queue depth and sample history storage.
The base QoS profile includes settings for the following policies:

- History.

  - Keep last: only store up to N samples, configurable via the queue depth option.
  - Keep all: store all samples, subject to the configured resource limits of the DDS vendor.

- Depth.

  - Size of the queue: only honored if used together with "keep last".

- Reliability.

  - Best effort: attempt to deliver samples, but may lose them if the network is not robust.
  - Reliable: guarantee that samples are delivered, may retry multiple times.

- Durability.

  - Transient local: *only applies to `DataWriter`s*.
    `DataWriter` becomes responsible of persisting samples until a corresponding `DataReader` becomes available.

  - Volatile: no attempt is made to persist samples.

Note: for each of the main bullets there is also the option of "system default", which uses whatever setting was defined via the DDS vendor tools (e.g. XML configuration files).

ROS 2.0 will provide QoS profiles based on the following use cases:

- Default QoS settings for publishers and subscriptions

  In order to make the transition from ROS1 to ROS2, exercising a similar network behavior is desirable.
  By default, publishers and subscriptions are reliable in ROS2, have volatile durability, and "keep last" history.

- Services

  In the same vein as publishers and subscriptions, services are reliable.
  It is especially important for services to use volatile durability, as otherwise service servers that re-start may receive outdated requests.

- Sensor data

  For sensor data, in most cases it's more important to receive readings in a timely fashion, rather than ensuring that all of them arrive.
  That is, developers want the latest samples as soon as they are captured, at the expense of maybe losing some.
  For that reason the sensor data profile uses best effort reliability and a smaller queue depth.

- Parameters

  Parameters are based on services, and as such have a similar profile.
  The difference is that parameters use a much larger queue depth so that requests do not get lost when, for example, the parameter client is unable to reach the parameter service server.

Profiles allow developers to focus on their applications without worrying about every QoS setting in the DDS specification.

Note: the values in the profiles are subject to further tweaks, based on the feedback from the community.

## Integration with existing DDS deployments

Both PrismTech OpenSplice and RTI Connext support loading of QoS policies via an external XML file.
In environments where DDS is already deployed and also to enable more extensibility other than the offered by the ROS 2.0 and the predefined profiles, ROS 2.0 may provide loading of the QoS settings via the same mechanisms the underlying DDS implementations use.
However, this mechanism will not be added into the common ROS2 API so as to keep the `rmw` layer transport agnostic and let future developers implement it for other transports (e.g. ZeroMQ, TCPROS, etc.)
To honor the QoS settings of the system, developers can use the `rmw_qos_profile_system_default` QoS profile which delegates the responsibility of the QoS machinery to the underlying DDS vendor.
This allows developers to deploy ROS2 applications and use DDS vendor tools to configure the QoS settings.

## Integration with non DDS RMW Implementations

There is no QoS setting that affects the order messages may be received.
Instead an RMW implementation must implement the same behavior as DDS.
Once a subscriber takes a message from the RMW implementation it must not be allowed to take an earlier message from the same publisher.
If the connection is Reliable then newer messages should be kept in the History until all older messages are received.
A best effort connection should drop an older message if it has already received a newer one.

## Open questions

How should the integration of the QoS machinery with intraprocess communication be like.

## References

- Gordon Hunt, [DDS - Advanced Tutorial, Using QoS to Solve Real-World Problems](http://www.omg.org/news/meetings/workshops/RT-2007/00-T5_Hunt-revised.pdf)
- [OpenDDS - QoS Policy Usages](http://www.opendds.org/qosusages.html)
- [OpenDDS - QoS Policy Compliance](http://www.opendds.org/qospolicies.html)
- Angelo Corsaro, [DDS QoS Unleashed](http://www.slideshare.net/Angelo.Corsaro/dds-qos-unleashed)
- Angelo Corsaro, [The DDS Tutorial Part II](http://www.slideshare.net/Angelo.Corsaro/the-dds-tutorial-part-ii)
- [DDS Spec, section 2.2.3](http://www.omg.org/spec/DDS/1.4/PDF/)
- Heidi Schubert, [QoS policies for configuring reliability](https://community.rti.com/content/forum-topic/qos-policies-configuring-reliability)
- [RTI Connext - Comprehensive Summary of QoS Policies](https://community.rti.com/static/documentation/connext-dds/5.2.0/doc/manuals/connext_dds/RTI_ConnextDDS_CoreLibraries_QoS_Reference_Guide.pdf)
- [DDS and Delegated... Durability?](http://blogs.rti.com/2013/12/11/dds-and-delegated-durability/)
