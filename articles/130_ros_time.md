---
layout: default
title: Clock and Time
permalink: articles/clock_and_time.html
abstract:
  This article describes the ROS primitives to support programming which can run both in real time as well as simulated time which may be faster or slower.
published: true
author: '[Tully Foote](https://github.com/tfoote)'
---

- This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Background

Many robotics algorithms inherently rely on timing as well as synchronization.
To this end we require that nodes running in the ROS network have a synchronized system clock such that they can accurately report timestamps for events.

There are however several use cases where being able to control the progress of the system is important.

### Real Time vs real-time computing

In this article the term 'real time' is used to express the true rate of progression of time.
This is not connected to 'real-time' computing with deterministic deadlines.

## Use cases requiring time abstraction

When playing back logged data it is often very valuable to support accelerated, slowed, or stepped control over the progress of time.
This control can allow you to get to a specific time and pause the system so that you can debug it in depth.
It is possible to do this with a log of the sensor data, however if the sensor data is out of synchronization with the rest of the system it will break many algorithms.

Another important use case for using an abstracted time source is when you are running logged data against a simulated robot instead of a real robot.
Depending on the simulation characteristics, the simulator may be able to run much faster than real time or it may need to run much slower.
Running faster than real time can be valuable for high level testing as well allowing for repeated system tests.
Slower than real time simulation is necessary for complicated systems where accuracy is more important than speed.
Often the simulation is the limiting factor for the system and as such the simulator can be a time source for faster or slower playback.
Additionally if the simulation is paused the system can also pause using the same mechanism.

## Approach

To provide a simplified time interface we will provide a ROS time and duration datatype.
To query for the latest time a ROS Clock interface will be provided.
A TimeSource can manage one or more Clock instances.


## Clock


### Challenges in using abstracted time

There are many algorithms for synchronization and they can typically achieve accuracies which are better than the latency of the network communications between devices on the network.
However, these algorithms take advantage of assumptions about the constant and continuous nature of time.

An important aspect of using an abstracted time is to be able to manipulate time.
In some cases, speeding up, slowing down, or pausing time entirely is important for debugging.

The ability to support pausing time requires that we not assume that the time values are always increasing.

When communicating the changes in time propagation, the latencies in the communication network becomes a challenge.
Any change in the time abstraction must be communicated to the other nodes in the graph, but will be subject to normal network communication latency.
This inaccuracy is proportional to the latency of communications and also proportional to the increase in the rate at which simulated time advances compared to real time (the "real time factor").
If very accurate timestamping is required when using the time abstraction, it can be achieved by slowing down the real time factor such that the communication latency is comparatively small.

The final challenge is that the time abstraction must be able to jump backwards in time, a feature that is useful for log file playback.
This behavior is similar to a system clock after a negative date change, and requires developers using the time abstraction to make sure their algorithm can deal with the discontinuity.
Appropriate APIs must be provided for to the developer API to enable notifications of jumps in time, both forward and backwards.

### Time Abstractions

There will be at least three versions of these abstractions with the following types, `SystemTime`, `SteadyTime` and `ROSTime`.
These choices are designed to parallel the [`std::chrono`][] [`system_clock`][] and [`steady_clock`][].
It is expected that the default choice of time will be to use the `ROSTime` source, however the parallel implementations supporting `steady_clock` and `system_clock` will be maintained for use cases where the alternate time source is required.

#### System Time

For convenience in these cases we will also provide the same API as above, but use the name `SystemTime`.

`SystemTime` will be directly tied to the system clock.

#### ROS Time

The `ROSTime` will report the same as `SystemTime` when a ROS Time Source is not active.
When the ROS time source is active `ROSTime` will return the latest value reported by the Time Source.
`ROSTime` is considered active when the parameter `use_sim_time` is set on the node.

#### Steady Time

Example use cases for this include hardware drivers which are interacting with peripherals with hardware timeouts.

In nodes which require the use of `SteadyTime` or `SystemTime` for interacting with hardware or other peripherals it is expected that they do a best effort to isolate any `SystemTime` or `SteadyTime` information inside their implementation and translate external interfaces to use the ROS time abstraction when communicating over the ROS network.

### ROS Time Source

#### Default Time Source

To implement the time abstraction the following approach will be used.

The time abstraction can be published by one source on the `/clock` topic.
The topic will contain the most up to date time for the ROS system.
If a publisher exists for the topic, it will override the system time when using the ROS time abstraction.
If `/clock` is being published, calls to the ROS time abstraction will return the latest time received from the `/clock` topic.
If time has not been set it will return zero if nothing has been received.
A time value of zero should be considered an error meaning that time is uninitialized.

If the time on the clock jumps backwards, a callback handler will be invoked and be required to complete before any calls to the ROS time abstraction report the new time.
Calls that come in before that must block.
The developer has the opportunity to register callbacks with the handler to clear any state from their system if necessary before time will be in the past.

The frequency of publishing the `/clock` as well as the granularity are not specified as they are application specific.

##### No Advanced Estimating Clock By Default

There are more advanced techniques which could be included to attempt to estimate the propagation properties and extrapolate between time ticks.
However all of these techniques will require making assumptions about the future behavior of the time abstraction.
And in the case that playback or simulation is instantaneously paused, it will break any of these assumptions.
There are techniques which would allow potential interpolation, however to make these possible it would require providing guarantees about the continuity of time into the future.
For more accuracy the progress of time can be slowed, or the frequency of publishing can be increased.
Tuning the parameters for the `/clock` topic lets you trade off time for computational effort and/or bandwidth.

#### Custom Time Source

It is possible that the user may have access to an out of band time source which can provide better performance than the default source the `/clock` topic.
It might be possible that for their use case a more advanced algorithm would be needed to propagate the simulated time with adequate precision or latency with restricted bandwidth or connectivity.
The user will be able to switch out the time source for the instance of their Time object as well as have the ability to override the default for the process.

It is possible to use an external time source such as GPS as a ROSTime source, but it is recommended to integrate a time source like that using standard NTP integrations with the system clock since that is already an established mechanism and will not need to deal with more complicated changes such as time jumps.

For the current implementation a `TimeSource` API will be defined such that it can be overridden in code.
If in the future a common implementation is found that would be generally useful it could be extended to optionally dynamically select the alternative TimeSource via a parameter similar to enabling the simulated time.

## Implementation

The `SystemTime`, `SteadyTime`, and `ROSTime` API's will be provided by each client library in an idiomatic way, but they may share a common implementation, e.g. provided by `rcl`.
However, if a client library chooses to not use the shared implementation then it must implement the functionality itself.

`SteadyTime` will be typed differently than the interchangable `SystemTime` and `ROSTime`.
This is because SystemTime and ROSTime have a common base class with runtime checks that they are valid to compare against each other.
However SteadyTime is never comparable to SystemTime or ROSTime so it would actually have a separate implementation of the same API.
Thus you could get protection from misusing them at compile time (in compiled languages) instead of only catching it at runtime.

### Public API

The implementation from client library will provide `Time`, `Duration`, and `Rate` datatypes, for all three time source abstractions.

The `Clock` will support a `sleep_for` function as well as a `sleep_until` method using a `Duration` or `Time` argument respectively.
The implementation will also provide a `Timer` object which will provide periodic callback functionality for all the abstractions.

It will also support registering callbacks for before and after a time jump.
The first callback will be to allow proper preparations for a time jump.
The latter will allow code to respond to the change in time and include the new time specifically as well as a quantification of the jump.

Any API which is blocking will allow a set of flags to indicate the appropriate behavior in case of time jump.
This will allow the user to choose to error immediately on a time jump or choose to ignore.
When registering a callback for jumps a filter for the minimum backwards or forwards distance will be possible and well as whether a clock change is to be included.

### RCL implementation

In `rcl` there will be datatypes and methods to implement each of the three time abstractions for each of the core datatypes.
However at the `rcl` level the implementation will be incomplete as it will not have a threading model and will rely on the higher level implementation to provide any threading functionality which is required by sleep methods.
It also will require appropriate threading to support the reception of `TimeSource` data.

It will provide implementations parallel to the public datastructures and storage which the client library can depend upon and to which it can delegate.
The underlying datatypes will also provide ways to register notifications, however it is the responsibility of the client library implementation to collect and dispatch user callbacks.

<div class="alert alert-warning" markdown="1">
  <b>TODO:</b> Enumerate the <code>rcl</code> datastructures and methods here.
</div>

## References

The default time source is modeled on the ROS Clock and ROS Time system used in ROS 1.0.
For more information on the implementation in ROS 1.0 see:

- [ROS Clock Documentation](http://wiki.ros.org/Clock)
- [rospy Time Documentation](http://wiki.ros.org/rospy/Overview/Time)
- [roscpp Time Documentation](http://wiki.ros.org/roscpp/Overview/Time)

[`std::chrono`]: http://en.cppreference.com/w/cpp/chrono
[`steady_clock`]: http://en.cppreference.com/w/cpp/chrono/steady_clock
[`system_clock`]: http://en.cppreference.com/w/cpp/chrono/system_clock
