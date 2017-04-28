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
To this end we require that nodes running in the ROS network have a synchronized clock such that they can accurately report timestamps for events.

There are, however, several other use cases where being able to control the progress of the system is important; we review those next.

## Use cases requiring time abstraction

What are the situations where `std::chrono` or other system time mechanisms are insufficient?

When playing back logged data it is often very valuable to support accelerated, slowed, or stepped control over the progress of time.
This control can allow you to get to a specific time and pause the system so that you can debug it in depth.
Data playback does not just affect a single node.
All nodes involved in the playback would need the same clock source.
It is possible to do this with a log of the sensor data. 
However, if the sensor data is out of synchronization with the rest of the system, it will break many algorithms.
For example, the transform system (tf) timestamps messages that contain data computed between two different time/data combos.

Another important use case for using an abstracted time source is when you are running logged data against a simulated robot instead of a real robot.
Depending on the simulation characteristics, the simulator may be able to run much faster than real time or it may need to run much slower.
Running faster than real time can be valuable for high level testing as well allowing for repeated system tests.
Slower than real time simulation is necessary for complicated systems where accuracy is more important than speed.
Often the simulation is the limiting factor for the system and as such the simulator can be a time source for faster or slower playback.
Additionally if the simulation is paused the system can also pause using the same mechanism.

A third situation arises with hardware that doesn't have a system clock chip available. 
Nodes running there need to be notified of the current time so that they can utilize that information to time-stamp their output. 
(We will assume that nodes running on this type of hardware have accurate frequency counters available that can assist in computing the current time when they have some base time to compound.)

### Challenges in using abstracted time

There are many algorithms for synchronization and they can typically achieve accuracies which are better than the latency of the network communications between devices on the network.
However, these algorithms take advantage of assumptions about the constant and continuous nature of time.

Again, this is not the primary intent of synchronizing time between ROS nodes. 
We need to ensure the ability to manipulate time.
In some cases, speeding up, slowing down, or pausing time entirely is important for debugging.
And the ability to support pausing time requires that we not assume that the time values are always increasing.

When communicating the changes in time propagation, the latencies in the communication network become a challenge.
Any change in the time abstraction must be communicated to the other nodes in the graph, but will be subject to normal network communication latency.
This inaccuracy is proportional to the latency of communications and also proportional to the increase in the real time factor.
If very accurate timestamping is required when using the time abstraction, it can be achieved by slowing down the real time factor such that the communication latency is comparatively small.
Of course we recommend using synchronized system clock sources for live work on nodes spread across multiple devices.

The final challenge is that the time abstraction must be able to jump backwards in time, a feature that is useful for log file playback.
This behavior is similar to a system clock after a negative date change, and requires developers using the time abstraction to make sure their algorithm can deal with the discontinuity.
Appropriate APIs must be provided for to the developer API to enable or handle notifications of jumps in time, both forward and backwards.

### Time Abstractions

There will be at least three versions of these abstractions with the following types, `ROSTime`, `SystemTime`, and `SteadyTime`.
All abstractions provide/utilize the same API.
The latter two choices are designed to parallel the [`std::chrono`][] [`system_clock`][] and [`steady_clock`][].
It is expected that the default choice of time will be to use the `ROSTime` source, however the parallel implementations supporting `steady_clock` and `system_clock` will be maintained for use cases where the alternate time source is required.

#### System Time

System Time pulls the current time from the system clock chip. 
It provides benefit over Steady Time when all nodes in the system have synchronized system time available or when all nodes run on the same host that contains a system clock chip.

We expect a System Time publisher to throw an error if the system clock resource is not available.
We expect methods returning the current time to fall back to Steady Time if System Time is not available.

#### Steady Time

Steady Time implies a real-time incrementing source.
There is no modern hardware or operating system that does not support this option.
Steady Time typically begins at system bootup and measures increments using hardware frequency counters.

Example use cases for this include hardware drivers which are interacting with peripherals with hardware timeouts and systems lacking a realtime clock chip.

In nodes which require the use of `SteadyTime` or `SystemTime` for interacting with hardware or other peripherals, it is expected that they do a best effort to isolate any `SystemTime` or `SteadyTime` information inside their implementation and translate external interfaces to use the ROS time abstraction when communicating over the ROS network.

#### ROS Time

ROS Time derives the current time from the most-recently received clock broadcast message published on the `/clock` topic.

When the ROS Time source is active `ROSTime` will return the latest value reported by the Time Source plus the elapsed time since that report multiplied by the real time factor.

The ROS Time broadcast (clock message) needs to include this information:
- The current time (in nanoseconds, epoch depends upon source and should not be needed)
- The current real time factor (also called the "multiplier", typically a value of one, but could be zero for paused or negative when going backwards).
- The system clock time (in nanoseconds since the Unix epoch) when the message was created (or -1 if not available)

This message cannot be latched; to use the real time factor you will need to grab a frequency count (aka, start a stopwatch) when the message is received.
Stale data would make this mechanism fail.
An exception to this is when all nodes have a system clock available and the message contains the system clock stamp.
This sytem timestamp could be used to determine the staleness of the "current time" in the message.
Nodes that publish data before they receive the current time would also be problematic.
It will be helpful to have the clock publisher publish a new message whenever there is a new subscription added to reduce this period of unknown time.

### ROS Time Message Sources

- An included (where?) Steady Clock Publisher.
- An included (where?) System Clock Publisher.
- An included (where?) Gazebo Plugin.
- The ROSBag playback engine.

#### Default Time Source

Users can call `Node::now()` for the current time.
With the assumption of a single Node instance per process, you can also get the current time from a static method: `Time::now()`.
Nodes will automatically subscribe for the `/clock` message unless it is programmatically disabled. (Do we need to disable it? We can simply "not publish" to use the system clock.)
If a publisher exists for the `/clock` topic, it will override the system time when using the ROS time abstraction.
The `ROSTime` and the `now()` methods will fall back to `SystemTime` (and then `SteadyTime`) when an ROS Time Source is not active (aka, it has been disabled purposely or hasn't received a clock broadcast).

Users can also add callbacks to `Node::time_reversing` and `Node::time_jumping_back`.
The developer has the opportunity to register callbacks with the handler to clear any state from their system if necessary before time will be in the past.
If the time on the clock jumps backwards, a callback handler will be invoked and be required to complete before any calls to the ROS time abstraction report the new time.

The frequency of publishing the `/clock` as well as the granularity are not specified as they are application-specific. 
If the realtime multiplier cannot be computed, the message will need to be published very frequently. 

#### Custom Time Source

It is possible that the user may have access to an out-of-band-time source which can provide better performance than the default source the `/clock` topic.
It might be possible that, for their particular use case, a more advanced algorithm is needed to propagate the simulated time with adequate precision for latency arising with restricted bandwidth or connectivity.
The user will be able to switch out the time source for each instance of their Time object as well as have the ability to override the default for the process or node.
This can be acheived in C++ using the template on the Time object. The RCL package also contains a method to disable `ROSTime`.

It is possible to use an external time source such as GPS as a ROS Time source. 
When writing this publisher, ensure that you handle jumps in time (both forward and back) and set the real time factor correctly in those situations.
The preferred option is to use the GPS as an TICSync (or IEEE 1588 PTP or, worst case, NTP) time source instead of a ROS Time source; synchronize your system clocks to the GPS unit's atomic clock.

The `SystemTime`, `SteadyTime`, and `ROSTime` API's will be provided by each client library in an idiomatic way, but they may share a common implementation, e.g. provided by `rcl`.
However, if a client library chooses to not use the shared implementation then it must implement the functionality itself.

### Public API

In each implementation will provide `Time`, `Duration`, and `Rate` datatypes, for all three time source abstraction..
The `Duration` will support a `sleep_for` function as well as a `sleep_until` method.
The implementation will also provide a `Timer` object which will provide periodic callback functionality for all the abstractions.

#### RCL implementation

In `rcl` there will be datatypes and methods to implement each of the three time abstractions for each of the core datatypes.
However at the `rcl` level the implementation will be incomplete as it will not have a threading model and will rely on the higher level implementation to provide any threading functionality such as is required by sleep methods.

It will provide implementations parallel to the public datastructures and storage which the client library can depend upon and to which it can delegate.
The underlying datatypes will also provide ways to register notifications, however it is the responsibility of the client library implementation to collect and dispatch user callbacks.

<div class="alert alert-warning" markdown="1">
  <b>TODO:</b> Enumerate the <code>rcl</code> datastructures and methods here.
</div>

## Work Needing To Be Done (Apr 2017)

- Decide where and when to autosubscribe to the `/clock` message. Built in to Node? A Node addon or service? Part of the first call to `rcl_node_init`?
- Decide how to deal with multiple nodes in a single process that all update the one static time (or decide to ditch the static time methods).
- Decide where to put the out-of-the-box clock publishers.
- Decide how to deal with nodes publishing data before they receive the current `ROSTime` update.
- Ensure that there is no reason we can't enable `ROSTime` (with fallbacks) by default.
- Change the RCL time implementation (in `rcl_get_ros_time`) to call a method on the time source to return the current time (rather than using a field). This is necessary to compute the current time from the real time factor. 
- Add an `rcl_time_source_t` (to the RCL project) for `ROSTime` that includes a method to compute the current time.


If we decide to opt-in to ROS Time, you might use an interface like this in C++:
```cpp
class ROSTimeManager {
  public:
    ROSTimeManager(Node::shared_ptr node, bool overrideStaticTimeNow = true, bool allowFallbackToSystemClock = true, milliseconds initialBlockWaitingForClockMsg = 50ms);
    Time now()
    void register_clock_message_received_handler(std::function ...)
    void register_time_about_to_reverse_handler(std::function ...)
    void register_time_about_to_jump_backwards_handler(std::function ...)
    ...
```

## References

The default time source is modeled on the ROS Clock and ROS Time system used in ROS 1.0.
For more information on the implementation in ROS 1.0 see:

- [ROS Clock Documentation](http://wiki.ros.org/Clock)
- [rospy Time Documentation](http://wiki.ros.org/rospy/Overview/Time)
- [roscpp Time Documentation](http://wiki.ros.org/roscpp/Overview/Time)

### Real Time

In this article the term 'real time' is used to express the true rate of progression of time.
This is not connected to 'real-time' computing with deterministic deadlines.

[`std::chrono`]: http://en.cppreference.com/w/cpp/chrono
[`steady_clock`]: http://en.cppreference.com/w/cpp/chrono/steady_clock
[`system_clock`]: http://en.cppreference.com/w/cpp/chrono/system_clock
