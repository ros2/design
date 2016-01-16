---
layout: default
title: Proposal for Implementation of Real-time Systems in ROS 2
permalink: articles/realtime_proposal.html
abstract: Proposal for a test-driven approach to the real-time performance requirement in ROS 2.
published: true 
author: Jackie Kay
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Requirements and Implementation of Real-Time Systems

### System requirements

The requirements of a real-time system vary depending on the use case.
At its core, the real-time requirement of a system has two components:

- Latency

  - Update period (also known as deadline)
  - Predictability

- Failure mode

  - How to react to a missed deadline

Language referring to hard/soft/firm real-time systems generally refers to the failure mode.
A hard real-time system treats a missed deadline as a system failure.
A soft real-time system tries to meet deadlines but does not fail when a deadline is missed.
A firm real-time system discards computations made by missed deadlines and may degrade its performance requirement in order to adapt to a missed deadline.

Though in this document we consider the failure mode orthogonal to latency characteristics, a hard real-time failure mode is often associated with high predictability (low jitter).
Safety-critical systems often require a hard real-time system with high predictability.

### Implementation choices: computer architecture and OS

Processors vary in capability, number of cores, memory size, and instruction set architectures, some of which are better suited to real-time performance than others.
The selection of computer architecture may be due to cost, size, or energy efficiency, which could set a constraint for the rest of the system.
A single robot may contain multiple computers with a variety of processor types, from the main control board to sensors.

Another major consideration when designing a real-time system is the choice of operating system.
A "hard" real-time operating system is truly deterministic.
"Soft" real-time operating systems are designed to meet most deadlines with low jitter, but are nondeterministic.
Most conventional user-facing operating systems are not designed with fairness and predictability as the first priorities, often because they have to support graphical displays and interactivity.
Here are some features critical to predictable performance that are often exposed to the user in an RTOS, but not in a conventional OS (see Wikipedia, [Real-time operating system](https://en.wikipedia.org/wiki/Real-time_operating_system)).
For more information about why these features matter, refer to the [introductory article on real-time systems](http://design.ros2.org/articles/realtime_background.html).

- Task scheduling

  - Expose thread priority and deterministic schedulers that incorporate priority
  - Allow full thread preemption

- Concurrency and synchronization primitives

  - Floored semaphores (allow a higher priority thread to take control of locked semaphore)

- Interrupt handling

  - Disable interrupts
  - Mask interrupts (temporarily suspend interrupts in a critical section)

- Memory allocation

  - Avoid pagefaults
  - Avoid nondeterministic heap allocation algorithms

Some real-time systems have a non real-time component, requiring inter-communication between the real-time and non-real-time components.
An example from the ROS 1 world is real-time Orocos components communicating with ROS nodes.
Another example is real-time code with a graphical or user interaction component.

Scheduling non-real-time threads alongside real-time threads on the same core can introduce nondeterminism into a system.
The non-real-time threads must have a lower priority than real-time threads in order to not interfere with real-time deadlines.
The real-time threads should never be blocked on the non-real-time threads and should never be pre-empted by non-real-time threads.

Real-time operating systems have a variety of methods for enforcing these constraints.
For example, a Linux RT_PREEMPT system could use the [SCHED_IDLE](http://man7.org/linux/man-pages/man2/sched_setscheduler.2.html) scheduling policy for non-real-time threads.
Systems like QNX and Xenomai give the option to partition real-time and non-real-time components onto different cores, or even different kernels.
Partitioning is the safest approach to real-time and non-real-time intercommunication because it minimizes the change of interference from non-real-time threads, with the added cost of latency due to coordinating data between the partitions.

Another important constraint is which processor architectures are supported by which operating systems, and which RTOS is an appropriate choice due to performance constraints of the architecture.
For example, an architecture that doesn't support SIMD instructions cannot take full advantage of parallelism, but if the system doesn't run an OS that supports threading, or doesn't have multiple cores, this constraint doesn't matter.

## Goal for ROS 2

In order to support real-time systems in ROS 2, we propose an iterative approach for identifying limitations in the API that force the code path into nondeterministic/unpredictable behavior and performance bottlenecks.

1. Select two use cases with real-time requirements that represent two extremes of the space described above, for example:

   - An STM32 embedded board running no operating system or a hard real-time OS with 1 kHZ update frequency with 3% allowable jitter.
   - An x86 computer running Xenomai or RT_PREEMPT that receives the sensor data over Ethernet or USB and processes the output in soft or firm real-time, but also runs separate non-RT processes for user interaction.

2. Implement the system with ROS 2.

3. Profile the performance of the system.

4. Identify solutions to improve the performance of the system, e.g. by exposing more concurrency primitives through the appropriate abstraction.

5. Adapt the ROS 2 API or underlying libraries to implement these solutions and to improve the experience of writing real-time code.

6. Refactor the system for the new API if necessary and repeat.

## Software architecture challenges and solutions

Wherever ROS 2 libraries expose sources of nondeterminism, there should be a path that exposes an alternative.
This section evaluates the current state of ROS 2 and rclcpp, predicts challenges to fulfilling real-time requirements, and presents possible solutions.

### Memory Allocation

Poor memory management can create performance issues due to page faults and nondeterministic algorithms for dynamically allocating memory.
Although C++ has an advantage over many programming languages because memory management is exposed, the default memory allocator for operator `new` and dynamically sized data structures is not well-suited to real-time applications (see [Composing High-Performance Memory Allocators](https://people.cs.umass.edu/~emery/pubs/berger-pldi2001.pdf)).
Dynamically-sized data structures in the C++ standard library offer an allocator interface, such as `std::vector` (see [cppreference.com](http://en.cppreference.com/w/cpp/concept/Allocator)).
C++11 greatly improved this interface by introducing [`std::allocator_traits`](http://en.cppreference.com/w/cpp/memory/allocator_traits), which constructs an allocator compatible with the standard allocator interface from a minimal set of requirements
To allow for optimal performance in `rclcpp`, the client library should expose custom allocators for publishers, subscribers, etc. that are propagated to standard library structures used by these entities.

As of the writing of this document (January 2016), selecting custom allocators in `rclcpp` is almost fully implemented.
The [`realtime_support`](https://github.com/ros2/realtime_support) package will offer real-time memory allocators and example code for real-time ROS 2.
An example of the ROS 2 allocator API using the Two Level Segregate Fit allocator is currently available [in realtime_support](https://github.com/ros2/realtime_support/blob/master/tlsf_cpp/example/allocator_example.cpp).

The following work remains to be done:

- Improve the interface in rclcpp such that the user need only provide an "allocate" function, a "deallocate" function, and an optional pointer to the allocator state (this interface is currently outlined in the rcl API).
- Custom allocators for services/clients.
- Custom allocators for parameters.
- Fully implement and test the allocator interface in rcl, rclc, and rmw.

### Scheduling and synchronization

Parallelism is valuable for real-time systems because it can reduce latency.

Even in single-core architectures, concurrency is important to us because asynchronous patterns are common in ROS and ROS 2.

The typical approaches to synchronizing concurrent access to data tend to decrease the predictability of a system and increase the complexity.
For example, mutexes introduce the possibility of deadlocks when used improperly.
As mentioned above, many real-time operating systems offer alternative synchronization primitives for real-time safety.

Guidelines for synchronization in ROS 2:

- Prefer atomics to mutexes when possible.

  - Many atomic types are not implemented as lock-free due to architectural limitations, can check in C++ with [`std::atomic_is_lock_free`](http://en.cppreference.com/w/cpp/atomic/atomic_is_lock_free)

- Thoroughly test multithreaded code for deadlock and livelock at the earliest opportunity.

  - Consider compiling multithreaded regression tests with the [Clang ThreadSanitizer](http://clang.llvm.org/docs/ThreadSanitizer.html).

- Hide synchronization primitives behind a layer of abstraction that wraps standard library classes such as `std::mutex` and `std::condition_variable` as the default behavior.

  - Inject alternative synchronization primitives from RTOS-specific APIs where applicable.

As of the time of writing, the Executor and the IntraProcessManager in rclcpp are designed to partially follow the last guideline.
However, the abstractions are messy (particularly for the IntraProcessManager).
There is no real-time multithreaded Executor that uses "real-time safe" synchronization primitives, similarly there is no such IntraProcessManager--there is only the abstraction path that would allow such alternatives to be implemented.
There are also synchronization primitives that are not hidden behind an abstraction layer for the global interrupt handler and for parameters.

### Middleware selection and QoS

If the underlying middleware used in ROS 2 is not suitable for real-time performance, the entire system will fail to meet the real-time requirement.
Network communication and discovery, particularly asynchronous communication, introduces a nondeterministic element into software.
Though some real-time operating systems partially mitigate this (e.g. disabling interrupts), careful selection and use of a middleware is important.

The embedded component of the system proposed will likely use `freertps` as the underlying middleware.
The "desktop" system can be matrix-tested using the various DDS implementations supported by ROS 2.
This allows us to compare the real-time performance of the different DDS implementations.

Quality of service is always an important consideration with DDS.
Some DDS QoS options were designed with real-time performance in mind, such as the DEADLINE policy (see opendds.org, [QoS Policy Usages](http://www.opendds.org/qosusages.html)).
It may be convenient to expose these policies and provide a "real-time" QoS profile in rmw as a suggestion to ROS 2 users.
Benchmarking and profiling may provide data to support the introduction of more QoS options into the rmw API.

### Component lifecycle

It should be noted that the planned component lifecycle features are useful for real-time programming, since it is common that real-time systems have a initialization phase in which the real-time constraint is relaxed, in which it is safe to preallocate memory, change thread priority, etc.
Distinguishing between initialization and execution could lead to better self-verification of real-time constraints during the execution phase (see the ROS 2 design article on [managed nodes](http://design.ros2.org/articles/node_lifecycle.html)).

## Testing

The proposed system must be tested for correctness and expected behavior.
The performance requirements of the system, such as latency, must also be benchmarked, and the inner workings of the system must be accessible, in order to identify bottlenecks.
Therefore we propose a mix of automated unit tests, performance benchmarking, and profiling/introspection tools.

### Benchmarking

The system will be subjected to varying levels of stress to benchmark "best" and "worst"-case performance statistics.

Stress can be applied "externally" via polluting the cache and running many garbage processes alongside the test system (if on an OS that supports processes).
It could also be added "internally" (to ROS 2) by arbitrarily adding more publishers, subscribers, clients, services, etc.

Possible performance metrics include:

- Minimum, maximum, and average latency

  - Latency of the update loop
  - Message round trip time over the middleware

- Standard deviation of latency

- Number of missed deadlines/overruns

- Number of messages received vs. number of messages published

`rttest` is an existing library developed to instrument the update loop of a real-time process and report on various metrics described here.
However, it contains code specific to Linux pthreads in its current state.
If it is to be used in this work, the library needs to be generalized to support other operating systems.

### Profiling Tools

In addition to gathering performance statistics, it is useful to introspect the conditions under which the statistics were gathered to pinpoint areas where the code needs improvement.

Two well-established open source command line profiling tools with orthogonal methodologies are:

- `perf`: counts system-wide hardware events, e.g. instructions executed, cache misses, etc. (see [perf wiki](https://perf.wiki.kernel.org/index.php/Main_Page))

  - Linux only.

- `gprof`: recompiles source code with instrumentation to count time spent in each function for a particular program, with option to format output in a call graph (see [GNU gprof documentation](https://sourceware.org/binutils/docs/gprof/)).

  - GNU and C/C++ only.

However, these particular tools may not be well suited to an RTOS or a bare metal embedded system: `perf` is specialized for the Linux kernel and `gprof` requires code to be compiled and linked with `gcc`.
These tools could be used to profile ROS 2 code on a patched Linux kernel like RT_PREEMPT, but there may be performance quirks in another RTOS that are not captured on Linux.
When this proposal is implemented, if changes are made to improve performance is made based on profiling data from Linux but no overall improvement is seen in the embedded system, then an alternative approach to profiling should be investigated.
For example, the [RTEMS](https://devel.rtems.org/wiki/Developer/Tracing) real-time operating system offers a tracing tool that could be used for profiling (8).

### Verification

In addition to performance testing, which can be thought of as a kind of regression testing, any change made to ROS 2 with the intent to improve performance in the real-time test system should be verified with an automated test.
For example, the allocator pipeline has a test case for publishers/subscribers with custom allocators that fails if calls are made to the default system allocator during the real-time execution phase.
A similar test could be written for the proposed goal of abstracting synchronization primitives, though the implementation may be more challenging since the allocator pipeline was helped by the ability to override global new in C++.

The expected response of a hard or firm real-time system to missed deadlines should also be thoroughly tested.

## Timeline

This proposal represents a short burst of work to develop the initial test system, followed by extended maintenance as testing identifies performance bottlenecks.

Since a key part of this proposal is the development of a hard real-time embedded system, this work depends on the development of `freertps` and `rmw_freertps` and the porting of `rclc` to STM32 or another microcontroller type.

- Alpha 4 (2/12/15): Create initial system and test suite (2-3 weeks).
- Every release thereafter: Iterate on the API, refactor and expand test suite as new features are added to core ROS 2 libraries (1-2 weeks each release cycle).
