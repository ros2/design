---
layout: default
title: ROS 2.0 in Realtime
permalink: articles/realtime.html
abstract: This article describes ROS 2.0's planned compatibility with real-time computing requirements.
published: true
author: Jackie Kay
---

* This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

Robotic systems need to be responsive. In mission critical applications, a delay of less than a
millisecond in the system can cause a catastrophic failure. For ROS 2.0 to capture the needs
of the robotics community, the core software components must not interfere with the requirements of real-time computing.

# Definition of Real-time Computing

Real-time software guarantees correct computation at the correct time. Hard real-time software systems
have a set of strict deadlines, and missing a deadline is considered a failure. Soft real-time systems
degrade their quality of service if a deadline is missed.

Real-time computer systems are often associated with low-latency systems. Many applications of real-time
computing are also low-latency applications (for example, automated piloting systems must be reactive to
sudden changes in the environment). However, it is
generally agreed upon that a real-time system is not defined by low latency, but by a deterministic schedule:
it must be guaranteed that the system finishes a certain task by a certain time. Therefore, it is
important that the latency in the system be measurable and a maximum allowable latency for tasks be
set.

A more useful metric in evaluating the "hardness" of a real-time system is jitter, which is defined
as the variable deviation between a task's deadline and its actual time of completion. A more jittery
system is less deterministic, less predictable, and less real-time. Though in practice it is impossible
to completely eliminate jitter from a real-time system, it is a worthy goal to determine a hard upper bound
for jitter.

# Implementation of Real-time Computing

In general, an operating system can guarantee that the tasks it handles for the developer, such as
thread scheduling, are deterministic, but the OS may not guarantee that the developer's code will
run in real-time. Therefore, it is up to the developer to know what the determinstic guarantees
of an existing system are, and what she must do to write hard real-time code on top of the OS.

In this section, various strategies for developing on top of a real-time
OS are explored, since these strategies might be applicable to ROS 2. The patterns focus on the
use case of C/C++ development on Linux-based real-time OS's (such as RTLinux), but the general
concepts are applicable to other platforms.

## Memory management

### Lock memory, prefault stack:

```c
    if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
            perror("mlockall failed");
            exit(-2);
    }
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
``` `

`mlockall` is a Linux system call for locking the process's virtual address space into RAM,
preventing the memory that will be accessed by the process from getting paged into swap space. Paging
and accessing paged memory is a nondeterministic operation that should be avoided in real-time computation.

This code snippet, when run at the beginning of a thread's lifecycle, ensures that no pagefaults occur
while the thread is running. `mlockall` locks the stack for the thread. The `memset` call pre-loads each
block of memory of the stack into the cache, so that no pagefaults will occur when the stack is accessed.

### Allocate dynamic memory pool

```c
  if (mlockall(MCL_CURRENT | MCL_FUTURE))
    perror("mlockall failed:");

  /* Turn off malloc trimming.*/
  mallopt(M_TRIM_THRESHOLD, -1);

  /* Turn off mmap usage. */
  mallopt(M_MMAP_MAX, 0);

  page_size = sysconf(_SC_PAGESIZE);
  buffer = malloc(SOMESIZE);

  for (i=0; i < SOMESIZE; i+=page_size)
  {
      buffer[i] = 0;
  }
  free(buffer);
``` `

It is commonly thought that dynamic memory allocations are not permitted in the real-time code path.
This code snippet shows how to lock the virtual address space, disallow returning deallocated memory
to the kernel via `sbrk`, and disable `mmap`. It effectively locks a pool of memory in the heap into RAM,
which prevents page faults due to `malloc` and `free`.

Pros:

* Can use malloc/new, free/delete, and even STL containers

Cons:

* Must accurately predict bounded memory size for the process!
* Using STL containers is therefore dangerous (unbounded sizes)
* In practice, only works for small processes

### Custom fixed allocators for STL containers

An alternative to the above approach is to implement custom allocators for STL containers
that only allocate memory on the stack.

There are various comprehensive tutorials
[already written](http://www.codeguru.com/cpp/article.php/c18503/C-Programming-Stack-Allocators-for-STL-Containers.htm)
for this task.

Pros:
* Use existing STL code with deterministic computation and without allocating global dynamic memory pool
* More modular solution

Cons:
* Complex to implement

### Global variables and (static) arrays

Global variables are preallocated at the start of a process, thus assigning and accessing them
is "real-time" safe. However, this strategy comes with the many disadvantages of using global
variables.

### Use inheritance sparingly

Classes with many levels of inheritance may not be real-time safe because of vtable overhead access.
When executing an inherited function, the program needs to access the data used in the function,
the vtable for the class, and the instructions for the function, which are all stored in different
parts of memory, and may or may not be stored in cache together.

In general, C++ patterns with poor cache locality are not well-suited to real-time environments.
Another such pattern is the opaque pointer idiom (PIMPL), which is convenient for ABI compatibility
and speeding up compile times. However, bouncing between the memory location for the object and its
private data pointer causes the cache to "spill" as it loads one chunk of memory and then another,
unrelated chunk for almost every function in the PIMPLized object.

### Exceptions

Throwing an exception can put large objects on the stack, which is undesirable in real-time
programming since we cannot allocate memory on the heap. On modern C++ compilers, catching an
exception has little memory or time overhead, but can lead to unexpected code growth.

## Device I/O
Interacting with physical devices (disk I/O, printing to the screen, etc.) may introduce
unacceptable latency in the real-time code path, since the process is often forced to wait
on slow physical phenomena. Additionally, many I/O calls such as `fopen` result in pagefaults.

Keep disk reads/writes at the beginning or end of the program, outside of the RT code path.

Spin up userspace threads (executing on a different CPU from the RT code) to print output to
the screen.

## Multithreaded Programming and Synchronization

Real-time computation requirements change the typical paradigm of multithreaded programming.
Program execution may not block asynchronously, and threads must be scheduled deterministically.
A real-time operating system will fulfill this scheduling requirement, but there are still pitfalls
for the developer to fall into. Typical assumptions about locking critical sections will fall apart
if thread execution can be arbitrarily preempted.

### Thread creation guidelines

Create threads at the start of the program. This confines the nondeterministic overhead of thread
allocation to a defined point in the process.

Create high priority (but not 99) threads with a FIFO or Round Robin scheduler.

### Avoid priority inversion
Priority inversion can occur on a system with a preemptive task scheduler and results in deadlock.
It occurs when: a low-priority task acquires a lock and is then pre-empted by a
medium-priority task, then a high-priority task acquires the lock held by the low-priority task.

The three tasks are stuck in a triangle: the high-priority task is blocked on the low-priority task,
which is blocked on the medium-priority task because it was preempted by a task with a higher
priority, and the medium-priority task is also blocked on a task with a higher priority.

Here are the some solutions to priority inversion:

* Don't use locks (but sometimes, they are necessary)
* Disable preemption for tasks holding locks (can lead to jitter)
* Increase priority of task holding a lock
* Use priority inheritance: a task that owns a lock inherits the priority of a task that tries to acquire the lock

### Timing shots

One real-time synchronization technique is when a thread calculates its next "shot" (the start of its
next execution period). For example, if a thread is required
to provide an update every 10 milliseconds, and it must complete an operation that takes 3-6 milliseconds,
the thread should get the time before the operation, do the operation, and then wait for the remaining
7-4 milliseconds, based on the time measured after the operation.

The most important consideration for the developer is to use a high precision timer, such as `nanosleep`
on Linux platforms, while waiting. Otherwise the system will experience drift.

### Spinlocks
Spinlocks tend to cause clock drift. The developer should avoid implementing his own spinlocks. The
RT Preempt patch replaces much of the kernel's spinlocks with mutexes, but this might not be
guaranteed on all platforms.

# Testing and Performance Benchmarking

## Static code checking

## cyclictest

cyclictest is a simple Linux command line tool for measuring the jitter of a real-time environment. It
takes as input a number of threads, a priority for the threads, and a scheduler type. It spins up
`n` threads that sleep regular intervals (the sleep period can also be spceified from the command line).



## Test Pipeline

Instrumenting real-time code to validate its correctness may be tricky because the time overhead due to
instrumentation. A simple benchmarking test that does not involve code instrumentation is as follows:

On an RTLinux machine, start script that runs the process to be tested for some fixed time interval.
Collect data with cyclictest to record the jitter in the system. A variety of command line tools could be
used to add extra stress to the system in a controlled way, such as
[cache calibrator](http://homepages.cwi.nl/~manegold/Calibrator/) to pollute the cache in an attempt to
create pagefaults, or `fping`, which generates a large number of interrupts. Tools for adding load to the
system could also be used to test QoS settings in DDS and how that affects real-time performance.

This procedure could be run on a suite of example cases: a simple benchmark program known to run in real
time, inter-process DDS communication, intra-process DDS communication,
inter-process ROS 2 communication, intra-process ROS 2 communication, another benchmark program that
obviously does not run in real time (lots of disk I/O, dynamic memory allocation, and blocking).

TODO: Latency test

TODO: Test schemes for different embedded platforms

# Design Guidelines for ROS 2

## Achieving real-time computation across platforms

TODO: How can ROS 2 be real-time friendly and cross-platform? Much of the research in this document
focuses on achieving real-time on Linux systems with pthreads.

## Implementation strategy

There are a few possible strategies for the real-time "hardening" of existing and future ROS 2 code:

* Create a configuration option for the stack to operate in "real-time friendly" mode.
  * Pros:
    * Allows user to dynamicaly switch between real-time and non-real-time modes.
  * Cons:
    * Refactoring overhead. Integrating real-time code with existing code may be intractable.
    * Total code size may be impractical for embedded systems.

* Implement a new real-time stack (rclrt, rmwrt, etc.) designed with real-time computing in mind.
  * Pros:
    * Easier to design and maintain.
    * Real-time code is "quarantined" from existing code. Can fully optimize library for real-time application.
  * Cons:
    * More packages to write and maintain.
    * Potentially less convenient for the user.

* Ensure option real-time safety up to a certain point in the stack, and implement a real-time safe language wrapper
(rclrt or rclc)
  * Pros:
    * Existing code is designed for this refactoring to be fairly easy
    * User can provide memory allocation strategy to rcl/rmw to ensure deterministic operation
    * Synchronization happens at the top the language/OS-specific layer, so refactoring rcl/rmw is easier
    * May be easier to support multiple embedded platforms
  * Cons:
    * Refactoring overhead
    * More flexibility for user may mean more complexity


# Sources

* [Real-Time Linux Wiki](https://rt.wiki.kernel.org/)

* Scott Salmon, [How to make C++ more real-time friendly](http://www.embedded.com/design/programming-languages-and-tools/4429790/2/How-to-make-C--more-real-time-friendly)

* Stack Overflow, [Are Exceptions still undesirable in Realtime environment?](http://stackoverflow.com/questions/5257190/are-exceptions-still-undesirable-in-realtime-environment)

* Pavel Moryc, [Task jitter measurement under RTLinux operating system](https://fedcsis.org/proceedings/2007/pliks/48.pdf)
