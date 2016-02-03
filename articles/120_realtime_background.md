---
layout: default
title: Introduction to Real-time Systems
permalink: articles/realtime_background.html
abstract: This article is a brief survey of real-time computing requirements and methods to achieve real-time performance.
published: true
author: Jackie Kay
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

This document seeks to summarize the requirements of real-time computing and the challenges of implementing real-time performance.
It also lays out options for how ROS 2 could be structured to enforce real-time compatibility.

Robotic systems need to be responsive.
In mission critical applications, a delay of less than a millisecond in the system can cause a catastrophic failure.
For ROS 2.0 to capture the needs of the robotics community, the core software components must not interfere with the requirements of real-time computing.

## Definition of Real-time Computing

The definition of real-time computing requires the definition of a few other key terms:

- Determinism: A system is deterministic if it always produces the same output for a known input.
  The output of a nondeterministic system will have random variations.

- Deadline: A deadline is the finite window of time in which a certain task must be completed.

- Quality of Service: The overall performance of a network.
  Includes factors such as bandwith, throughput, availability, jitter, latency, and error rates.

Real-time software guarantees correct computation at the correct time.

*Hard real-time* software systems have a set of strict deadlines, and missing a deadline is considered a system failure.
Examples of hard real-time systems: airplane sensor and autopilot systems, spacecrafts and planetary rovers.

*Soft real-time* systems try to reach deadlines but do not fail if a deadline is missed.
However, they may degrade their quality of service in such an event to improve responsiveness.
Examples of soft real-time systems: audio and video delivery software for entertainment (lag is undesirable but not catastrophic).

*Firm real-time systems* treat information delivered/computations made after a deadline as invalid.
Like soft real-time systems, they do not fail after a missed deadline, and they may degrade QoS if a deadline is missed (1).
Examples of firm real-time systems: financial forecast systems, robotic assembly lines (2).

Real-time computer systems are often associated with low-latency systems.
Many applications of real-time computing are also low-latency applications (for example, automated piloting systems must be reactive to sudden changes in the environment).
However, a real-time system is not defined by low latency, but by a deterministic schedule: it must be guaranteed that the system finishes a certain task by a certain time.
Therefore, it is important that the latency in the system be measurable and a maximum allowable latency for tasks be set.

A real-time computer system needs both an operating system that operates in real-time and user code that delivers deterministic execution.
Neither deterministic user code on a non-real-time operating system or nondeterministic code on a real-time operating system will result in real-time performance.

Some examples of real-time environments:

- The `RT_PREEMPT` Linux kernel patch, which modifies the Linux scheduler to be fully preemptible (3).

- Xenomai, a POSIX-compliant co-kernel (or hypervisor) that provides a real-time kernel cooperating with the Linux kernel.
  The Linux kernel is treated as the idle task of the real-time kernel's scheduler (the lowest priority task).

- RTAI, an alternative co-kernel solution.

- QNX Neutrino, a POSIX-compliant real-time operating system for mission-critical systems.

## Best Practices in Real-time Computing

In general, an operating system can guarantee that the tasks it handles for the developer, such as thread scheduling, are deterministic, but the OS may not guarantee that the developer's code will run in real-time.
Therefore, it is up to the developer to know what the determinstic guarantees of an existing system are, and what she must do to write hard real-time code on top of the OS.

In this section, various strategies for developing on top of a real-time OS are explored, since these strategies might be applicable to ROS 2.
The patterns focus on the use case of C/C++ development on Linux-based real-time OS's (such as `RT_PREEMPT`), but the general concepts are applicable to other platforms.
Most of the patterns focus on workarounds for blocking calls in the OS, since any operation that involves blocking for an indeterminate amount of time is nondeterministic.

It is a common pattern to section real-time code into three parts; a non real-time safe section at the beginning of a process that preallocates memory on the heap, starts threads, etc., a real-time safe section (often implemented as a loop), and a non-real-time safe "teardown" section that deallocates memory as necessary, etc.
The "real-time code path" refers to the middle section of the execution.

### Memory management

Proper memory management is critical for real-time performance.
In general, the programmer should avoid page faults in the real-time code path.
During a page fault, the CPU pauses all computation and loads the missing page from disk into RAM (or cache, or registers).
Loading data from disk is a slow and unpredictable operation.
However, page faults are necessary or else the computer will run out of memory.
The solution is to avoid pagefaults.

Dynamic memory allocation can cause poor real-time performance.
Calls to `malloc/new` and `free/delete` will probably result in pagefaults.
Additionally, the heap allocates and frees memory blocks in such a way that leads to memory fragmentation, which creates poor performance for reads and writes, since the OS may have to scan for an indeterminate amount of time for a free memory block.

#### Lock memory, prefault stack

```
if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
  perror("mlockall failed");
  exit(-2);
}
unsigned char dummy[MAX_SAFE_STACK];

memset(dummy, 0, MAX_SAFE_STACK);
```

[`mlockall`](http://linux.die.net/man/2/mlockall) is a Linux system call for locking the process's virtual address space into RAM, preventing the memory that will be accessed by the process from getting paged into swap space.

This code snippet, when run at the beginning of a thread's lifecycle, ensures that no pagefaults occur while the thread is running.
`mlockall` locks the stack for the thread.
The [`memset`](http://linux.die.net/man/3/memset) call pre-loads each block of memory of the stack into the cache, so that no pagefaults will occur when the stack is accessed (3).

#### Allocate dynamic memory pool

```
if (mlockall(MCL_CURRENT | MCL_FUTURE))
  perror("mlockall failed:");

/* Turn off malloc trimming.*/
mallopt(M_TRIM_THRESHOLD, -1);

/* Turn off mmap usage. */
mallopt(M_MMAP_MAX, 0);

page_size = sysconf(_SC_PAGESIZE);
buffer = malloc(SOMESIZE);

for (i=0; i < SOMESIZE; i+=page_size) {
  buffer[i] = 0;
}
free(buffer);
```

The intro to this section stated that dynamic memory allocation is usually not real-time safe.
However, this code snippet shows how to make dynamic memory allocation real-time safe (mostly).
It locks the virtual address space to a fixed size, disallows returning deallocated memory to the kernel via `sbrk`, and disables `mmap`.
This effectively locks a pool of memory in the heap into RAM, which prevents page faults due to `malloc` and `free` (3).

Pros:

- Can use malloc/new, free/delete, and even STL containers

Cons:

- Platform/implementation dependent
- Must accurately predict bounded memory size for the process!
- Using STL containers is therefore dangerous (unbounded sizes)
- In practice, only works for processes with small memory footprint

#### Custom real-time safe memory allocators

The default allocator on most operating systems is not optimized for real-time safety.
However, there is another strategy that is an exception to the "avoid dynamic memory allocation" rule.
Research into alternative dynamic memory allocators is a rich research topic (8).

One such alternative allocator is TLSF (Two-Level Segregate Fit).
It is also called the O(1) allocator, since the time cost of `malloc`, `free`, and `align` operations under TLSF have a constant upper bound.
It creates a low level of fragmentation.
The disadvantages of TLSF are that it is not thread safe and that its current implementation is architecture specific: it assumes the system can make 4-byte aligned accesses.

Pros:

- Can safely allocate memory in a program with memory bounds that are unknown at runtime or compile
- Advantages vary depending on the choice of allocator

Cons:

- Implementations of custom allocators may not be well tested, since they are less widely used
- Extra dependency, potentially more code complexity
- Drawbacks vary depending on the choice of allocator

#### Global variables and (static) arrays

Global variables are preallocated at the start of a process, thus assigning and accessing them is real-time safe.
However, this strategy comes with the many disadvantages of using global variables.

#### Cache friendliness for pointer and vtable accesses

Classes with many levels of inheritance may not be real-time safe because of vtable overhead access.
When executing an inherited function, the program needs to access the data used in the function, the vtable for the class, and the instructions for the function, which are all stored in different parts of memory, and may or may not be stored in cache together (5).

In general, C++ patterns with poor cache locality are not well-suited to real-time environments.
Another such pattern is the opaque pointer idiom (PIMPL), which is convenient for ABI compatibility and speeding up compile times.
However, bouncing between the memory location for the object and its private data pointer causes the cache to "spill" as it loads one chunk of memory and then another, unrelated chunk for almost every function in the PIMPLized object.

#### Exceptions

Handling exceptions can incur a large performance penalty.
Running into an exception tends to push a lot of memory onto the stack, which is often a limited resource in real-time programming.
But if exceptions are used properly, they should not be a concern to real-time programmers (since they indicate a place in the program with undefined behavior and are integral to debugging) (6).

#### Know your problem

Different programs have different memory needs, thus memory management strategies vary between applications.

- Required memory size known at compile time

  - Example: publishing a message of a fixed size.
  - Solution: use stack allocation with fixed-size objects.

- Required memory size known at runtime, before real-time execution.

  - Example: publishing a message of a size specified on the command line.
  - Preallocate variable size objects on the heap once required size is known, then execute real-time code.

- Required memory size computed during real-time

  - Example: a message received by the robot's sensors determines the size of the messages it publishes.

  - Multiple solutions exist

    - [Object pools](https://en.wikipedia.org/wiki/Object_pool_pattern)
    - [TLSF O(1) memory allocation](http://www.gii.upv.es/tlsf/)
    - Use stack allocation and fail if allocated memory is exceeded

### Device I/O

Interacting with physical devices (disk I/O, printing to the screen, etc.) may introduce unacceptable latency in the real-time code path, since the process is often forced to wait on slow physical phenomena.
Additionally, many I/O calls such as `fopen` result in pagefaults.

Keep disk reads/writes at the beginning or end of the program, outside of the RT code path.

Spin up threads that are not scheduled in real-time to print output to the screen.

### Multithreaded Programming and Synchronization

Real-time computation requirements change the typical paradigm of multithreaded programming.
Program execution may not block asynchronously, and threads must be scheduled deterministically.
A real-time operating system will fulfill this scheduling requirement, but there are still pitfalls for the developer to fall into.
This section provides guidelines for avoiding these pitfalls.

#### Thread creation guidelines

Create threads at the start of the program.
This confines the nondeterministic overhead of thread allocation to a defined point in the process.

Create high priority (but not 99) threads with a FIFO, Round Robin, or Deadline scheduler (see POSIX [sched](http://man7.org/linux/man-pages/man7/sched.7.html) API).

#### Avoid priority inversion

Priority inversion can occur on a system with a preemptive task scheduler and results in deadlock.
It occurs when: a low-priority task acquires a lock and is then pre-empted by a medium-priority task, then a high-priority task acquires the lock held by the low-priority task.

The three tasks are stuck in a triangle: the high-priority task is blocked on the low-priority task, which is blocked on the medium-priority task because it was preempted by a task with a higher priority, and the medium-priority task is also blocked on a task with a higher priority.

Here are the some solutions to priority inversion:

- Don't use blocking synchronization primitives
- Disable preemption for tasks holding locks (can lead to jitter)
- Increase priority of task holding a lock
- Use priority inheritance: a task that owns a lock inherits the priority of a task that tries to acquire the lock
- Use lock-free data structures and [algorithms](http://www.1024cores.net/home/lock-free-algorithms)

#### Timing shots

One real-time synchronization technique is when a thread calculates its next "shot" (the start of its next execution period).
For example, if a thread is required to provide an update every 10 milliseconds, and it must complete an operation that takes 3-6 milliseconds, the thread should get the time before the operation, do the operation, and then wait for the remaining 7-4 milliseconds, based on the time measured after the operation.

The most important consideration for the developer is to use a high precision timer, such as `nanosleep` on Linux platforms, while waiting.
Otherwise the system will experience drift.

#### Spinlocks

Spinlocks tend to cause clock drift.
The developer should avoid implementing his own spinlocks.
The RT Preempt patch replaces much of the kernel's spinlocks with mutexes, but this might not be guaranteed on all platforms.

#### Avoid fork

[`fork`](http://linux.die.net/man/3/memset) is not real-time safe because it is implemented using [copy-on-write](https://en.wikipedia.org/wiki/Copy-on-write).
This means that when a forked process modifies a page of memory, it gets its own copy of that page.
This leads to page faults!

Page faults should be avoided in real-time programming, so Linux `fork`, as well as programs that call `fork`, should be avoided.

## Testing and Performance Benchmarking

### cyclictest

[`cyclictest`](http://manpages.ubuntu.com/manpages/trusty/man8/cyclictest.8.html) is a simple Linux command line tool for measuring the jitter of a real-time environment.
It takes as input a number of threads, a priority for the threads, and a scheduler type.
It spins up `n` threads that sleep regular intervals (the sleep period can also be specified from the command line) (7).

For each thread, `cyclictest` measures the time between when the thread is supposed to wake up and when it actually wakes up.
The variability of this latency is the scheduling jitter in the system.
If there are processes with non-deterministic blocking behavior running in the system, the average latency will grow to a large number (on the order of milliseconds), since the scheduler cannot meet the deadlines of the periodically sleeping threads profiled in the program.

### Instrumenting code for testing

A more precise way to measure the scheduling jitter in a program is to instrument the periodic real-time update loop of existing code to record scheduling jitter.

A proposed header for a minimal library for real-time code instrumentation can be found here: [rttest.h](https://github.com/jacquelinekay/rttest/blob/master/include/rttest/rttest.h).

### Pagefaults

The Linux system call [`getrusage`](http://linux.die.net/man/2/getrusage) returns statistics about many resource usage events relevant to real-time performance, such as minor and major pagefaults, swaps, and block I/O.
It can retrieve these statistics for an entire process or for one thread.
Thus it is simple to instrument code to check for these events.
In particular, `getrusage` should be called right before and right after the real-time section of the code, and these results should be compared, since `getrusage` collects statistics about the entire duration of the thread/process.

Collecting these statistics gives an indication of what events could cause the latency/scheduling jitter measured by the previously described methods.

## Design Guidelines for ROS 2

## Implementation strategy

With judicious application of the performance patterns and benchmarking tests proposed in this document, implementing real-time code in C/C++ is feasible.
The question of how ROS 2 will achieve real-time compatibility remains.

It is acceptable for the setup and teardown stages of the ROS node lifecycle to not be real-time safe.
However, interacting with ROS interfaces, particularly within an intra-process context, should be real-time safe, since these actions could be on the real-time code path of a process.

There are a few possible strategies for the real-time "hardening" of existing and future ROS 2 code:

- Create a configuration option for the stack to operate in "real-time friendly" mode.

  - Pros:

    - Could allow user to dynamically switch between real-time and non-real-time modes.

  - Cons:

    - Refactoring overhead.
      Integrating real-time code with existing code may be intractable.

- Implement a new real-time stack (rclrt, rmwrt, etc.) designed with real-time computing in mind.

  - Pros:

    - Easier to design and maintain.

    - Real-time code is "quarantined" from existing code.
      Can fully optimize library for real-time application.

  - Cons:

    - More packages to write and maintain.
    - Potentially less convenient for the user.

- Give the option for real-time safety up to a certain point in the stack, and implement a real-time safe language wrapper (rclrt or rclc)

  - Pros:

    - Existing code is designed for this refactoring to be fairly easy
    - User can provide memory allocation strategy to rcl/rmw to ensure deterministic operation
    - Synchronization happens at the top the language/OS-specific layer, so refactoring rcl/rmw is easier
    - May be easier to support multiple embedded platforms with different wrappers

  - Cons:

    - More code to test for real-time safety
    - More flexibility for user-developer may mean more complexity

The third option is most appealing because it represents the least amount of work for the most number of benefits.

## Sources

1. Stefan Petters, [Presentation on Real-Time Systems](http://www.cse.unsw.edu.au/~cs9242/08/lectures/09-realtimex2.pdf)
2. [Differences between hard real-time, soft real-time, and firm real-time](http://stackoverflow.com/questions/17308956/differences-between-hard-real-time-soft-real-time-and-firm-real-time), Stack Overflow
3. [Real-Time Linux Wiki](https://rt.wiki.kernel.org/)
4. [Real-time operating system, Wikipedia](https://en.wikipedia.org/wiki/Real-time_operating_system#Memory_allocation)
5. Scott Salmon, [How to make C++ more real-time friendly](http://www.embedded.com/design/programming-languages-and-tools/4429790/2/How-to-make-C--more-real-time-friendly)
6. Stack Overflow, [Are Exceptions still undesirable in Realtime environment?](http://stackoverflow.com/questions/5257190/are-exceptions-still-undesirable-in-realtime-environment)
7. Pavel Moryc, [Task jitter measurement under RTLinux operating system](https://fedcsis.org/proceedings/2007/pliks/48.pdf)
8. Alfons Crespo, Ismael Ripoll and Miguel Masmano, [Dynamic Memory Management for Embedded Real-Time Systems](http://www.gii.upv.es/tlsf/files/papers/tlsf_slides.pdf)
9. Miguel Masmno, [TLSF: A New Dynamic Memory Allocator for Real-Time Systems](http://www.gii.upv.es/tlsf/files/ecrts04_tlsf.pdf)
