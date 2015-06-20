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
Real-time 

(example)

(how to verify)

# Implementation of Real-time Computing

## RTLinux

## RTAI

## 

## Memory management


Strategies:

Lock memory, prefault stack:

```c
if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        perror("mlockall failed");
        exit(-2);
}
unsigned char dummy[MAX_SAFE_STACK];

memset(dummy, 0, MAX_SAFE_STACK);
```

Allocate pool for dynamic memory allocation:


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
    // Each write to this buffer will *not* generate a pagefault.
    // Even if nothing has been written to the newly allocated memory, the physical page
    //  is still provisioned to the process because mlockall() has been called with
    //  the MCL_FUTURE flag
    buffer[i] = 0;
    // print the number of major and minor pagefaults this application has triggered
    getrusage(RUSAGE_SELF, &usage);
    printf("Major-pagefaults:%d, Minor Pagefaults:%d\n", usage.ru_majflt, usage.ru_minflt);
}
free(buffer);
```

Pros:
  Can use `malloc/new`, `free/delete` and even STL containers

Cons:
  Must accurately predict bounded memory size for the process!
  Using STL containers is therefore dangerous (unbounded size)
  Only works for small processes

Global variables and (static) arrays

Global variables are preallocated at the start of a process, thus assigning and accessing them
is "real-time" safe. However, this strategy comes with the many disadvantages of using global
variables.

Avoiding excessive inheritance and PIMPL

Classes with many levels of inheritance create nondeterministic performance because 

Exceptions

## Device I/O
Interacting with physical devices (disk I/O, printing to the screen, etc.) introduces
unacceptable latency in the real-time code path.

Strategies:

Keep disk reads/writes at the beginning or end of the program, outside of the RT code path.

Spin up userspace threads (executing on a different CPU from the RT code) to print output to
the screen.

## 

# Testing and Benchmarking

# Design Guidelines for ROS 2

We must consider the target specifications for ROS 2 running in realtime. Real-time operating
systems and frameworks are diverse, and it is improbable that ROS 2 can target all use cases.
The best we can do is identify the real-time performance characteristics for the core ROS 2
stack, such as latency, jitter, and drift, and drive them to numbers that are acceptable for
developers 

## Real-time code path in ROS 2

## Implementations of RCL and RMW

There are two strategies for 

