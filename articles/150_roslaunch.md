---
layout: default
title: ROS 2 Launch System
permalink: articles/roslaunch.html
abstract:
  The launch system in ROS is responsible for helping the user describe the configuration of their system and then execute it as described. The configuration of the system includes what programs to run, what arguments to pass them, and ROS specific conventions which make it easy to reuse components throughout the system through configuration. Also, because the launch system is the entity that executes processes, it is also responsible for monitoring the state of processes in the system, then reporting and/or reacting to changes in the state of those processes.
author: '[William Woodall](https://github.com/wjwwood)'
published: true
---

- This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Context

<div class="alert alert-warning" markdown="1">
TODO: Fill out some context with respect to ROS 1's `roslaunch` and other similar tools which exist in OS's.
</div>

## Separation of Concern

The launch system can be considered in parts, separated by concern.
The coarse breakdown is like so:

- Calling Conventions for Processes and Various Styles of Nodes
- System Description and Static Analysis
- Execution and Verification of the System Description
- Reporting System for Event
- Testing

The purpose of the following sections is to enumerate what the launch system could interact with or do, but is not the requirements list for the launch system in ROS 2.
The requirements for the launch system will be enumerated in section below based on what's possible in these sections.

## Calling Conventions

In order for the launch system to execute a described system, it needs to understand how it can achieve the description.
This is an existing phrase in Computer Science[^calling_convention_wikipedia], but this section is not talking specifically about the compiler defined calling convention, through it is appropriating the term to describe a similar relationship.
In this case, the phrase "calling conventions" is meant to describe the "interface" or "contract" the launch system has with the entities it is executing and monitoring.
This contract covers initial execution, activity during runtime, signal handling and behavior of the launch system, and shutdown.

### Operating System Processes

The most basic version of these entities, and the foundation for the other entities, are operating system processes.

#### Execution

For these, the launch system needs to know how to execute them, and to do that it needs:

- name of the executable (just the name, relative path, or absolute path)
- environment variables (`PATH`, `LD_LIBRARY_PATH`, `DL_PRELOAD`, etc...)
- command line arguments
- working directory (directory from which to execute the process)
- launch prefix (used to inject things like `gdb`, `valgrind`, etc...)

<div class="alert alert-warning" markdown="1">
RFC:

Missing from this list is the user which should be used to execute the process.
It's possible that it would be necessary or at least useful to change the user based on the launch description.
However, it can always be done in a user written script and supporting it in our Python implementation in a portable way looks to be difficult.
</div>

With this information the launch system can execute any arbitrary operating system process on the local machine.

#### Runtime

During runtime, the launch system may monitor all operating system process's:

- `stdout` pipe
- `stderr` pipe

The launch system may choose to either capture these pipes, for logging or suppressing output to the console, or it can connect the pipes to an existing `pty`, like the terminal's `stdout` and/or `stderr` pipes or a null pipe (e.g. `/dev/null`).

When capturing the output pipes of a process, the launch system could report this data in a way that the user may process them in real-time or could pass the data through user defined filters, generating a user-handled event when the filter matches.

In addition, the launch system may interact with, or allow the user to interact with, an operating system process's:

- `stdin` pipe
- signals (`SIGINT`, `SIGTERM`, `SIGUSR1`, etc...)

Regardless of how the user uses the launch system to interact with these items, they should be exposed by the launch system, which is the only entity which can interact with them directly.

#### Termination

If the operating system process terminates, and therefore returns a return code, the launch system will report this event and it can be handled in a user defined way.
Termination covers expected termination (e.g. return from `main()` or use `exit()`) and unexpected termination (e.g. the abort trap or a segmentation fault or bus error).

Historically, ROS 1's `roslaunch` allowed a few common exit handling cases:

- `require=true`: if this process exits (any reason) shutdown everything else, as it's "required"
- `respawn=true`: if this process exits (any reason) restart it with the same settings as startup
  - `respawn_delay=N`: if restarting it, delay a number of seconds between attempts

The launch system may initiate the termination of an operating system process.
This starts with the signaling of `SIGINT` on the child process.
If this does not result in the termination of the process, then one of a few things can happen based on the configuration of the launch system:

- after a period of time, signal `SIGTERM`
- after a period of time, signal `SIGKILL`
- nothing

By default, the launch system will:

- send `SIGINT`
- after 10 seconds, send `SIGTERM`
- after 10 additional seconds, send `SIGKILL`

The latter two steps can be skipped, or the time until escalation can be adjusted, on a per process basis.

The launch system will initiate this process when an event (built-in or user generated) initiates shutdown, e.g. when a process with the equivalent of the `require=true` exit handler terminates, or when the launch system itself receives the `SIGINT` signal.

If the launch system itself receives the `SIGTERM` signal it will send the `SIGKILL` signal to all child processes and exit immediately.

<div class="alert alert-warning" markdown="1">
RFC:

There are several small decisions here that I made somewhat arbitrarily, e.g. the default time until escalation and the propagation of `SIGKILL` when the launch system receives `SIGTERM`.
</div>

#### Shell Evaluation

A special case of operating system processes, shell evaluation would simply be passing shell script code as an argument to the default system shell.
This is a problematic thing to support because it is hard/messy to make it portable to all operating systems.

An in between entity is an operating system process which uses shell evaluation to expand a relative executable name to an absolute path using the PATH environment variable.

#### Remote Operating System Processes

<div class="alert alert-warning" markdown="1">
TODO: Describe this as a "mix-in" which can convert anything based on an "operating system process" into a remote one by adding ssh/user/setup/etc... information needed to do so, see how ROS 1's roslaunch does it.
</div>

### ROS Nodes

Any operating system process can become ROS specific by having at least one ROS Node within it.
Having one or more "plain" ROS nodes in a process doesn't add new ways to get information into or out of the operating system process that contains them.
It does however, add some specific kinds of inputs during execution and it also can affect how the process reacts to signals.

#### Execution

In addition to the "Execution" subsection of the "Operating System Processes" section, processes with ROS Nodes in them may take need to consider additional elements, like:

- Package name + executable name rather than executable name + PATH (i.e. `ros2 run` equivalent)
- ROS specific environment variables (e.g. `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, console output formatting, etc...)
- ROS specific command line arguments
  - Varies for single Node processes and multi-Node processes
  - Change node name or namespace
  - Remap topics, services, actions, parameters, etc...
  - Initialize parameter values

The specific syntax of these extra environment variables and command line arguments are defined in other documents[^logging_wiki] [^static_remapping].

In each of these cases, they simply are ROS specific ways to extend or add to the existing mechanism described by the equivalent section for "operating system processes", i.e. there's no new ROS specific way to pass more information into the initial execution of the process.
Here is also the first opportunity for the launch system to take ROS specific declarations, e.g. "remap 'image' to 'left/image'", and convert them implicitly into terms that a normal operating system process can consume like environment variables or command line arguments, e.g. adding `image:=left/image` to the command line arguments.

#### Runtime

During runtime a "plain" ROS node doesn't expose beyond what an operating system process does.

It also does not react to any additional signals, but processes containing ROS nodes do tend to have a signal handler for `SIGINT` which does a more graceful shutdown, but that is not enforced.
Sending the `SIGINT` signal typically causes most nodes to shutdown if they are monitoring `rclcpp::ok()` as recommended.

#### Termination

Termination of a ROS Node (the node, not the process) is not externally observable beyond what is observed with an operating system process (the return code).

### Managed ROS Nodes

For ROS nodes that have a lifecylce, a.k.a. Managed ROS Nodes[^lifecycle], each node will have additional runtime state, which the launch system could access and either utilize, pass through to the event system, or aggregate before passing it through the event system.

Building yet again on previous entities, the "Managed ROS Nodes" inherits all of the execution, runtime, and termination characteristics from normal ROS nodes and therefore operating system processes where applicable.

#### Execution

Managed ROS Nodes do not add any additional inputs or specific configurations at execution time on top of what "plain" ROS nodes add, at least not at this time.
In the future this might change, so reference the design doc[^lifecylce] or future documentation on the subject.

#### Runtime

During runtime, a Managed ROS node emits events anytime the state of the node changes.
This is at least emitted on a topic, but could also be captured, aggregated, and/or communicated in other ways too.

#### Termination

Again, Managed ROS Nodes have some additional observable effects when terminating (again the node, not necessarily the process containing it).
A managed node enters the `Finalized` state after passing through the `ShuttingDown` transition state on termination.
Since these are state transitions, they are observable via the lifecylce event system, at least through the ROS topic `lifecycle_state` (subject to change, always reference the managed nodes design document[^lifecylce]).

### Process with a Single Node

It extends the idea of 
This was likely the most commonly used type of entity launched in ROS 1, as you could only have one node per process in ROS 1.
In ROS 2, this will likely be less common, but will still be used quite a bit in the form of quickly developed scripts and drivers or GUI tools which might require control over the main thread.

Additional 

### Process with Multiple Nodes

### Dynamically loaded Nodes

#### By Configuration

#### By Proxy

## System Description

<div class="alert alert-warning" markdown="1">
TODO: Restructure notes on this and put them here.

Temporary summary:

This is the section where the Python API in `ros2/launch` and the XML file format from ROS 1 overlap.
Basically how do you describe the system in a way that is flexible, but also verifiable and ideally can also be statically analyzed (at least for something like the XML format).

I have compare and contrast like notes for other systems like upstart, systemd, and launchd. 
</div>

## Event System

<div class="alert alert-warning" markdown="1">
TODO: Restructure notes on this and put them here.

Temporary summary:

The launch system is responsible for executing processes (according to calling convention below) and exposing information that only it, as the parent process, has access to via events, e.g. things like stdout/stderr, return code.
The "reporting responsibility" could also extend to ROS specific things in the form of aggregation, i.e. allow the user to say "do X when nodes A, B, and C all reach the inactive state", but this could also be done on top of roslaunch.
This event system would also be reused (in combination with other sources of information) by the launch system to implement things like verification.
</div>

## Requirements

<div class="alert alert-warning" markdown="1">
TODO: Reformat requirements list, possibly combine/reconcile with "separation of concerns section"
</div>

## Reference Implementation Proposal

<div class="alert alert-warning" markdown="1">
TODO: This will outline what we have and what we need to build and how it should be separated.
</div>

## Alternatives

<div class="alert alert-warning" markdown="1">
TODO: Anything we choose not to support in the requirements vs. the "separation of concern section", and also any alternatives which we considered but rejected in the reference implementation proposal.
</div>

## References

[^calling_convention_wikipedia]: [https://en.wikipedia.org/wiki/Calling_convention](https://en.wikipedia.org/wiki/Calling_convention)
[^logging_wiki]: [https://github.com/ros2/ros2/wiki/Logging#console-output-configuration](https://github.com/ros2/ros2/wiki/Logging#console-output-configuration)
[^static_remapping]: [http://design.ros2.org/articles/static_remapping.html#remapping-rule-syntax](http://design.ros2.org/articles/static_remapping.html#remapping-rule-syntax)
[^lifecycle]: [http://design.ros2.org/articles/node_lifecycle.html](http://design.ros2.org/articles/node_lifecycle.html)
