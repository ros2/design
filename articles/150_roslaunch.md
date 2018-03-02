---
layout: default
title: ROS 2 Launch System
permalink: articles/roslaunch.html
abstract:
  The launch system in ROS is responsible for helping the user describe the configuration of their system and then execute it as described. The configuration of the system includes what programs to run, what arguments to pass them, and ROS specific conventions which make it easy to reuse components throughout the system by giving them each different configurations. Also, because the launch system is the process (or the set of processes) which executes the user's processes, it is responsible for monitoring the state of the processes it launched, as well as reporting and/or reacting to changes in the state of those processes.
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

This article describes the launch system for ROS 2, and as the successor to the launch system in ROS 1 it makes sense to summarize the features and roles of `roslaunch` from ROS 1 and compare them to the goals of the launch system for ROS 2.

### Description of `roslaunch` from ROS 1

From the description of `roslaunch` from the wiki ([https://wiki.ros.org/roslaunch](https://wiki.ros.org/roslaunch)):

> roslaunch is a tool for easily launching multiple ROS nodes locally and remotely via SSH, as well as setting parameters on the Parameter Server. It includes options to automatically respawn processes that have already died. roslaunch takes in one or more XML configuration files (with the .launch extension) that specify the parameters to set and nodes to launch, as well as the machines that they should be run on.

This description lays out the main roles of `roslaunch` from ROS 1 as:

- launch nodes
- launching nodes remotely via SSH
- setting parameters on the parameter server
- automatic respawning of processes that die
- static, XML based description of the nodes to launch, parameters to set, and where to run them

Further more the wiki goes on to say ([https://wiki.ros.org/roslaunch/Architecture](https://wiki.ros.org/roslaunch/Architecture)):

> roslaunch was designed to fit the ROS architecture of complexity via composition: write a simple system first, then combine it with other simple systems to make more complex systems. In roslaunch, this is expressed through several mechanisms:

> 1. `<include>`s: you can easily include other .launch files and also assign them a namespace so that their names do not confict with yours.

> 2. `<group>`s: you can group together a collection of nodes to give them the same name remappings.

> 3. aliased `<machine>`s: you can separate machine definitions and node definitions into separate .launch files and use aliases to define which machines get used at runtime. This allows you to reuse the same node definitions for multiple robots. For example, instead of saying that a laser_assembler runs on 'foo.willowgarage.com', you can say that it runs on the 'tilt-laser' machine. The machine definitions then take care of which host is the 'tilt-laser' machine.

> roslaunch also contains a variety of tools to help you write your .launch files as portably as possible. You can use the `<env>` tag to specify environment variables that need to be set for a particular machine or node. The $(find pkg) syntax let you specify file paths relative to a ROS package, instead of specifying their location on a particular machine. You can also use the $(env ENVIRONMENT_VARIABLE) syntax within include tags to load in .launch files based on environment variables (e.g. MACHINE_NAME).

From this, there are a few more design goals and roles for `roslaunch` from ROS 1:

- composition of systems into systems of systems to manage complexity
- use include semantic to reuse fragments rather than writing each from scratch
- use groups to apply settings (e.g. remappings) to collections of nodes/processes/included launch files
  - also use groups with namespaces to form hierarchies
- portability through abstraction of operating system concepts, e.g. environment variables
- utilities to locate files on the filesystem in a relocatable and portable way, e.g. `$(find <package_name>)`

That covers most of the features and design goals of `roslaunch` from ROS 1, but in the next subsection we'll discuss what is different for the launch system in ROS 2 due to changes in ROS 2 and how it might improve on the launch system from ROS 1.

### Differences in ROS 2

One of the objectives of the launch system in ROS 2 is to emulate the features of the launch system in ROS 1, but due to architectural changes in ROS 2, some of the features, goals, and terminology need to change.

#### Relationship Between Nodes and Processes

In ROS 1, there could only ever be one node per process and so the goals of `roslaunch` from ROS 1 reflect that by using "ROS nodes" and "processes" almost interchangeably.

Even for the ROS 1 feature called 'nodelet' (where you could emulate having more than one node per process), the conceptual mapping from node or nodelet to process was preserved by proxy processes.
For example, you would run a "NodeletManager" and then run a process for each nodelet you wanted to run in that manager.
This allowed nodelet's which exited to be detected by `roslaunch` from ROS 1, as well as allowing them to respond to signals that it sent to the proxy process.

Since you can have many nodes per process in ROS 2, it is no longer necessary to conflate nodes and processes.
Due to this, the design and documentation for the launch system in ROS 2 will need to be clearer when talking about processes and nodes.
Additionally, the way that configuration (e.g. parameters and remappings) get passed to nodes by the launch system needs to be adapted, though this part overlaps with the design documents for static remapping[^static_remapping] and for parameters[^parameters].

Also, since there can be multiple nodes per process, shutting down a node no longer always means sending a unix signal to a single process.
Other mechanisms might need to be used to have more granular shutdown control in multi-node processes.

#### Launching Nodes (Processes) Remotely and Portability

The launch system in ROS 1 only really ever was supported on Linux and other Unix-like operating systems like BSD and macOS.
These machines all have SSH, which is the mechanism which is specifically called out to be used when launching processes on remote machines.
It also played a role in defining what you specified and how when configuring `roslaunch` from ROS 1 to be able to launch processes on remote machines.

In ROS 2, Windows has been added to the list of targeted platforms, and as of the writing of this document it does not support SSH natively.
So unless that changes (more possible than it sounds), a different, more portable mechanism might be required to support this feature everywhere.
At the very least, an alternative solution would need to be used on Windows even if SSH was still used on Unix-like operating systems.

#### Parameters

In ROS 1, there was a global parameter server which stored all parameters and nodes would get and set all parameters through this server.
The server was tightly integrated into `roslaunch` from ROS 1, and was also used by the other kind of parameters from ROS 1, which were called "dynamic reconfigure parameters".

In ROS 2, there are only one kind of parameters and they work differently.
In general they work more like "dynamic reconfigure parameters" from ROS 1, in that they are node specific (no truly global parameters) and they are managed by the node (the node can refuse changes and parameters can only be read and changed while the node is running).
More details can be found in the parameters design document[^parameters].

There can (and probably will) still be a "global parameter server" in ROS 2, but it will simply be implemented as a node which accepts all changes and could be run along with the launch system automatically or could be invoked explicitly by the user (a la `roscore` from ROS 1), but it should not be required for basic functionality.

This fundamental difference in how parameters work will affect both the architecture of the launch system in ROS 2 and how users specify parameters for nodes via the launch system.

#### Process Related Events and Responses

In `roslaunch` from ROS 1 there were only a few ways that it could react to changes in the system, and they were both related to a process "dieing" (either a clean or unclean exit):

- respawn a process if it died
- shutdown the whole launch system if a required process died

This is somewhere that the launch system in ROS 2 can hopefully improve on what `roslaunch` from ROS 1 had to offer, and it can do so by providing not only these common reactions to processes exiting, but also by providing more granular information about the process exit (and other events), and by letting the user specify arbitrary responses to these type of events.

<div class="alert alert-warning" markdown="1">
RFC:

This is very much still a point for debate in my opinion.
I think there is a valid argument against arbitrary event handling and restricting it to "canned" responses to a limited set of events, and leaving everything else to external programs.
I especially feel this way about changes in node state and nodes exiting even when their process does not, but I don't feel as strongly when it comes to reporting and reacting to events related to the process itself, e.g. the return code, stdout/stderr/stdin, etc...
Mostly because no other process can observe these things, so they should definitely be exported in some way by the launch system, and if you're already doing that then it might just be much more convenient to let the user tell the launch system what to do rather than having to have a separate process that monitors what the launch system is reporting and then has to react, mostly likely by asking the launch system to do something else.
</div>

#### Deterministic Startup

In the ROS 1 wiki for `rosluanch`, it says ([https://wiki.ros.org/roslaunch/Architecture](https://wiki.ros.org/roslaunch/Architecture)):

> roslaunch does not guarantee any particular order to the startup of nodes -- although this is a frequently requested feature, it is not one that has any particular meaning in the ROS architecture as there is no way to tell when a node is initialized.

Hopefully this is another case on which the launch system for ROS 2 can improve, at least for nodes with a lifecycle, a.k.a. Managed Nodes[^lifecycle].
For Managed Nodes, it would not be possible to apply constraints on when something is launched, rather than how it is in `roslaunch` from ROS 1, where things are run in a non-deterministic order.

In order to do this, the launch system in ROS 2 will need to model the dependencies between processes and/or nodes where they exist, and the constraints on those dependencies.
For example, a user might express that an image processing node has a dependency on a camera driver node with the constraint that it should not be launched (what ever the action to do that might be, e.g. run a process or something else) until the camera driver node reaches the "Active" state.
These constraints can be arbitrarily defined by the user or common constraints could be modeled directly by the launch system.

Also, these constraints don't have to be related to 
For example, a user might express that plain process should be launch (in this case executed as a subprocess) after another process has been running for ten seconds.
The launch system in ROS 2, could either choose to let the user define a predicate which satisfied that constraint, or it could provide a generic constraint like: "launch N seconds after another process".

<div class="alert alert-warning" markdown="1">
RFC:

This is also very much still a point for debate in my opinion.
I'm not sure if this kind of arbitrary constraint is a good idea, either because it doesn't provide enough value given the complexity, or because it allows users to any manner of dangerous or unexpected things in the predicate.
</div>

#### Node Related Events and Responses

Also leveraging Managed Nodes when possible, the launch system in ROS 2 could export, aggregate and export, or react to lifecycle events of nodes.
For example, it might be possible to say that a node, rather than a process, is "required" such that the launch system shutdowns if that node's state ends up in the "Finalized" state, which would be similar to a process exiting with the "required=true" setting for `roslaunch` from ROS 1.

<div class="alert alert-warning" markdown="1">
RFC:

There is still a gray area for me here as to where to drawn the line between the launch system and general purpose "supervision" of managed nodes (we've called this the "lifecycle manager" in the past).
It seems ok for the launch system to use the manage node's state information for deterministic startup and for the above kind of "require node" feature, but it's hard for me to say exactly to what degree the launch system should be required or tied into the long running supervision of system of managed ros nodes.
As opposed to what I always imagined which was a completely user written program which would monitor the lifecycle event system (and possibly other things like devices, the diagnostics system, etc...), would have domain and application specific knowledge, as well as knowledge about the ROS graph layout, and could react to certain events.

I didn't really think that would be implemented as a series of event handlers within the launch system, and I'm still not convinced that's a good idea.
On the other hand, I also have a hard time thinking that simple events which result in an action only the launch system can take, things like "respawn=true" and "required=true" for nodes, should be a separate program rather than just a user defined reaction within the launch system itself.
So I don't know where the line is, or if we should try to define it.
We could choose to support reacting to lifecycle events in both the launch system and externally, or we could choose to not utilize them at all in the launch system, but my gut reaction at the moment is to allow them to be used in the launch system and also not try to restrict their complexity, simply because I don't see a good way to do that or how to decide what should and should not be allowed.

I'm very interested to see what others think of this question.
</div>

#### Static Description and Programmatic API

Most users of `roslaunch` from ROS 1 used it by defining a static XML description of what they wanted executed and which parameters they wanted to set.
There is an API for `roslaunch` in ROS 1, but in our experience few people use this interface.
We can only speculate as to why, but the API is not very well documented and is not prevalent in the tutorials and examples.
Sticking strictly to the XML description has caused two different approaches to dynamic behavior/configuration to become more popular:

- preprocessing with an XML preprocessor, like `xacro` or some other general purpose templating system
- more sophisticated expressions as XML tags in the `roslaunch` from ROS 1 syntax, e.g. `$(eval expression)` (added in ROS Kinetic) or the `if=$(arg ...)` and `unless=$(arg ...)` attributes

Often when these kind of "dynamic" features are discussed the question of "why is roslaunch (from ROS 1) a static description and not a script"?
The direct answer is that "it doesn't have to be", but the API for doing it programmatically is not very well documented or easy to use.

There are pro's and con's to both scripted launch files as well as static, declarative launch files, but that will be covered in its own section later in this article.
But even if the preference is for a static launch file format like is common in ROS 1, it's a goal of the launch system in ROS 2 to have a more accessible public API which is used to execute that static launch file, so a programmatic approach will always be an option.

#### Locating Files

It's often the case that you need to express the location of a file when describing your system to the launch system, whether it be an executable to run, a file to be passed as an argument, or a file from which to load parameters.
In the launch system for ROS 2, like the launch system for ROS 1 the concept of packages is used to group related resources and programs together to make this easier, but it will also support some other kinds of relative paths (other than just package share folders).
But where ROS 1 and ROS 2 differ in this topic is how the packages will be found, which folders a package can be associated with, and therefore probably also the syntax for how to get that relative path.

### Similarities with ROS 1

The previous subsection dealt with what may be different for the launch system in ROS 2, but in this subsection the similarities will be enumerated (not necessarily exhaustively).
The launch system in ROS 2 will:

- convert common ROS concepts like remapping and changing the namespace into appropriate command line arguments and configurations for nodes so the user doesn't have to do so
- manage complexity through composition of simpler systems (launch files)
- allow including of other launch files
- use groups to apply settings to collections of nodes and processes
- provide operating system portability where possible

and possibly other things, all of which it will share in common with `roslaunch` from ROS 1.

## Separation of Concern

The launch system can be considered in parts, separated by concern.
The coarse breakdown is like so:

- Calling Conventions for Processes and Various Styles of Nodes
- Reporting System for Events
- System Description and Static Analysis
- Execution and Verification of the System Description
- Testing

The purpose of the following sections is to enumerate what the launch system could do and the things with which it could interact, but is not the requirements list for the launch system in ROS 2.
The requirements for the launch system will be enumerated in section below based on what's possible in these sections.

## Calling Conventions

In order for the launch system to execute a described system, it needs to understand how it can achieve the description.
The phrase "calling conventions" is an existing phrase in Computer Science[^calling_convention_wikipedia], but this section is not talking specifically about the compiler defined calling convention, through it is appropriating the term to describe a similar relationship.
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

A kind of in-between entity is an operating system process which uses shell evaluation to expand a relative executable name to an absolute path using the PATH environment variable.

#### Remote Operating System Processes

Any of the entities based on an operating system process can be made into a remote operating system process by simply adding the requirement information needed to gain access to the other machine and execute it.
This is a feature that ROS 1's `roslaunch` has, and is useful in multi machine robots.

<div class="alert alert-warning" markdown="1">
TODO: figure out what we need to do here in terms of portability and configuration
</div>

### ROS Nodes

Any operating system process can become ROS specific by having at least one ROS Node within it.
Having one or more "plain" ROS nodes in a process doesn't add new standardized ways to get information into or out of the operating system process that contains them, though ROS topics, services, parameters, etc. can be accessed during runtime.
It does however, add some specific kinds of inputs during execution and it also can affect how the process reacts to signals.

This applies to "plain" ROS nodes, but there is more that the launch system can use in Managed ROS Nodes, which is described in the next section.

#### Execution

In addition to the "Execution" subsection of the "Operating System Processes" section, processes with ROS Nodes in them may need to consider additional elements, like:

- "Package name + executable name" rather than "executable name + PATH" (i.e. `ros2 run` equivalent)
- ROS specific environment variables (e.g. `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, console output formatting, etc...)
- ROS specific command line arguments
  - Varies for single Node processes and multi Node processes
  - Change node name or namespace
  - Remap topics, services, actions, parameters, etc...
  - Initialize parameter values

The specific syntax of these extra environment variables and command line arguments are defined in other documents[^logging_wiki] [^static_remapping].

In each of these cases, the ROS specific constructs can be expressed with the existing mechanisms described by the "Execution" subsection for "Operating System Processes", i.e. the ROS specific constructs can be expanded into either command line arguments or environment variables.
Therefore the launch system is able to take ROS specific declarations, e.g. "remap 'image' to 'left/image'", and convert them implicitly into terms that a normal operating system process can consume like environment variables or command line arguments, e.g. adding `image:=left/image` to the command line arguments.
However, what a given ROS specific declaration is converted into depends on how the nodes are used within the process, but later sections will go into details about that.

#### Runtime

During runtime a "plain" ROS node doesn't expose anything new beyond what an operating system process does.
It does have ROS topics, services, parameters, etc. but none that are standardized in a way that's useful for the launch system at this time.

It also does not react in any special way to `stdin`, but processes containing ROS nodes do tend to have a signal handler for `SIGINT` which does a more graceful shutdown, but that is not enforced.
Sending the `SIGINT` signal typically causes most nodes to shutdown if they are using one of the "spin" functions in `rclcpp` or are polling `rclcpp::ok()`, as is recommended.

#### Termination

Termination of a ROS Node (the node, not the process) is not externally observable beyond what is observed with an operating system process (the return code).

### Managed ROS Nodes

For ROS nodes that have a lifecylce, a.k.a. Managed ROS Nodes[^lifecycle], each node will have additional runtime state, which the launch system could access and either utilize directly, pass through to the event system, or aggregate before passing it through the event system.

Building yet again on previous entities, the "Managed ROS Nodes" inherits all of the execution, runtime, and termination characteristics from normal ROS nodes and therefore operating system processes.

#### Execution

Managed ROS Nodes do not add any additional inputs or specific configurations at execution time on top of what "plain" ROS nodes add, at least not at this time.
In the future this might change, so reference the design doc[^lifecylce] or future documentation on the subject.

#### Runtime

During runtime, a Managed ROS node emits events anytime the state of the node changes.
This is at least emitted on a topic, but could also be captured, aggregated, and/or communicated in other ways too.
These state changes could be consumed by either the launch system itself or by the user, either of which could react to these changes.

For example, the user could express something like "when node 'A' enters the `Active` state, launch nodes 'B' and 'C'" or "if node 'A' exits with a return code or enters the `Finalized` state, shutdown everything".

<div class="alert alert-warning" markdown="1">
RFC:

This is where things get murky for me.
There is a gray line between more complex startup behaviors (like "wait for A to get to Active then launch B") and long running monitoring and reactions to events, what I would call "supervision" (like "if A goes transitions to ErrorProcessing change B to Inactive").

I'm interested to see what other people think.

For me the best way to separate the two cases (if they're to be separated at all) is to ask "do I want/need to understand this event handling off-line"?
For example, when looking at a system description, it would be nice to see the event handler "when A crashes or exits, the whole system is down" (i.e. `require=true`) or the event handler "when A reaches Inactive, launch B" (something like "B depends on A").
However, maybe it doesn't need to capture/understand the "when A crashes or has an error move B to inactive".

Alternatively, we could say "all event handling is opaque to the launch system and is user defined" which removes the ambiguity.

\<wjwwod's opinion>
<br/>
I think the idea that saying all "supervision" or "reactionary launching" like activities are not part of the launch system is the simplest way forward, but then you'd still like to include in the launch system some convenient ways to express the most common cases, e.g. something like ROS 1's `roslaunch`'s `require=true` and `respawn=true`.
In my opinion, it would be unacceptably inconvenient to have users write a lambda for each of those cases, and I also think it would lead to everyone doing it slightly different and making it impossible to understand with tools.
<br/>
</wjwwod's opinion>

This will come up again in the "System Description" section, because I believe the mind set around the "Execution of the System Description" (basically the python library that runs things) will be "personal responsibility".
</div>

#### Termination

Managed ROS Nodes have some additional observable effects when terminating (the node, not necessarily the process containing it).
A managed node enters the `Finalized` state after passing through the `ShuttingDown` transition state on termination.
Since these are state transitions, they are observable via the lifecylce event system, at least through the ROS topic `lifecycle_state` (subject to change, always reference the managed nodes design document[^lifecylce]).

### Process with a Single Node

In this subsection, and the following subsections of the "Calling Conventions" section, the different possible combinations of nodes and processes is explained.
In each case they "inherit" any behaviors from either the "ROS nodes" or the "Managed ROS nodes" subsections above, but in these subsections the "how" of communicating ROS specific options is described in more detail.

The first is a single process with a single ROS node within it.
This was likely the most commonly used type of entity launched in ROS 1, as you could only have one node per process in ROS 1.
In ROS 2, this will likely be less common because you can have one to many nodes per process, but will may still be used quite a bit in the form of quickly developed scripts and drivers or GUI tools which might require control over the main thread.

Since there is only one ROS node, the command line arguments do not need to be explicit about to which node they apply.
For example, changing the namespace of the single node could be expressed with the command line argument `__ns:=new_namespace`.

Even though there is only one node in the process, that node does not need to start with the process starts, nor does the process need to end when the node is shutdown and/or destroyed.
If it is a managed node, the lifecycle of the node is best tracked using the lifecycle events.
In fact, a process with a single node could start a node, run for a while, later destroy it, and then create it again.

So the biggest impact of a single node process is that the configuration, in terms of command line arguments and environment variables, can be simplified.

### Process with Multiple Nodes

In a process with multiple nodes, things are much the same as with a process with a single node, but the configuration, again in terms of command line arguments and environment variables, need to be more specific in order to discriminate between the various nodes being instantiated in the process.
The remapping design document[^static_remapping] goes into detail on how you can selectively configure multiple nodes using command line arguments, so check there for up-to-date details.

However, as an example of a process with multiple nodes, consider a program that instantiates two camera driver nodes called "camera1" and "camera2" by default.
You could configure their namespaces separately by doing something like `camera1:__ns:=left camera2:__ns:=right`.

### Dynamically loaded Nodes

A special case of processes with multiple nodes are nodes that do not statically instantiate nodes.
Two separate ways to do this are described below, but in both cases the typical command line arguments (as described in the remapping design document[^static_remapping]) and environment variables may not work as expected.
In stead the method for causing the dynamic instantiation of a node needs to communicate these settings separately.
And because of this, the launch system also needs to know how to do this.

#### By Configuration

The first way would be to pass some user define command line arguments to the process which the process would interpret and use to dynamically instantiate some ROS nodes.
There's no current plan to have a generic tool for this, but that's subject to change.
However, since there's no standard tool, the launch system cannot know how to convert ROS specific declarations into a format that can be used to invoke it.

Though the user could always generate the appropriate command line arguments for the process and have the launch system treat it as a normal operating system process.
In that case, though, it might be useful to indicate to the launch system that a process may contain nodes, but should be otherwise treated like a plain operating system process.
This would let the launch system still monitor it for ROS specific events and input/output.

As an example of a "by configuration" dynamic node process, you could have a process that takes a variable number of arguments, where each is a serial port address for a laser scanner, e.g. `/dev/ACM0 /dev/ACM1 ...`, and it would instantiate a laser scanner driver node for each address.
It would would up to this process to figure out how to allow the user to further configure the nodes though.
It would not be possible, in this case, for the launch system to "apply a ROS namespace" to the nodes within it, for example.
For this reason, it's probably not an ideal way to structure nodes until a standardized way to load many nodes into a single process via configuration is defined.

<div class="alert alert-warning" markdown="1">
RFC:

On the point of a standardized way of having a "dynamic nodes process by configuration", I think it might be worth pursuing this, because it will basically become a more efficient way of launch a bunch of nodes into a single process, more efficient than the "by proxy" way described below.

The "by proxy" way below is essentially the pattern used by nodelets in ROS 1, but what I'm describing here is essentially like a NodeletManager executable which rather than loading nodelets via Service calls would load and run them based on command line arguments or a config file or something.

The "by proxy" way is more convenient when you're running things by hand, but if the roslaunch description looks something like this (just pseudo code):

```xml
<node_container_process name="my_container_process">
  <node package="my_package" executable="talker" />
  <node package="my_package" executable="listener" />

</node_container_process>
```

Whether or not the above results in three processes (one container and two proxy processes) or a single process, probably doesn't matter to the user or at least would be fine with the user in exchange for less overhead.

It would be more efficient because you don't need to have a node in each proxy to actually make the Service call to the load that node the user wants to run, and you don't need to maintain a bond between the container and the proxy processes.

So I'm interested in what others think about coming up with a standardized executable which can load and configure many nodes at once from command line arguments or a config file.
</div>

#### By Proxy

The other way to implement a process which dynamically instantiates ROS nodes, is to send the request to instantiate a node asynchronously from another process.
So the "container" process would start up and provide a service (ROS Service or otherwise) that lets external processes request a node be instantiated with a given set of configurations.

One form this can take is by providing a "proxy" process which doesn't actually run the node, but instead loads it into an already running container and just stays running until the node is shutdown in the remote container.
This pattern is used by the "nodelet's" in ROS 1.
It's a useful pattern when you're running nodes by hand but you want them to share a process and don't want to write your own program to do that.

There are a few other forms this can take, but the common thread between them is that a process instantiates nodes dynamically based on asynchronous input from external actors (proxy's), and that the configuration for those nodes is communicated through something other than command line arguments and environment variables.

## Event System

<div class="alert alert-warning" markdown="1">
TODO: Restructure notes on this and put them here.

Temporary summary:

The launch system is responsible for executing processes (according to calling convention below) and exposing information that only it, as the parent process, has access to via events, e.g. things like stdout/stderr, return code.
The "reporting responsibility" could also extend to ROS specific things in the form of aggregation, i.e. allow the user to say "do X when nodes A, B, and C all reach the inactive state", but this could also be done on top of roslaunch.
This event system would also be reused (in combination with other sources of information) by the launch system to implement things like verification.
</div>

## System Description

<div class="alert alert-warning" markdown="1">
TODO: Restructure notes on this and put them here.

Temporary summary:

This is the section where the Python API in `ros2/launch` and the XML file format from ROS 1 overlap.
Basically how do you describe the system in a way that is flexible, but also verifiable and ideally can also be statically analyzed (at least for something like the XML format).

I have compare and contrast like notes for other systems like upstart, systemd, and launchd.
</div>

## Execution and Verification of the System Description

<div class="alert alert-warning" markdown="1">
TODO: Restructure notes on this and put them here.

Temporary summary:

Whether described via static file or programmatically, once the system is described it has to be executed, and this section will cover all of that.
Most of this is already covered in the "calling conventions" section, but this section will also cover some more details about execution, and then add on to that verification (starting another discussion about what the launch system should and should not do itself).
Verification is runtime assertion that mirrors the static analysis that can be done off-line.
</div>

## Testing

<div class="alert alert-warning" markdown="1">
TODO: Restructure notes on this and put them here.

Temporary summary:

In ROS 1, `rostest` is an important extension of `roslaunch`, and so far in ROS 2 we're already using the foundation of launching (executing processes and reacting to their exit, return codes, and stdout/stderr), called `ros2/launch_testing` right now, to implement some tests.
This section will cover how that happens and how it integrates with the static description files as well as the programmatic API, adding ROS specific concepts to what we're already doing with `ros2/launch_testing`.
</div>

## Requirements

<div class="alert alert-warning" markdown="1">
TODO: Reformat requirements list, possibly combine/reconcile with "separation of concerns section" (consider dropping in favor of renaming to something that implies requirements as well)
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
[^parameters]: [http://design.ros2.org/articles/ros_parameters.html](http://design.ros2.org/articles/ros_parameters.html)
*[operating system process]: Operating System Process
*[operating system processes]: Operating System Processes
