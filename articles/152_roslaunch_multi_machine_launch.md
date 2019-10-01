---
layout: default
title: ROS 2 Multi-Machine Launching
permalink: articles/roslaunch_mml.html
abstract:
  Robotic systems are often distributed across multiple networked machines.
  This document describes proposed modifications and enhancements to ROS2's launch system to facilitate launching, monitoring, and shutting down systems spread across multiple machines.
author: '[Matt Lanting](https://github.com/mlanting)'
published: false
---

- This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Authors: {{ page.author }}

## Context

This document elaborates on the details of launching remote operating system processes alluded to [here](https://github.com/ros2/design/blob/gh-pages/articles/150_roslaunch.md#remote-operating-system-processes) in the main ROS 2 ros_launch design document.

## Goals

Our primary goal is to eliminate the need for users to connect to multiple machines and manually launch different components of a system on each of them independently.
The launch system in ROS 1 included a `<machine>` tag for launch files that allowed users to include information about networked machines and how to connect so that processes could be started remotely.
We would like to replicate that capability in the launch system for ROS 2.

We would like the launch system for ROS 2 to avoid becoming a single point of failure, while still having the capability to shut down the system as a whole on command.
In ROS 1, communication among nodes was facilitated by roscore which roslaunch would start automatically if no instance was already running.
As a result, the machine that roslaunch was run from became a core part of the system and the entire system would go down if it crashed or became disconnected.
This has been problematic on occasion when working with machines running headlessly and interfacing with a laptop.
The `launch` command either had to be run specifically on the computer that roscore was meant to run on, or other steps would need to be taken to launch `roscore` on a remote machine before running the `roslauch` command.
In ROS 2, nodes use DDS to connect in a peer-to-peer fashion with no centralized naming and registration services to have to start up.

Other issues that we've dealt with on multi-machine systems include ensuring all the machines are properly configured and set up and keeping files and packages synchronized and up to date across machines.
These issues, while related to working with multiple machines, are a bit outside the scope of roslaunch.
There are a number of third-part orchestration tools, such as Kubernetes, that could be leveraged to get some of this extra functionality in addition to using them to facilitate execution of nodes, but we felt that would be too large of a dependency to require of people.
Resource constraind projects in particular don't need to be burdened with additional third-party tools, and some hardware architectures do not have strong Docker support.
It might however make sense to consider including an optional API to facilitate such third-party tools, or at the very least be mindful of them so we can avoid doing anything to make integrating them later too much more difficult.

## Capabilities

In order to meet the above use goals, we will provide the following capabilities:

- Connect to a remote host and running nodes on it
- Support arbitrary remote execution or orchestration mechanisms (`ssh` by default)
- Push configuration parameters for nodes to remote hosts
- Monitor the status and managing the lifecycles of nodes across hosts
- Gracefully shut down nodes across hosts
- Command line tools for managing and monitoring systems across machines
- A grouping mechanism allowing collections of nodes to be stopped/introspected as a unit with the commandline tools

### Stretch-goals
- API to facilitate integration of third party orchestration tools such as Kubernetes or Ansible
- Load balancing nodes on distributed networks (Possibly outsource this capability to the previously mentioned third-party tools)
- Sharing and synchronizing files across machines
- Deployment and configuration of packages on remote machines

## Considerations

There are some outstanding issues that may complicate things:

- How to group nodes/participants/processes is somewhat of an open issue with potential implications for this part of ROS2.
    - https://github.com/ros2/design/pull/250/files/8ccaac3d60d7a0ded50934ba6416550f8d2af332?short_path=dd776c0#diff-dd776c070ecf252bc4dcc4b86a97c888
    - The number of domain participants is limited per vendor (Connext is 120 per domain).
- No `rosmaster` means there is no central mechanism for controlling modes or distributing parameters
- Machines may be running different operating systems
- If we intend to do any kind of load balancing, certain types of resources may need to be transferred to other machines.
     - Calibration data, map files, training data, etc.
     - Need to keep track of which machine has the most recent version of such resources
- Security: we'll need to manage credentials across numerous machines both for SSH and secure DDS.

## Proposed Approach

Following are some of the possible design approaches we have started considering.
This section should evolve to describe a complete and homogenous solution as we iterate over time, but at the moment may be a bit piecemeal as we explore ideas.
The point is to capture all of our ideas and approaches to different pieces of the problem, even rejected approaches, and to facilitate discussion and maintain a record of our reasoning.

### Simple Remote Process Execution

Create an action in `launch` called `ExecuteRemoteProcess` that extends the `ExecuteProcess` action but includes parameters for the information needed to connect to a remote host and executes the process there.

### Spawn Remote LaunchServers

The `LaunchServer` is the process that, given a `LaunchDescription`, visits all of the constituent `LaunchDescriptionEntities`, triggering them to perform their functions.
Since the launch process involves more than simply executing nodes, it is unlikely that simply providing a way to execute nodes remotely will be adequate for starting non-trivial systems.
The `LaunchServer` is responsible for things such as setting environment variables, registering listeners, emitting events,  filling out file and directory paths, declaring arguments, etc.
Remote machines will need to be made aware of any environment changes that are in-scope for nodes that they will be executing, and events may need to be handled across machines.

One approach would be to add logic to the launch system allowing it to group `LaunchDescriptionEntities` containing the necessary actions and substitutions for successfully executing a node remotely, spawning a LaunchService on the remote machine, serializing the group of entities and sending them to the remote machine to be processed.
This could turn out to be a recursive process depending on how a launch file creator has nested `LaunchDescriptionEntities` (which can themselves be `LaunchDescriptions`).
Additional logic will be needed to detect cases where event emission and listener registration cross machine boundaries, and helper objects can be generated to forward events over the wire so handlers on other machines can react appropriately.

### Define Remote Execution Mechanisms on a Per-Machine Basis

Historically, ROS1 launched nodes by using `ssh` to connect to a remote machine and execute processes
on it.  This is still a reasonable way of doing it and is the expected remote execution mechanism in most
environments.

Some hosts or environments may use a different mechanism, such as Windows Remote Shell on Windows hosts
or `kubectl` for Kubernetes clusters.  There will be an abstract interface for remote execution mechanisms;
it will be possible to write custom implementations that use arbitrary mechanisms, and the launch system
can be configured to decide which mechanism to use on a per-machine basis.  When a launch system is run,
information about all of the nodes assigned to a machine will be passed to the remote execution mechanism
implementation so that it can execute them appropriately.

## Proposed Multi-Machine Launch Command Line Interface

Launching is controlled through the `launch` command for the `ros2` command-line tool.

### Commands

```bash
$ ros2 launch
usage: ros2 launch (subcommand | [-h] [-d] [-D] [-p | -s] [-a]
                                 package_name [launch_file_name]
                                 [launch_arguments [launch_arguments ...]]) ...

Without a subcommand, `ros2 launch` will run a launch file.  Call
`ros2 launch <subcommand> -h` for more detailed usage.

positional arguments:
  package_name          Name of the ROS package which contains the launch file
  launch_file_name      Name of the launch file
  launch_arguments      Arguments to the launch file; '<name>:=<value>' (for
                        duplicates, last one wins)
  argv                  Pass arbitrary arguments to the launch file

optional arguments:
  -h, --help            Show this help message and exit.
  -d, --debug           Put the launch system in debug mode, provides more verbose output.
  -D, --detach          Detach from the launch process after it has started.
  -p, --print, --print-description
                        Print the launch description to the console without launching it.
  -s, --show-args, --show-arguments
                        Show arguments that may be given to the launch file.
  -a, --show-all-subprocesses-output
                        Show all launched subprocesses' output by overriding
                        their output configuration using the
                        OVERRIDE_LAUNCH_PROCESS_OUTPUT envvar.

Subcommands:
  list         Search for and list running launch systems
  attach       Attach to a running launch system and wait for it to finish
  term         Terminate a running launch system

  Call `ros2 launch <subcommand> -h` for more detailed usage.
```

Example output:

```bash
$ ros2 launch demo_nodes_cpp talker_listener.launch.py
[INFO] [launch]: All log files can be found below /home/preed/.ros/log/2019-09-11-20-54-30-715383-regulus-2799
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch]: Launch System ID is 50bda6fb-d451-4d53-8a2b-e8fcdce8170b
[INFO] [talker-1]: process started with pid [2809]
[INFO] [listener-2]: process started with pid [2810]
[talker-1] [INFO] [talker]: Publishing: 'Hello World: 1'
[listener-2] [INFO] [listener]: I heard: [Hello World: 1]
[talker-1] [INFO] [talker]: Publishing: 'Hello World: 2'
[listener-2] [INFO] [listener]: I heard: [Hello World: 2]
[talker-1] [INFO] [talker]: Publishing: 'Hello World: 3'
[listener-2] [INFO] [listener]: I heard: [Hello World: 3]
[talker-1] [INFO] [talker]: Publishing: 'Hello World: 4'
[listener-2] [INFO] [listener]: I heard: [Hello World: 4]
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[listener-2] [INFO] [rclcpp]: signal_handler(signal_value=2)
[INFO] [talker-1]: process has finished cleanly [pid 2809]
[INFO] [listener-2]: process has finished cleanly [pid 2810]
[talker-1] [INFO] [rclcpp]: signal_handler(signal_value=2)
```

Note how there is one difference from the old behavior of `ros2 launch`; the group of nodes is assigned a Launch System ID.
This is a unique identifier that can be used to track all of the nodes launched by a particular command across a network.

Additionally, it is possible to detach from a system and let it run in the background:

```bash
$ ros2 launch -D demo_nodes_cpp talker_listener.launch.py
[INFO] [launch]: All log files can be found below /home/preed/.ros/log/2019-09-11-20-54-30-715383-regulus-2799
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch]: Launch System ID is 50bda6fb-d451-4d53-8a2b-e8fcdce8170b
$
```

#### `list`

Since it is possible to launch a system of nodes that spans a network and detach from it, it is necessary to be able to query the network to find which systems are active.

```bash
$ ros2 launch list -h
usage: ros2 launch list [-h] [-v] [--spin-time SPIN_TIME]

List running launch systems


optional arguments:
  -h, --help            Show this help message and exit.
  -v, --verbose         Provides more verbose output.
  --spin-time SPIN_TIME
                        Spin time in seconds to wait for discovery (only
                        applies when not using an already running daemon)
```

Example output:

```bash
$ ros2 launch list
ab1e0138-bb22-4ec9-a590-cf377de42d0f
50bda6fb-d451-4d53-8a2b-e8fcdce8170b
5d186778-1f50-4828-9425-64cc2ed1342c
$
```

```bash
$ ros2 launch list -v
ab1e0138-bb22-4ec9-a590-cf377de42d0f: 5 nodes, 2 hosts
    Launch host: 192.168.10.5
    Launch time: Fri Sep 13 15:39:45 CDT 2019
    Launch command: ros2 launch package_foo bar.launch.py argument:=value
50bda6fb-d451-4d53-8a2b-e8fcdce8170b: 2 nodes, 1 host
    Launch host: 192.168.10.15
    Launch time: Fri Sep 13 12:39:45 CDT 2019
    Launch command: ros2 launch demo_nodes_cpp talker_listener.launch.py
5d186778-1f50-4828-9425-64cc2ed1342c: 16 nodes, 3 hosts
    Launch host: 192.168.10.13
    Launch time: Fri Sep 12 10:39:45 CDT 2019
    Launch command: ros2 launch package_foo bar2.launch.py
$
```

#### `attach`

Since it is possible to detach from a launched system, it is useful for scripting or diagnostic purposes to be able to re-attach to it.

```bash
$ ros2 launch attach -h
usage: ros2 launch attach [-h] [-v] [--spin-time SPIN_TIME] [system_id]

Blocks until all nodes running under the specified Launch System ID have exited

positional arguments:
  system_id          Launch System ID of the nodes to attach to; if less than a full UUID is specified, it will attach to the first Launch System it finds whose ID begins with that sub-string

optional arguments:
  -h, --help            Show this help message and exit.
  -v, --verbose         Provides more verbose output.
  --spin-time SPIN_TIME
                        Spin time in seconds to wait for discovery (only applies when not using an already running daemon)
```

Example output:

```bash
$ ros2 launch attach 50bda6fb-d451-4d53-8a2b-e8fcdce8170b
Attached to Launch System 50bda6fb-d451-4d53-8a2b-e8fcdce8170b.
(... in another terminal, run `ros2 launch term 50bda6fb`...)
All nodes in Launch System 50bda6fb-d451-4d53-8a2b-e8fcdce8170b have exited.
$
```

Verbose mode:

```bash
$ ros2 launch attach -v 50bda6fb
Attached to Launch System 50bda6fb-d451-4d53-8a2b-e8fcdce8170b.
Waiting for node /launch_ros
Waiting for node /talker
Waiting for node /listener
(... in another terminal, run `ros2 launch term 50bda6fb`...)
Node /launch_ros has exited
Node /talker has exited
Node /listener has exited
All nodes in Launch System 50bda6fb-d451-4d53-8a2b-e8fcdce8170b have exited.
$
```

#### `term`

Terminates all nodes that were launched under a specific Launch System ID.

```bash
$ ros2 launch term -h
usage: ros2 launch term [-h] [-v] [--spin-time SPIN_TIME] [system_id]

Terminates all nodes that were launched under a specific Launch System ID

positional arguments:
  system_id          Launch System ID of the nodes to terminate; if less than
                     a full UUID is specified, it will terminate nodes
                     belonging to the first Launch System it finds whose ID
                     begins with that sub-string

optional arguments:
  -h, --help            Show this help message and exit.
  -v, --verbose         Provides more verbose output.
  --spin-time SPIN_TIME
                        Spin time in seconds to wait for discovery (only
                        applies when not using an already running daemon)
```

Example output:

```bash
$ ros2 launch term 50bda6fb-d451-4d53-8a2b-e8fcdce8170b
Terminating Launch System 50bda6fb-d451-4d53-8a2b-e8fcdce8170b.
$
```
