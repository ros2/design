---
layout: default
title: ROS 2 Multi-Machine Launching
permalink: articles/roslaunch_mml.html
abstract:
  Robotic systems are often distributed across multiple networked machines.
  This document describes proposed modifications and enhancements to ROS2's
  launch system to facilitate launching, monitoring, and shutting
  down systems spread across multiple machines.
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

## Purpose

Allow a system of ROS nodes to be launched on a hardware architecture that is
spread across multiple networked computers and facilitate introspection and
management of the system from a single machine.

### Features and Considerations

Many of these are just extensions of the design goals for the single-machine
version of roslaunch for ROS2, but instead of only dealing with how to group
nodes in terms of processes, we can additionally group processes in terms of
machines. In addition to extending the current ROS2 launch goals for remote
machines, We would also like to consider more advanced features that could be
added such as advanced command line tools, forms of load balancing,
sending/retrieving files (such as configuration data or maps) to/from remote
machines.

    1. Some nodes may need to be run on specific machines due to hardware architecture.
        - Cameras or other sensors being directly connected
        - Specialized processing hardware
    2. Other nodes may not care which machine they run on and can be executed on machines with less workload as determined at time of launch.
    3. Need to manage lifecycles of nodes that have them.
    4. Need to monitor node status, and attempt recovery from failures.
        - Attempt recovery from crashed nodes by restarting them, possibly on a different machine.
    5. Connect to remote machines securely over SSH.
        - How to manage credentials?
            - Have users manually set up the accounts and put passwords in launch files?
            - Have users set up ssh keys for the computers in the system?
    6. Provide command line tools for managing and monitoring launched systems on remote machines.
    7. Provide mechanisms for locating files and executables across machines, and sending files to machines that need them for certain nodes.
        - If we intend to do any kind of load balancing or launching of nodes on non-specified machines (i.e. determined by the launch system during the launch process rather than specified by the user in the launch file), certain types of resources may need to be transferred to other machines. Calibration data, map files, training data, etc. This potentially creates a need to keep track of which machine has the most recent version of such resources (e.g. in the case of training data, or any other resource where a node might save data to be loaded next time it is launched).
    8. Should be able to work with machines running different operating systems on the same network.


## Proposed Multi-Machine Launch Command Line Interface

The multi-machine launching interface is controlled through the `launcher`
command for the `ros2` command-line tool.  The existing `launch` command
provides a subset of this functionality that is sufficient for single-machine
launching.

### Commands

```bash
$ ros2 launcher
usage: ros2 launcher [-h] Call `ros2 launcher <command> -h` for more detailed usage. ...

Various launching related sub-commands

optional arguments:
  -h, --help            show this help message and exit

Commands:
  launch       Run a launch file
  list         Search for and list running launch systems
  attach       Attach to a running launch system and wait for it to finish
  term         Terminate a running launch system

  Call `ros2 launcher <command> -h` for more detailed usage.
```

#### `launch`

The `ros2 launcher launch` is equivalent to `ros2 launch`, which is preserved
for backwards compatibility and ease of use.  It is used to run a launch file.

```bash
$ ros2 launcher launch -h
usage: ros2 launcher launch [-h] [-d] [-D] [-p | -s] [-a]
                            package_name [launch_file_name]
                            [launch_arguments [launch_arguments ...]] ...

Run a launch file

positional arguments:
  package_name          Name of the ROS package which contains the launch file
  launch_file_name      Name of the launch file
  launch_arguments      Arguments to the launch file; '<name>:=<value>' (for
                        duplicates, last one wins)
  argv                  Pass arbitrary arguments to the launch file

optional arguments:
  -h, --help            Show this help message and exit.
  -d, --debug           Put the launch system in debug mode, provides more
                        verbose output.
  -D, --detach          Detach from the launch process after it has started.
  -p, --print, --print-description
                        Print the launch description to the console without
                        launching it.
  -s, --show-args, --show-arguments
                        Show arguments that may be given to the launch file.
  -a, --show-all-subprocesses-output
                        Show all launched subprocesses' output by overriding
                        their output configuration using the
                        OVERRIDE_LAUNCH_PROCESS_OUTPUT envvar.
```

Example output:

```bash
$ ros2 launcher launch demo_nodes_cpp talker_listener.launch.py
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

Note how there is one difference from the old behavior of `ros2 launch`; the
group of nodes is assigned a Launch System ID.  This is a unique identifier
that can be used to track all of the nodes launched by a particular command
across a network.

Additionally, it is possible to detach from a system and let it run in the
background:

```bash
$ ros2 launcher launch -D demo_nodes_cpp talker_listener.launch.py
[INFO] [launch]: All log files can be found below /home/preed/.ros/log/2019-09-11-20-54-30-715383-regulus-2799
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch]: Launch System ID is 50bda6fb-d451-4d53-8a2b-e8fcdce8170b
$
```

#### `list`

Since it is possible to launch a system of nodes that spans a network and detach
from it, it is necessary to be able to query the network to find which systems
are active.

```bash
$ ros2 launcher list -h
usage: ros2 launcher list [-h] [-v] [--spin-time SPIN_TIME]

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
$ ros2 launcher list
ab1e0138-bb22-4ec9-a590-cf377de42d0f
50bda6fb-d451-4d53-8a2b-e8fcdce8170b
5d186778-1f50-4828-9425-64cc2ed1342c
$
```

```bash
$ ros2 launcher list
ab1e0138-bb22-4ec9-a590-cf377de42d0f: 5 nodes, 2 hosts
50bda6fb-d451-4d53-8a2b-e8fcdce8170b: 2 nodes, 1 host
5d186778-1f50-4828-9425-64cc2ed1342c: 16 nodes, 3 hosts
$
```

#### `attach`

Since it is possible to detach from a launched system, it is useful for
scripting or diagnostic purposes to be able to re-attach to it.

```bash
$ ros2 launcher attach -h
usage: ros2 launcher attach [-h] [-v] [--spin-time SPIN_TIME] [system_id]

Blocks until all nodes running under the specified Launch System ID have exited

positional arguments:
  system_id          Launch System ID of the nodes to attach to; if less than
                     a full UUID is specified, it will attach to the first
                     Launch System it finds whose ID begins with that sub-string

optional arguments:
  -h, --help            Show this help message and exit.
  -v, --verbose         Provides more verbose output.
  --spin-time SPIN_TIME
                        Spin time in seconds to wait for discovery (only
                        applies when not using an already running daemon)
```

Example output:

```bash
$ ros2 launcher attach 50bda6fb-d451-4d53-8a2b-e8fcdce8170b
Attached to Launch System 50bda6fb-d451-4d53-8a2b-e8fcdce8170b.
(... in another terminal, run `ros2 launcher term 50bda6fb`...)
All nodes in Launch System 50bda6fb-d451-4d53-8a2b-e8fcdce8170b have exited.
$
```

Verbose mode:

```bash
$ ros2 launcher attach -v 50bda6fb
Attached to Launch System 50bda6fb-d451-4d53-8a2b-e8fcdce8170b.
Waiting for node /launch_ros
Waiting for node /talker
Waiting for node /listener
(... in another terminal, run `ros2 launcher term 50bda6fb`...)
Node /launch_ros has exited
Node /talker has exited
Node /listener has exited
All nodes in Launch System 50bda6fb-d451-4d53-8a2b-e8fcdce8170b have exited.
$
```

#### `term`

Terminates all nodes that were launched under a specific Launch System ID.

```bash
$ ros2 launcher term -h
usage: ros2 launcher term [-h] [-v] [--spin-time SPIN_TIME] [system_id]

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
$ ros2 launcher term 50bda6fb-d451-4d53-8a2b-e8fcdce8170b
Terminating Launch System 50bda6fb-d451-4d53-8a2b-e8fcdce8170b.
$
```
