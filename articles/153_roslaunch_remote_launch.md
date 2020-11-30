# Multi-Machine and Remote Launching
This article describes the design of the components of the ros2 launch system related to the remote execution of processes.

## Goals
ROS 1's launch system provided the means to launch nodes remotely via SSH by defining a "machine" and referencing that machine in the declaration of each node meant to run on it.
The machine declaration would contain the information necessary to execute processes remotely, such as address, username, password, etc.
The goal of this design is to enable the same ability to launch nodes on remote machines, as well as to provide infrastructure for more advanced capabilities.

- Monitor the status and manage lifecycles of nodes across systems
- Shutdown nodes remotely
- Ability to detach the machine used to launch a system and have the system continue running
    - Be able to reconnect to the system to monitor and manage nodes running remotely


To achieve our goals, this update will consist of two primary components: an ExecuteRemote action, and a LaunchServiceNode.

The ExecuteRemote action will provide the basic capaiblity to start processes on remote machines.
It will build off of the refactor that is described [here](https://github.com/ros2/design/pull/272) and include a machine object as an additional parameter providing implementation-specific connection information.
Since we don't want to limit remote execution to a single protocol, ExecuteRemote will be a base class and we will create an implementation using SSH which we will call ExecuteRemoteSSH.
The machine class will also be a base class for protocol-specific implementations.
Together these components will provide the means to execute processes remotely and should cover most basic use cases.

The [LaunchServiceNode(s)](154_roslaunch_remote_launch_service_node.md) will expose ROS2 services and topics for handling startup and shutdown of processes on remote machines.
This component won't be necessary for basic execution of remote nodes, but would allow remote system to persist beyond the life of the initial LaunchService.
The topics and services will allow remote tools to view and manage the processes running remotely.
The LaunchServiceNode allows users to monitor and manage the status and lifecycles of nodes across systems, and enables the system to continue running if the machine used to start a system becomes disconnected.



## New Classes

#### launch.descriptions.Machine
##### Members
Members are defined by child classes, since different protocols require different types of information

#### launch.descriptions.SSHMachine
Contains information needed to run a process on another machine via ssh
##### Members
| Name | Type | Description |
|---|---|---|
| hostname   | String | Hostname of the machine. |
| port       | int | Port on the host to connect to (default 22) |
| ssh\_keys  | String | Path to ssh key file |
| passphrase | String | Optional passphrase if the key uses one. |

#### launch.actions.ExecuteRemote
Base class for actions that use connection information contained in `machine` to start a process on a remote machine.
##### Members
| Name | Type | Description |
|---|---|---|
| process\_description  |`launch.descriptions.Executable` | An object describing the process to be executed.|
| shell                 | boolean | If True, a shell is used to execute the process.|
| sigterm\_timeout     | List[Substitution] | Time until shutdown should escalate to `SIGTERM`, as a string or a list of strings and `Substitution`s to be resolved at runtime, defaults to the `LaunchConfiguration` called `sigterm_timeout` |
| sigkill\_timeout     | List[Substitution] | Time until escalating to `SIGKILL` after `SIGTERM`, as a string or a list of strings and `Substitution`s to be resolved at runtime, defaults to the `LaunchConfiguration` called `sigkill_timeout` |
| emulate\_tty         | boolean | Emulate a tty (terminal), defaults to `False`, but can be overridden with the `LaunchConfiguration` called `emulate_tty`, the value of which is evaluated as true or false according to `evaluate_condition_expression`. |
| prefix               | List[Substitution] | A set of commands/arguments to preceed the `cmd`, used for things like `gdb`/`valgrind` and defaults to the `LaunchConfiguration` called `launch-prefix` |
| output               | ?? | Configuration for process output logging. Defaults to `log` i.e. log both `stdout` and `stderr` to launch main log file and stderr to the screen. |
| output\_format       | ?? | For logging each output line, supporting `str.format()` substitutions with the following keys in scope: `line` to reference the raw output line and `this` to reference this action instance. |
| log\_cmd             | boolean | If `True`, prints the final cmd before executing the process, which is useful for debugging when substitutions are involved. |
| on\_exit             | List[LaunchDescriptionEntity] | List of actions to execute upon process exit.|
| persistent\_connection | boolean | Whether the LaunchService should maintain a persistent connection to the ssh instance |
| machine              | `launch.descriptions.SSHMachine` | The machine on which to execute the process |
##### Methods
| Name               | Type | Description                                                  |
| ------------------ | --- | ------------------------------------------------------------ |
| get\_sub\_entities | List[LaunchDescriptionEntity] | Override. If on\_exit was provided in the constructor, returns that; otherwise returns an empty list. |
| execute            | Optional[List[LaunchDescriptionEntity]] | Override. Establishes event handlers for process execution, passes the execution context to the process definition for substitution expansion, then uses `osrf_pycommon.process_utils.async_execute_process` to launch the defined process. |
| get\_asyncio\_future | Optional[asyncio.Future] | Override. Return an asyncio Future, used to let the launch system know when we're done. |


#### launch.actions.ExecuteRemoteSSH
Inherits from `launch.actions.ExecuteRemote` and manages the ssh connection
##### Members
Inherited
##### Methods
| Name | Type | Description |
|---|---|---|
| open\_ssh\_connection | SSHConnection | Object for interacting with the SSH connection |
| exec\_remote\_command | void | Executes a command on the remote machine

#### launch.substitutions.RemoteSubstitution
Uses a connection to a machine to run a command remotely and retuns the result as a string
##### Members
##### Constructor
| Argument | Type | Description |
|---|---|---|
| command | Text | The command to run on the remote machine |
| machine | `launch.descriptions.Machine` | An object decribing the machine that the command should be run on. |
##### Methods
| Name | Type | Description |
|---|---|---|
| describe | Text | Override method. Returns a description of this substitution as a string. |
| perform  | Text | Override method. Perform the substitution, given the launch context, and return it as a string. |

