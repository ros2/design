# Multi-Machine and Remote Launching
This article describes the design of the components of the ros2 launch system related to the remote execution of processes.

## Goals
ROS 1's launch system provided the means to launch nodes remotely via SSH by defining a "machine" and referencing that machine in the declaration of each node meant to run on it.
The machine declaration would contain the information necessary to execute processes remotely, such as address, username, password, etc.
These nodes would be executed on the specified machines and run until the launch process that started them was killed.

The goal of this design is to create an interface for similar remote execution capabilities to be added to the launch system for ROS2.
This design will include an SSH implementation of the interface, but the design of the interface itself will allow for implementations using other protocols to be added as well.
The primary component of this design will be the ExecuteRemote action.
It will be similar to the ExecuteLocal action described in the design for refactoring ExecuteProcess into Execute and Executable (https://github.com/ros2/design/pull/272), with the addition of a machine parameter providing implementation-specific connection information.
Since we don't want to limit remote execution to a single protocol, the ExecuteRemote and Machine classes will be base classes, and we will include implementations using SSH which we will call ExecuteRemoteSSH and SSHMachine, respectively.
Together these components will provide the means to execute processes remotely and should cover most basic use cases.




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
| ssh\_key   | String | Path to ssh key file |
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
| prefix               | List[Substitution] | A set of commands/arguments to precede the `cmd`, used for things like `gdb`/`valgrind` and defaults to the `LaunchConfiguration` called `launch-prefix` |
| output               | ?? | Configuration for process output logging. Defaults to `log` i.e. log both `stdout` and `stderr` to launch main log file and stderr to the screen. |
| output\_format       | ?? | For logging each output line, supporting `str.format()` substitutions with the following keys in scope: `line` to reference the raw output line and `this` to reference this action instance. |
| log\_cmd             | boolean | If `True`, prints the final cmd before executing the process, which is useful for debugging when substitutions are involved, though implementations should take care to ensure sensitive information such as keys and passphrases are hidden. |
| on\_exit             | List[LaunchDescriptionEntity] | List of actions to execute upon process exit.|
| persistent\_connection | boolean | Whether the LaunchService should maintain a persistent connection to the ssh instance |
| machine              | `launch.descriptions.Machine` | The machine on which to execute the process |
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
Uses a connection to a machine to run a command remotely and returns the result as a string
##### Members
##### Constructor
| Argument | Type | Description |
|---|---|---|
| command | Text | The command to run on the remote machine |
| machine | `launch.descriptions.Machine` | An object describing the machine that the command should be run on. |
##### Methods
| Name | Type | Description |
|---|---|---|
| describe | Text | Override method. Returns a description of this substitution as a string. |
| perform  | Text | Override method. Perform the substitution, given the launch context, and return it as a string. |

