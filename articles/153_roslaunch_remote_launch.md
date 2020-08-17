# Multi-Machine and Remote Launching
## Goals
- Connect to a remote host and run nodes on it
    - Ensure this capability does not compromise security
- Support arbitrary remote execution methods
- Monitor the status of and manage lifecycles of nodes across systems
- Shutdown nodes remotely
- Ability to detach the machine used to launch a system and have the system continue running
    - Be able to reconnect to the system to monitor and manage nodes running remotely


To achieve our goals, this update will consist of two primary components: an ExecuteRemote action, and a LaunchServiceNode.

The ExecuteRemote action will build off of the refactor he is working on described here: https://github.com/ros2/design/pull/272
Since we don't want to limit remote execution to a single protocol, ExecuteRemote will be a base class and we will create an implementation using SSH which we will call ExecuteRemoteSSH.
Remote machines will be described by a Machine class which will also be a base class for protocol-specific implementations.
Together these components will provide the means to execute processes remotely and should cover most basic use cases.

The LaunchServiceNode(s) will expose ROS2 services and topics for handling startup and shutdown of processes on remote machines.
This component won't be necessary for basic execution of remote nodes, but would allow remote system to persist beyond the life of the initial LaunchService.
The topics and services will allow remote tools to view and manage the processes running remotely.
To prevent execution of completely arbitrary processes, the node will be given a list of commands it is allowed to run at startup, each with a unique ID assigned by the user.
Requests to start or stop processes would then use the unique ID to indicate to the node which process to start or stop.


## New Classes

#### launch.descriptions.Machine
##### Members
Members would be defined by child classes, since different protocols will require different types of information

#### launch.descriptions.SSHMachine
Contains information needed to run a process on another machine
##### Members
| Name | Type | Description |
|---|---|---|
| hostname   | String | Hostname of the machine. |
| port       | int | Port on to connect to (default 22) |
| ssh\_keys  | String | Path to ssh key file |
| passphrase | String | Optional passphrase if the key uses one. |

#### launch.actions.ExecuteRemote
Base class for actions that use connection informationn contained in `machine` to start a process on a remote machine.
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


## ROS

### Nodes
#### launch\_ros.LaunchServiceNode
This node runs on each machine in a system is is repsonsible for launching and managing remote processes.
#### Params
| Name | Type | Description |
|---|---|---|
| allowable\_processes | List[Executable] | List of executables the node is allowed to launch |
##### Topics
###### Sent
- heartbeat
###### Received
##### Services
- query\_status
- start\_process
- stop\_process
- shutdown

### Messages
#### Heartbeat
Contains basic status information and enough information for launch related tools to find the node.
DDS might already provide enough info making this superfluous.
##### Fields
| Name | Type | Description |
|---|---|---|
| timestamp  | `builtin_interface.Timestamp` | Time at which heartbeat message was sent |
| hostname   | String | Hostname of machine sending the status |
| num\_procs | int | Number of processes currently being tracked by the LaunchServiceNode. |

#### ProcessStatus
##### Fields
| Name | Type | Description |
|---|---|---|
| timestamp       | `builtin_interface.Timestamp` | Time at which message was sent |
| process\_name   | String | Name of the process |
| process\_id     | int | A unique numerical identifier for the process. This value is generated by the launch system and should not be expected to be the same as the proc id on the machine. |
| process\_status | int | An enumerated status value. STOPPED=0; RUNNING=1; SUSPENDED=2; |
| mem\_usage      | double | Percent of memory used by the process |
| processor\_load | double | Percent of processor load coming from this process |
| err\_msg        | String | Error string |

### Services

#### QueryStatus
Service to be called by launch tools to get detailed information about the processes being managed by the LaunchServiceNode
##### Request
| Name | Type | Description |
|---|---|---|
Empty since no info is needed to answer the request.
##### Response
| Name | Type | Description |
|---|---|---|
| timestamp               | `builtin_interface.Timestamp` | Time at which message was sent |
| hostname                | String | Hostname of machine sending the status |
| total\_mem\_usage       | double | Percent of memory used |
| total\_processor\_load  | double | processor load as a percent |
| num\_procs              | int | Number of processes being managed by the node |
| processes               | List[ProcessStatus] | List of ProcessStatus objects |

#### StartProcess
##### Request
| Name | Type | Description |
|---|---|---|
| cmd  | String | The command to be run when starting the process. |
| name | String | The name to use for the process. |
| uid  | int | A unique numerical id for use within the launch system. |
| cwd  | String | The directory from which to run the process on the remote machine. |
| env  | List[common\_interfaces.KeyValue] | Dictionary of environment variables to be used from a clean environment. |
##### Response
| Name | Type | Description |
|---|---|---|
| success | boolean | Whether the process was sucessfully started or not |
| status | ProcessStatus | Status of the process |

#### StopProcess
##### Request
| Name | Type | Description |
|---|---|---|
| proc\_id | int | The launch id of the process that should be stopped |
##### Response
| Name | Type | Description |
|---|---|---|
| success | boolean | Whether the process was sucessfully stopped or not |
| status  | ProcessStatus | Status of the process |

#### Shutdown
##### Request
Empty message
##### Response
| Name | Type | Description |
|---|---|---|
| success | boolean | Whether the process was sucessfully stopped or not |
| status  | ProcessStatus | Status of the process |
