# Refactoring ExecuteProcess in ROS2

Initially proposed in [114](https://github.com/ros2/launch/issues/114), this document describes changes to `ExecuteProcess` and related classes.
As initially implemented, some aspects of this class are tightly coupled to both the implementation chosen and the assumption that processes will execute only in the local environment of the launch process itself.
The following changes are designed to provide a more decoupled architecture that will ease support of launching processes locally, remotely, and potentially in containerized environments.
Additionally, these changes will support the goal of decoupling the description of an executable from the actual action of executing it.
These changes are designed so that existing systems may continue to use the interface before these changes without modification; newer systems may find benefits from adapting to the newer syntax, particularly when composing more complicated environments.

## Affected classes before any changes

The following classes currently exist and are relevant to the proposed changes.
A brief summary of some relevant concepts is included with each.

### launch.actions.ExecuteProcess

Extends the `launch.actions.Action` class.
This command initiates the execution of a command on the local system, and is not ROS-specific.

#### Constructor

Allows many items to be set via constructor, including information both on how to launch the process, and how to interact with it after launching.
The available parameters and their documentation follow, split into categories considering whether they are more relevant to launching the process, or to dealing with its execution following launch.

##### Launch Details

|Argument|Description|
|---|---|
|cmd|A list where the first item is the executable and the rest are arguments to the executable, each item may be a string or a list of strings and `Substitution`s to be resolved at runtime|
|name|The label used to represent the process, as a string or a `Substitution` to be resolved at runtime, defaults to the basename of the executable|
|cwd|The directory in which to run the executable|
|env|Dictionary of environment variables to be used, starting from a clean environment. If `None`, the current environment is used.|
|additional\_env|Dictionary of environment variables to be added. If `env` was `None`, they are added to the current environment. If not, `env` is updated with `additional_env`.|

##### Execution Details

|Argument|Description|
|---|---|
|shell|If True, a shell is used to execute the cmd|
|sigterm\_timeout|Time until shutdown should escalate to `SIGTERM`, as a string or a list of strings and `Substitution`s to be resolved at runtime, defaults to the `LaunchConfiguration` called `sigterm_timeout`|
|sigkill\_timeout|Time until escalating to `SIGKILL` after `SIGTERM`, as a string or a list of strings and `Substitution`s to be resolved at runtime, defaults to the `LaunchConfiguration` called `sigkill_timeout`|
|emulate\_tty|Emulate a tty (terminal), defaults to `False`, but can be overridden with the `LaunchConfiguration` called `emulate_tty`, the value of which is evaluated as true or false according to `evaluate_condition_expression`.|
|prefix|A set of commands/arguments to preceed the `cmd`, used for things like `gdb`/`valgrind` and defaults to the `LaunchConfiguration` called `launch-prefix`|
|output|Configuration for process output logging. Defaults to `log` i.e. log both `stdout` and `stderr` to launch main log file and stderr to the screen.|
|output\_format|For logging each output line, supporting `str.format()` substitutions with the following keys in scope: `line` to reference the raw output line and `this` to reference this action instance.|
|log\_cmd|If `True`, prints the final cmd before executing the process, which is useful for debugging when substitutions are involved.|
|on\_exit|List of actions to execute upon process exit.|

#### Properties

| Name            | Description                                                  |
| --------------- | ------------------------------------------------------------ |
| process_details | `None` if the process is not yet started; otherwise an object containing information such as the command, working directory, environment, and process ID |

Additional properties provide access to constructor parameters for `name`, `cmd`, `cwd`, `env`, `additional_env`, `shell`, and `output`.

#### Methods

| Name               | Description                                                  |
| ------------------ | ------------------------------------------------------------ |
| get_sub_entities   | Override. If on_exit was provided in the constructor, returns that; otherwise returns an empty list. |
| execute            | Override. Establishes event handlers for process execution, performs variable expansion, then uses `osrf_pycommon.process_utils.async_execute_process` to launch the defined process. |
| get_asyncio_future | Override. Return an asyncio Future, used to let the launch system know when we're done. |

#### Events

##### Handled Events

| Event Type                            | Description                                                  |
| ------------------------------------- | ------------------------------------------------------------ |
| `launch.events.process.ShutdownProcess` | Begins standard shutdown procedure for a running executable  |
| `launch.events.process.SignalProcess`   | Passes the signal provided by the event to the running process |
| `launch.events.process.ProcessStdin`    | Passes the text provided by the event to the stdin of the process |
| `launch.events.Shutdown`                | Same as ShutdownProcess                                      |

##### Emitted Events

| Event Type                           | Description                                                  |
| ------------------------------------ | ------------------------------------------------------------ |
| `launch.events.process.ProcessStarted` | Emitted when the process starts                              |
| `launch.events.process.ProcessExited`  | Emitted when the process exits; event contains return code   |
| `launch.events.process.ProcessStdout`  | Emitted when the process produces data the stdout pipe; event contains the data from the pipe |
| `launch.events.process.ProcessStderr`  | Emitted when the process produces data the stderr pipe; event contains the data from the pipe |

### launch_ros.actions.Node

Extends the `ExecuteProcess` class.
This allows execution of a ROS-specific node, and extends the available information to include node related details.

#### Constructor

Added constructor parameters for this class all focus on concepts similar to the Launch Details discussed above, but the class also overrides the `execute` method of `ExecuteProcess`.
The purpose of this override is to provide more granular structure for the command used to launch a ROS node.
All the provided constructor parameters eventually reduce into a collection of substitution elements to compose the `cmd` parameter of `ExecuteProcess`.

|Argument|Description|
|---|---|
|node_executable|The name of the executable to find if a package is provided or otherwise a path to the executable to run.|
|package|The package in which the node executable can be found|
|node_name|The name of the node|
|node_namespace|The ros namespace for this Node|
|parameters|List of names of yaml files with parameter rules, or dictionaries of parameters.|
|remappings|Ordered list of 'to' and 'from' string pairs to be passed to the node as ROS remapping rules|
|arguments|List of extra arguments for the node|

Additional parameters which were provided to `ExecuteProcess` are also applicable here, such as `name`, `cwd`, `env`, `additional_env`, `shell`, `sigterm_timeout`, `sigkill_timeout`, `prefix`, `output`, `output_format`, `log_cmd`, and `on_exit`.
The `cmd` parameter is *not* supported, as it will be composed to execute the ROS node specified by the `Node`-specific parameters.

#### Properties

| Name      | Description                                                  |
| --------- | ------------------------------------------------------------ |
| node_name | Returns the node's name after execution; this may be affected by substitutions. Throws an error if called prior to execution. |

Properties available from `ExecuteProcess` are inherited: `process_details`, along with constructor parameters for `name`, `cmd`, `cwd`, `env`, `additional_env`, `shell`, and `output`.

#### Methods

| Name    | Description                                                  |
| ------- | ------------------------------------------------------------ |
| execute | Override. Applies additional ROS-related substitutions to several parameters, then calls the superclass to execute the parsed command. |

Methods available from `Node` are inherited: `get_sub_entities` and `get_asyncio_future`.

#### Events

Inherits events from `Node`, but does not define any additional events.

### launch_ros.descriptions.ComposableNode

Does not extend another class.
Represents a node which may be executed as part of a composite process hosting multiple nodes.

#### Constructor

Constructor parameters closely mirror those of `Node`, despite not being related.

|Argument|Description|
|---|---|
|package|Name of the ROS package the node plugin lives in|
|node_plugin|Name of the plugin to be loaded|
|node_name|Name the node should have|
|node_namespace|Namespace the node should create topics/services/etc in|
|parameters|List of either paths to yaml files or dictionaries of parameters|
|remappings|List of from/to pairs for remapping names|
|extra_arguments|Container specific arguments to be passed to the loaded node|

#### Properties

Properties are available to access constructor parameters: `package`, `node_plugin`, `node_name`, `node_namespace`, `parameters`, `remappings`, and `extra_arguments`.

#### Methods

No custom methods are defined.

#### Events

No custom events are defined.

### launch_ros.actions.ComposableNodeContainer 

Extends the `Node` class.
This class allows loading of composable nodes into a container process.

#### Constructor

Added constructor parameters for this class allow specification of the node container class name and namespace, along with a list of `Node`s to include.

|Argument|Description|
|---|---|
|node_name|The name of the node, mandatory for full container node name resolution|
|node_namespace|The ros namespace for this Node, mandatory for full container node name resolution|
|composable\_node\_descriptions|Optional descriptions of composable nodes to be loaded|

Additional parameters which were provided to `Node` are also applicable here, such as `package`, `node_executable`, `parameters`, `remappings`, `arguments`, `name`, `cwd`, `env`, `additional_env`, `shell`, `sigterm_timeout`, `sigkill_timeout`, `prefix`, `output`, `output_format`, `log_cmd`, and `on_exit`. 

#### Properties

Properties available from `Node` are inherited: `node_name` and `process_details`, along with constructor parameters for `name`, `cmd`, `cwd`, `env`, `additional_env`, `shell`, and `output`.

#### Methods

| Name    | Description                                                  |
| ------- | ------------------------------------------------------------ |
| execute | Override. Adds actions to load the contained nodes once the process is started, then calls the superclass to execute the parsed command. |

Methods available from `Node` are inherited: `get_sub_entities` and `get_asyncio_future`.

#### Events

Inherits events from `Node`, but does not define any additional events.

### launch_ros.actions.LifecycleNode

Extends the `Node` class.
This class exposes additional events related to ROS lifecycle state transitions.

#### Constructor

Parameters which were provided to `Node` are applicable here, such as `package`, `node_name`, `node_namespace`, `node_executable`, `parameters`, `remappings`, `arguments`, `name`, `cwd`, `env`, `additional_env`, `shell`, `sigterm_timeout`, `sigkill_timeout`, `prefix`, `output`, `output_format`, `log_cmd`, and `on_exit`. 

#### Properties

Properties available from `Node` are inherited: `node_name` and `process_details`, along with constructor parameters for `name`, `cmd`, `cwd`, `env`, `additional_env`, `shell`, and `output`.

#### Methods

| Name    | Description                                                  |
| ------- | ------------------------------------------------------------ |
| execute | Override. Adds subscription event handlers to listen for ROS node lifecycle state changes; this handler will trigger appropriate events from this class when transitions are detected. |

#### Events

##### Handled Events

In addition to events handled by the `ExecuteProcess` superclass:

| Event Type                            | Description                                                  |
| ------------------------------------- | ------------------------------------------------------------ |
| `launch.events.lifecycle.ChangeState` | This event can be targeted to a single lifecycle node, or more than one, or even all lifecycle nodes, and it requests the targeted nodes to change state, see its documentation for more details |

##### Emitted Events

In addition to events emitted by the `ExecuteProcess` superclass:

| Event Type                                | Description                                                  |
| ----------------------------------------- | ------------------------------------------------------------ |
| `launch.events.lifecycle.StateTransition` | This event is emitted when a message is published to the "`/<node_name>/transition_event`" topic, indicating the lifecycle node represented by this action changed state |

## Considerations for Refactored Classes

The primary goal of the refactor is to attempt to split the definition aspect of processes/nodes apart from the logic which executes those elements.
`ComposableNode` provides a basis for similar breakouts of other elements.

In general, existing classes should have similar analogs in the refactor to ensure that existing classes can be replaced with backwards-compatible replacements that use the new structure.
These replacements can be implemented to warn the user that the older definitions are deprecated, and suggest moving to the newer refactored style.

Further consideration will be required for alternate frontends, such as XML, to determine how to implement these changes with streamlined syntax where possible.

One difficulty when looking forward to allowing functionality such as remote and containerized execution is that the target environment is likely different than the local environment.
Different packages may be installed, different environment variables may be defined, and so forth.
An subclass implementation of the proposed `launch.descriptions.Executable` class may choose to override the base `apply_context` method to implement additional layer(s) of substitutions appropriate to its destination contexts, such as remote installation paths or environment variables.
Additionally, the available support for concepts such as signals and PIDs may be different; to that end, it is likely that remote execution of processes will require sibling classes to those described below, rather than direct subclasses.

## Proposed New Classes

### launch.descriptions.Executable

This class would represent only the information required to define an entity which might be run.
It would not include execution-time details, nor provide monitoring of the process after launch. 

#### Constructor

The constructor parameters would be pulled directly from the current `launch.actions.ExecuteProcess` class.

| Argument        | Description                                                  |
| --------------- | ------------------------------------------------------------ |
| cmd             | A list where the first item is the executable and the rest are arguments to the executable, each item may be a string or a list of strings and `Substitution`s to be resolved at runtime |
| name            | The label used to represent the process, as a string or a `Substitution` to be resolved at runtime, defaults to the basename of the executable |
| cwd             | The directory in which to run the executable                 |
| env             | Dictionary of environment variables to be used, starting from a clean environment. If `None`, the current environment is used. |
| additional\_env | Dictionary of environment variables to be added. If `env` was `None`, they are added to the current environment. If not, `env` is updated with `additional_env`. |

#### Properties

Accessors would be provided for the various constructor parameters: `cmd`, `name`, `cwd`, `env`, and `additional_env`.

#### Methods

In the current classes, substitutions are handled through internal method calls.
To support better isolation of class functionality, substitutions will be performed in this class, but exposed from a public method which can be overridden by subclasses to perform additional appropriate substitutions.
Substituted versions of the various parameters will be stored in a local dictionary, similar to the current `__process_event_args`.

| Name          | Description                                                  |
| ------------- | ------------------------------------------------------------ |
| apply_context | Takes a given LaunchContext and performs actions necessary to prepare to execute the defined command in that context. This will primarily consist of expanding various substitutions. |

#### Events

No events would be handled nor emitted.

### launch.actions.ExecuteLocal

Extends the `launch.actions.Action` class.
This class would represent the execution-time aspects of `launch.actions.ExecuteProcess`.
The new `process_description` constructor parameter could be a `launch.descriptions.Executable` or a subclass.

This class is simplified from the current `launch.actions.ExecuteProcess` by removing the logic relating to process definition/configuration, and by removing the logic regarding substitutions thereof.

(Details regarding remote/containerized execution would not be considered here, and should be implemented in alternate execute actions as appropriate.
They could, however, reuse the same process/node description classes, while providing alternate `launch.LaunchContext` information.)

#### Constructor

| Argument             | Description                                                  |
| -------------------- | ------------------------------------------------------------ |
| process\_description | The `launch.descriptions.Executable` to execute as a local process |
| shell                | If True, a shell is used to execute the cmd                  |
| sigterm\_timeout     | Time until shutdown should escalate to `SIGTERM`, as a string or a list of strings and `Substitution`s to be resolved at runtime, defaults to the `LaunchConfiguration` called `sigterm_timeout` |
| sigkill\_timeout     | Time until escalating to `SIGKILL` after `SIGTERM`, as a string or a list of strings and `Substitution`s to be resolved at runtime, defaults to the `LaunchConfiguration` called `sigkill_timeout` |
| emulate\_tty         | Emulate a tty (terminal), defaults to `False`, but can be overridden with the `LaunchConfiguration` called `emulate_tty`, the value of which is evaluated as true or false according to `evaluate_condition_expression`. |
| prefix               | A set of commands/arguments to preceed the `cmd`, used for things like `gdb`/`valgrind` and defaults to the `LaunchConfiguration` called `launch-prefix` |
| output               | Configuration for process output logging. Defaults to `log` i.e. log both `stdout` and `stderr` to launch main log file and stderr to the screen. |
| output\_format       | For logging each output line, supporting `str.format()` substitutions with the following keys in scope: `line` to reference the raw output line and `this` to reference this action instance. |
| log\_cmd             | If `True`, prints the final cmd before executing the process, which is useful for debugging when substitutions are involved. |
| on\_exit             | List of actions to execute upon process exit.                |

#### Properties

| Name            | Description                                                  |
| --------------- | ------------------------------------------------------------ |
| process_details | `None` if the process is not yet started; otherwise an object containing information such as the command, working directory, environment, and process ID |

Additional properties provide access to constructor parameters for `process_description`, `shell`, and `output`.

#### Methods

| Name               | Description                                                  |
| ------------------ | ------------------------------------------------------------ |
| get_sub_entities   | Override. If on_exit was provided in the constructor, returns that; otherwise returns an empty list. |
| execute            | Override. Establishes event handlers for process execution, passes the execution context to the process definition for substitution expansion, then uses `osrf_pycommon.process_utils.async_execute_process` to launch the defined process. |
| get_asyncio_future | Override. Return an asyncio Future, used to let the launch system know when we're done. |

#### Events

##### Handled Events

| Event Type                              | Description                                                  |
| --------------------------------------- | ------------------------------------------------------------ |
| `launch.events.process.ShutdownProcess` | Begins standard shutdown procedure for a running executable  |
| `launch.events.process.SignalProcess`   | Passes the signal provided by the event to the running process |
| `launch.events.process.ProcessStdin`    | Passes the text provided by the event to the stdin of the process |
| `launch.events.Shutdown`                | Same as ShutdownProcess                                      |

##### Emitted Events

| Event Type                             | Description                                                  |
| -------------------------------------- | ------------------------------------------------------------ |
| `launch.events.process.ProcessStarted` | Emitted when the process starts                              |
| `launch.events.process.ProcessExited`  | Emitted when the process exits; event contains return code   |
| `launch.events.process.ProcessStdout`  | Emitted when the process produces data the stdout pipe; event contains the data from the pipe |
| `launch.events.process.ProcessStderr`  | Emitted when the process produces data the stderr pipe; event contains the data from the pipe |

### launch_ros.descriptions.Node

This class would represent only the common information required to define a ROS2 node.
It would not inherit from any other class, and would have a subset of the fields of the current `launch_ros.actions.Node`.

#### Constructor

These parameters are drawn from the shared parameters of the current `launch_ros.actions.Node` and `launch_ros.descriptions.ComposableNode`, and introduces a new collection of `traits`, described in following classes.

| Argument       | Description                                                  |
| -------------- | ------------------------------------------------------------ |
| node_name      | Name the node should have                                    |
| node_namespace | Namespace the node should create topics/services/etc in      |
| parameters     | List of either paths to yaml files or dictionaries of parameters |
| remappings     | List of from/to pairs for remapping names                    |
| arguments      | Container specific arguments to be passed to the node        |
| traits      | Additional features of the node, specifically Lifecycle or Composable information|

#### Properties

Accessors would be provided for the various constructor parameters: `node_name`, `node_namespace`, `parameters`, `remappings`, `arguments`, and `traits`.

#### Methods

In the current classes, substitutions are handled through internal method calls.
To support better isolation of class functionality, substitutions will be performed in this class, but exposed from a public method which can be overridden by subclasses to perform additional appropriate substitutions.

| Name    | Description                                                  |
| ------- | ------------------------------------------------------------ |
| prepare | Takes a given LaunchContext and Action, and performs actions necessary to prepare the node for execution in that context. This will primarily consist of expanding various substitutions, but may additionally add event handlers related to the Action which will be invoked. Additionally, this will call the `prepare()` method of any defined traits. |

This function would match that of the current `launch_ros.actions.Node` internal method `_perform_substitutions`.

#### Events

No events would be handled or emitted.

### launch_ros.descriptions.ComposableNode

This class would extend the `launch_ros.descriptions.Node` class, and provide the additional information required for defining a node which can be launched in a composable context. 

#### Constructor

Most parameters would be passed to the superclass.

|Argument|Description|
|---|---|
| package        | Optional. Name of the ROS package the node plugin lives in. If not specified, the package of the containing executable will be assumed.         |
|node_plugin|Name of the plugin to be loaded|

Additional parameters that may be passed, which are handled by `launch_ros.descriptions.NodeBase`: `node_name`, `node_namespace`, `parameters`, `remappings`, `arguments`, `traits`.

#### Properties

Accessors would be provided for the additional constructor parameters: `package`, `node_plugin`.
Inherited accessors would also be available: `node_name`, `node_namespace`, `parameters`, `remappings`, `arguments`, and `traits`.

#### Methods

No methods would be defined or overridden.

#### Events

No events would be handled or emitted.

### launch_ros.traits.NodeTrait
This abstract class would not extend any others, and would provide a common interface to allow definition of traits which apply to `launch_ros.descriptions.Node` objects.

#### Constructor

This constructor would take no parameters, and should not be used directly.

#### Properties

This object would expose no properties.

#### Methods

To allow a trait to perform its actions, the `prepare()` method will be called prior to execution.

| Name    | Description                                                  |
| ------- | ------------------------------------------------------------ |
| prepare | Abstract method which takes a given Node, LaunchContext, and Action, and performs actions necessary to prepare the node for execution in that context.|

#### Events

No events would be handled or emitted.

### launch_ros.traits.HasLifecycle

This class would extend the `launch_ros.traits.NodeTrait` class, and provide the additional functionality required for managing lifecycle node events that are currently defined in `launch_ros.actions.LifecycleNode`. 

#### Constructor

The constructor for this trait would take no parameters, as it serves only to assert that the ROS2 Lifecycle events are implemented by the `launch_ros.descriptions.Node`, and appropriate events should be managed.

#### Properties

This object would expose no properties.

#### Methods

| Name    | Description                                                  |
| ------- | ------------------------------------------------------------ |
| prepare | Overridden. Adds subscription event handlers to listen for ROS node lifecycle state changes; this handler will trigger appropriate events from this class when transitions are detected. |

#### Events

##### Handled Events

| Event Type                            | Description                                                  |
| ------------------------------------- | ------------------------------------------------------------ |
| `launch.events.lifecycle.ChangeState` | This event can be targeted to a single lifecycle node, or more than one, or even all lifecycle nodes, and it requests the targeted nodes to change state, see its documentation for more details |

##### Emitted Events

| Event Type                                | Description                                                  |
| ----------------------------------------- | ------------------------------------------------------------ |
| `launch.events.lifecycle.StateTransition` | This event is emitted when a message is published to the "`/<node_name>/transition_event`" topic, indicating the lifecycle node represented by this action changed state |

### launch_ros.traits.ComposesNodes

This class would extend the `launch_ros.traits.NodeTrait` class, and provide the additional information required for launching composable nodes that are currently defined in `launch_ros.descriptions.ComposableNode`. 

#### Constructor

Most parameters would be passed to the new superclass.

|Argument|Description|
|---|---|
|nodes|Collection of `launch_ros.descriptions.ComposableNode` objects that should be launched inside the `launch_ros.descriptions.Node` with this trait.|

#### Properties

Accessors would be provided for the constructor parameter: `nodes`.

#### Methods

| Name    | Description                                                  |
| ------- | ------------------------------------------------------------ |
| prepare | Overridden. Adds an event handler to launch the defined composable nodes in the container once the container is started. |

#### Events

No events would be handled or emitted.

### launch_ros.descriptions.RosExecutable

This class would represent the information required to run a ROS-aware process and inherit from `launch.descriptions.Executable`.
It would not include execution-time details, nor provide monitoring of the process after launch. 

#### Constructor

| Argument        | Description                                                  |
| --------------- | ------------------------------------------------------------ |
| package        | name of the ROS package the node executable lives in         |
|node_executable|Name of the executable to find|
| nodes             | A list of `launch_ros.descriptions.Node` objects which are part of the executable. |

Additional parameters that may be passed, which are handled by `launch.actions.ExecuteProcess`: `cmd`, `name`, `cwd`, `env`, `additional_env`.
NOTE: To allow ROS to determine the appropriate executable based on the package and executable name, the `cmd` parameter should *NOT* be specified. When `cmd` is not specified, this class will determine the appropriate executable if and only if exactly one of the nodes it contains has the `launch_ros.traits.IsExecutable` trait.

#### Properties

Accessors would be provided for the constructor parameters: `package`, `node_executable`, `nodes`.
Inherited accessors would also be available: `cmd`, `name`, `cwd`, `env`, and `additional_env`.

#### Methods

| Name          | Description                                                  |
| ------------- | ------------------------------------------------------------ |
| apply\_context | Overridden. Calls `prepare()` on each `launch_ros.descriptions.Node`, and determines the appropriate executable if necessary. |

In constructing the command line parameters for execution, if more than one node is defined, all appropriate parameters will be prefixed with the node name as necessary.

#### Events

No events would be handled nor emitted.

## Future Extension

One topic of interest is the ability to launch nodes on remote machines as part of more complex systems.
While the current refactoring will not attempt to implement this functionality, such functionality could be implemented using sibling classes to those described above.

### launch.actions.ExecuteSshProcess

Extends the `launch.actions.Action` class.
This class would represent the execution-time aspects of an analog to `launch.actions.ExecuteProcess` which executes the given process via an SSH connection.
This likely would be a remote machine in virtually all usages. This is *not* a subclass of the proposed `launch.actions.ExecuteLocal` class; certain features provided by that class are not available from standard SSH connections.

It is also conceivable that this class, or one like it, could be constructed to simultaneously execute multiple processes over a single SSH connection.
If this is attempted, several features such as input/output pipes would likely no longer be feasible.

#### Constructor

| Argument               | Description                                                  |
| ---------------------- | ------------------------------------------------------------ |
| process\_description   | The `launch.descriptions.Executable` to execute as a remote process |
| connection_description | An object defining the SSH connection information, such as host, port, user, etc. |
| prefix                 | A set of commands/arguments to preceed the `cmd`, used for things like `gdb`/`valgrind` and defaults to the `LaunchConfiguration` called `launch-prefix` |
| output                 | Configuration for process output logging. Defaults to `log` i.e. log both `stdout` and `stderr` to launch main log file and stderr to the screen. |
| output\_format         | For logging each output line, supporting `str.format()` substitutions with the following keys in scope: `line` to reference the raw output line and `this` to reference this action instance. |
| log\_cmd               | If `True`, prints the final cmd before executing the process, which is useful for debugging when substitutions are involved. |
| on\_exit               | List of actions to execute upon process exit.                |

The following constructor parameters which were present in the proposed `launch.actions.ExecuteLocalProcess` are not provided as part of this class due to the SSH execution environment.

| Argument        | Rationale for Exclusion                                      |
| --------------- | ------------------------------------------------------------ |
| shell           | The process is being executed through SSH, which is by definition a shell |
| sigterm_timeout | Signal handling across SSH is not robust                     |
| sigkill_timeout | Signal handling across SSH is not robust                     |
| emulate_tty     | The process is being executed through SSH, which may restrict flexibility of TTY choice |

#### Properties

| Name            | Description                                                  |
| --------------- | ------------------------------------------------------------ |
| process_details | `None` if the process is not yet started; otherwise an object containing information such as the command, working directory, environment, and process ID |

Additional properties provide access to constructor parameters for `process_description`, `connection_description`, and `output`.

#### Methods

| Name               | Description                                                  |
| ------------------ | ------------------------------------------------------------ |
| get_sub_entities   | Override. If on_exit was provided in the constructor, returns that; otherwise returns an empty list. |
| execute            | Override. Establishes event handlers for process execution, passes the execution context to the process definition for substitution expansion, then uses `osrf_pycommon.process_utils.async_execute_process` to launch the defined process. |
| get_asyncio_future | Override. Return an asyncio Future, used to let the launch system know when we're done. |

This assumes the SSH interface is provided by a library with get_asyncio_future support; if such a library is not found/used, an alternate method for providing the appropriate feedback will be required.

#### Events

##### Handled Events

| Event Type                              | Description                                                  |
| --------------------------------------- | ------------------------------------------------------------ |
| `launch.events.process.ShutdownProcess` | Begins standard shutdown procedure for a running executable  |
| `launch.events.process.ProcessStdin`    | Passes the text provided by the event to the stdin of the process |
| `launch.events.Shutdown`                | Same as ShutdownProcess                                      |

The `launch.events.process.SignalProcess` event is not handled by this class, due to the difficulty in managing signals across SSH.
The `ProcessStdin` event may need to be evaluated, depending on the SSH interface.

##### Emitted Events

| Event Type                             | Description                                                  |
| -------------------------------------- | ------------------------------------------------------------ |
| `launch.events.process.ProcessStarted` | Emitted when the process starts                              |
| `launch.events.process.ProcessExited`  | Emitted when the process exits; event contains return code   |
| `launch.events.process.ProcessStdout`  | Emitted when the process produces data the stdout pipe; event contains the data from the pipe |
| `launch.events.process.ProcessStderr`  | Emitted when the process produces data the stderr pipe; event contains the data from the pipe |

The `ProcessStdout` and `ProcessStderr` events may need to be evaluated, depending on the SSH interface.