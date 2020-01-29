# Refactoring ExecuteProcess in ROS2
Initially proposed in [114](https://github.com/ros2/launch/issues/114), this document describes changes to `ExecuteProcess` and related classes. As initially implemented, some aspects of this are tightly coupled to both the implementation chosen and the assumption that processes will execute only in the local environment of the launch process itself. The following changes are designed to provide a more decoupled architecture that will ease support of launching processes locally, remotely, or potentially in containerized environments. Additionally, these changes will support the goal of decoupling the description of an executable from the actual action of executing it. These changes are designed so that existing systems may continue to use the interface before these changes without modification; newer systems may find benefits from adapting to the newer syntax, particularly when composing more complicated environments.
## Affected classes before any changes
The following classes currently exist and are relevant to the proposed changes. A brief summary of some relevant concepts is included with each.
### launch.actions.ExecuteProcess
Extends the `Action` class. Allows many items to be set via constructor, including information both on how to launch the process, and how to interact with it after launching. The available parameters and their documentation follow, split into categories considering whether they are more relevant to launching the process, or to dealing with its execution following launch.

_Launch Details_

|Argument|Description|
|---|---|
|cmd|a list where the first item is the executable and the rest are arguments to the executable, each item may be a string or a list of strings and `Substitution`s to be resolved at runtime|
|name|the label used to represent the process, as a string or a `Substitution` to be resolved at runtime, defaults to the basename of the executable|
|cwd|the directory in which to run the executable|
|env|dictionary of environment variables to be used, starting from a clean environment. If `None`, the current environment is used.|
|additional\_env|dictionary of environment variables to be added. If `env` was `None`, they are added to the current environment. If not, `env` is updated with `additional_env`.|

_Execution Details_

|Argument|Description|
|---|---|
|shell|if True, a shell is used to execute the cmd|
|sigterm\_timeout|time until shutdown should escalate to `SIGTERM`, as a string or a list of strings and `Substitution`s to be resolved at runtime, defaults to the `LaunchConfiguration` called `sigterm_timeout`|
|sigkill\_timeout|time until escalating to `SIGKILL` after `SIGTERM`, as a string or a list of strings and `Substitution`s to be resolved at runtime, defaults to the `LaunchConfiguration` called `sigkill_timeout`|
|emulate\_tty|emulate a tty (terminal), defaults to `False`, but can be overridden with the `LaunchConfiguration` called `emulate_tty`, the value of which is evaluated as true or false according to `evaluate_condition_expression`.|
|prefix|a set of commands/arguments to preceed the `cmd`, used for things like `gdb`/`valgrind` and defaults to the `LaunchConfiguration` called `launch-prefix`|
|output|configuration for process output logging. Defaults to `log` i.e. log both `stdout` and `stderr` to launch main log file and stderr to the screen.|
|output\_format|for logging each output line, supporting `str.format()` substitutions with the following keys in scope: `line` to reference the raw output line and `this` to reference this action instance.|
|log\_cmd|if `True`, prints the final cmd before executing the process, which is useful for debugging when substitutions are involved.|
|on\_exit|list of actions to execute upon process exit.|

### launch_ros.actions.Node

Extends the `ExecuteProcess` class. Added constructor parameters for this class all focus on concepts similar to the Launch Details discussed above, but the class also overrides the `execute` method of `ExecuteProcess`. The purpose of this override is to provide more granular structure for the command used to launch a ROS node. All the provided constructor parameters eventually reduce into a collection of substitution elements to compose the `cmd` parameter of `ExecuteProcess`.

|Argument|Description|
|---|---|
|node_executable|the name of the executable to find if a package is provided or otherwise a path to the executable to run.|
|package|the package in which the node executable can be found|
|node_name|the name of the node|
|node_namespace|the ros namespace for this Node|
|parameters|list of names of yaml files with parameter rules, or dictionaries of parameters.|
|remappings|ordered list of 'to' and 'from' string pairs to be passed to the node as ROS remapping rules|
|arguments|list of extra arguments for the node|

### launch_ros.descriptions.ComposableNode

Does not extend another class. Constructor parameters closely mirror those of `Node`, and represent a node which may be executed as part of a composite process hosting multiple nodes.

|Argument|Description|
|---|---|
|package|name of the ROS package the node plugin lives in|
|node_plugin|name of the plugin to be loaded|
|node_name|name the node should have|
|node_namespace|namespace the node should create topics/services/etc in|
|parameters|list of either paths to yaml files or dictionaries of parameters|
|remappings|list of from/to pairs for remapping names|
|extra_arguments|container specific arguments to be passed to the loaded node|

### launch_ros.actions.ComposableNodeContainer 

Extends the `Node` class. Added constructor parameters for this class allow specification of the node container class name and namespace, along with a list of `Node`s to include. This class overrides the `execute` method of `Node` to add additional post-execution actions, loading the composable nodes into the now-executing container process.

|Argument|Description|
|---|---|
|node_name|the name of the node, mandatory for full container node name resolution|
|node_namespace|the ros namespace for this Node, mandatory for full container node name resolution|
|composable\_node\_descriptions|optional descriptions of composable nodes to be loaded|

### launch_ros.actions.LifecycleNode

Extends the `Node` class. This class overrides the `execute` method of `Node` to register lifecycle message handlers, and exposes events to allow reactions to said lifecycle state transitions.

## Considerations for Refactored Classes

The primary goal of the refactor is to attempt to split the definition aspect of processes/nodes apart from the logic which executes those elements. `ComposableNode` provides a basis for similar breakouts of other elements.

In general, classes should have similar analogs even in the refactor to ensure that existing classes can be replaced with backwards-compatible replacements that use the new structure. These replacements can be implemented to warn the user that the older definitions are deprecated, and suggest moving to the newer refactored style.

Further consideration will be required for alternate frontends, such as XML, to determine how to implement these changes with streamlined syntax where possible.

One difficulty when looking forward to allowing functionality such as remote and containerized execution is that the target environment is likely different than the local environment. Different packages may be installed, different environment variables may be defined, and so forth. To support this, it may be helpful to allow an alternate `launch.LaunchContext` to be defined for specific executions.

## Proposed New Classes
### launch.descriptions.Process

This class would represent only the information required to define a process which might be run. It would not include execution-time details, nor provide monitoring of the process after launch. The constructor parameters would be pulled directly from the current `launch.actions.ExecuteProcess` class.

|Argument|Description|
|---|---|
|cmd|a list where the first item is the executable and the rest are arguments to the executable, each item may be a string or a list of strings and `Substitution`s to be resolved at runtime|
|name|the label used to represent the process, as a string or a `Substitution` to be resolved at runtime, defaults to the basename of the executable|
|cwd|the directory in which to run the executable|
|env|dictionary of environment variables to be used, starting from a clean environment. If `None`, the current environment is used.|
|additional\_env|dictionary of environment variables to be added. If `env` was `None`, they are added to the current environment. If not, `env` is updated with `additional_env`.|
|shell|if True, a shell is used to execute the cmd|

### launch_ros.descriptions.Node

This class would represent only the information required to define a node which might be run. It would inherit from `launch.descriptions.Process`, and populate its fields in the same manner as the current `launch.actions.Node`.

|Argument|Description|
|---|---|
|node_executable|the name of the executable to find if a package is provided or otherwise a path to the executable to run.|
|package|the package in which the node executable can be found|
|node_name|the name of the node|
|node_namespace|the ros namespace for this Node|
|parameters|list of names of yaml files with parameter rules, or dictionaries of parameters.|
|remappings|ordered list of 'to' and 'from' string pairs to be passed to the node as ROS remapping rules|
|arguments|list of extra arguments for the node|

### launch_ros.descriptions.ComposableNode

This class would be modified to inherit from `launch_ros.descriptions.Node`. Most parameters would be passed to the new superclass.

|Argument|Description|
|---|---|
|node_plugin|name of the plugin to be loaded|

### launch.actions.ExecuteLocalProcess

This class would represent the execution-time aspects of launch.actions.ExecuteProcess. The new `process_description` constructor parameter could be either a `launch.descriptions.Process` or a `launch_ros.descriptions.Node`; node-specific functionality is limited to proper configuration of the command line to be executed, so no special execution wrapper is needed. This is not the case for lifecycle nodes or composable nodes, which do require custom execution wrappers, described below.

(Details regarding remote/containerized execution would not be considered here, and should be implemented in alternate execute actions as appropriate. They could, however, reuse the same process/node description classes, while providing alternate `launch.LaunchContext` information.)

|Argument|Description|
|---|---|
|process\_description|the `launch.descriptions.Process` to execute as a local process|
|shell|if True, a shell is used to execute the cmd|
|sigterm\_timeout|time until shutdown should escalate to `SIGTERM`, as a string or a list of strings and `Substitution`s to be resolved at runtime, defaults to the `LaunchConfiguration` called `sigterm_timeout`|
|sigkill\_timeout|time until escalating to `SIGKILL` after `SIGTERM`, as a string or a list of strings and `Substitution`s to be resolved at runtime, defaults to the `LaunchConfiguration` called `sigkill_timeout`|
|emulate\_tty|emulate a tty (terminal), defaults to `False`, but can be overridden with the `LaunchConfiguration` called `emulate_tty`, the value of which is evaluated as true or false according to `evaluate_condition_expression`.|
|prefix|a set of commands/arguments to preceed the `cmd`, used for things like `gdb`/`valgrind` and defaults to the `LaunchConfiguration` called `launch-prefix`|
|output|configuration for process output logging. Defaults to `log` i.e. log both `stdout` and `stderr` to launch main log file and stderr to the screen.|
|output\_format|for logging each output line, supporting `str.format()` substitutions with the following keys in scope: `line` to reference the raw output line and `this` to reference this action instance.|
|log\_cmd|if `True`, prints the final cmd before executing the process, which is useful for debugging when substitutions are involved.|
|on\_exit|list of actions to execute upon process exit.|

### launch_ros.actions.ExecuteLocalLifecycleNode

This class would extend the `launch.actions.ExecuteLocalProcess` class, and provide the additional functionality required for managing lifecycle node events that are currently defined in `launch_ros.actions.LifecycleNode`. No additional constructor parameters are defined in this class.

### launch_ros.actions.ExecuteLocalNodeContainer

This class would extend the `launch.actions.ExecuteLocalProcess` class, and provide the additional functionality required for loading multiple nodes into a single container. The description of the composable node container itself should be provided as the `process_description` parameter to pass to `ExecuteLocalProcess`; child nodes would be provided as an additional constructor parameter.

|Argument|Description|
|---|---|
|composable\_node\_descriptions|optional descriptions of composable nodes to be loaded|
