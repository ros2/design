## Desired User Experience
This section describes the desired user interface of the ROS 2 logging system.
To define the interface of the ROS 2 logging system, it is helpful to consider all of the various tasks users will want to perform.
From this list of tasks, we can describe the steps users will have to take in order to perform each one.
Based on our experiences with ROS 1, other logging systems, and community feedback, the logging system in ROS 2 will support the following interactions:
- creating loggers
- setting log levels
- outputting logs
- defining output destinations
- specifying a backend
- implementing a custom backend
- tailoring performance impact
### Interaction Descriptions
This section describes what each of the interactions in the list entails, and the steps users will have to take to complete them.
#### Creating Loggers
This interaction constitutes creating the objects that users will use to output information from their code.
Users are able to create multiple named loggers which they can use to exercise fine-grained control over the logging output at runtime.
Loggers are namespaced and can be controlled hierarchically by having their severity levels set individually.
If a logger's level is not set explicitly, it will use the level of the nearest ancestor with an explicit severity, or the default if no ancestors have had their levels set.
All named loggers are descendants of the default/unnamed logger.
#### Setting Log Levels
This is how users set the severity levels for the loggers in their code.
There are a number of ways users might want to be able to change these levels, so this interaction is further broken down to describe each of those.
##### Programmatically
Severity levels for each logger can be set programmatically by calling a function that takes the desired severity level, and optionally a reference to or the name of a specific logger.
If no name/reference is provided, function will apply the severity level as the default level.
##### Command Line
When launching nodes from the command line, an argument can be provided setting the default log severity for that node.
Programmatic logging severity changes within the node's code will override the severity set by the command line argument.
##### Via Launch File
Severity levels can be set for loggers by name via launch files.
The severity levels will be applied to the appropriate loggers at startup, but may be overridden by programmatic changes.
##### Via ROS 2 Service Call
Users will be able to control severity levels of loggers during runtime via ROS 2 service calls.
This level of control if often useful when debugging complex dynamic systems with many moving parts, especially when there is a high startup/restart cost.
Severity levels set this way will override levels set at launch via command line arguments or configuration files.
Programmatic changes to the severity may still occur after a level has been changed via the service call and will override the level set by the service call.
#### Outputting Logs
This is the primary interaction where users output information from their code.
Users call macros that take loggers and message text as arguments and generate the code to produce log output when appropriate.
Logging macros exist for each of the severity levels, and the macros for each severity cover a variety of use cases such as how frequently to output or whether to output based on conditional logic.
The specific cases covered for each severity level are:
- Log once
- Log if given expression is true
- Log if given function returns true
- Ignore first call
- Throttle to a certain rate (Don't log until a given duration has passed since the previous log)
- Throttle, but ignore first call
#### Defining Output Destinations (Sinks)
Users may want logged information printed to the screen, written to a file, published to a topic, or a combination of those. This is how users setup those preferences.
By default, logs are printed to the stderr and written to a log file.
The location of the log file can be specified with an environment variable (`ROS_LOG_DIR`).
If the log file location environment variable is not set, it will default to `$ROS_HOME/log`, using `~/.ros` if `$ROS_HOME` is not set.
Console output can be redirected to stdout instead of stderr using an environment variable (`RCUTILS_LOGGING_USE_STDOUT`).
Individual loggers can be configured to output to specific files or output streams in launch files.
The destinations set in launch files can also be set according to severity level for a particular logger.
For example, a logger could be configured to output all messages of severity ERROR or greater to stderr and messages below that severity to stdout.
Logger configurations in launch files will override the behavior specified by the environment variables, and the behavior specified by the env variables will define the default behavior of any loggers not configured explicitly.
#### Specifying A Backend
There are a number of widely used third-party logging libraries already available which may better suit a users use-case or provide extra features they need.
This defines how users choose the implementation that best suits their needs.
ROS 2 provides a default implementation, as well as alternatives based on log4cxx and spdlog, and a no-op implementation.
The implementation can be specified at compile time by setting the environment variable `RCL_LOGGING_IMPLEMENTATION` to the name of the desired implementation when building `rcl`.
#### Creating A Backend
Users may have requirements that none of the included backends support, and may create additional implementations of `rcl_logging_interface`.
#### Tailoring Performance Impact
Adding a sophisticated logging system to a complex code base can have performance impacts and are often mostly useful during development and debugging.
ROS 2 provides a few ways to configure the logging system to eliminate this performance impact when the logs are not needed.
This section describes how to make those configuration changes.
#### Interacting with RMW Logging
RMW implementations have their own built-in logging, and a function called `rmw_set_log_severity` which may need to be handled with special considerations.



## Existing infrastructure
ROS 2
### `rcutils`
Contains macros for logging at several different levels and under various constraints
### `rclcpp`
Contains similar macros to those in `rcutils`, which call down to the macros in `rcutils`
### `rcl_logging`
Contains a logging interface and several implementations including log4cxx, spdlog, and a no-op implementation
### `rcl`
Contains infrastructure to tie the logger implementations defined in `rcl_logging` to the macros in `rcutils`



## User Interactions
The focus of this document is to help decide what the user experience with the ROS 2 logging system should look like and to identify what changes or additions need to be made to the existing infrastructure to enable that experience.
Below are enumerated the key high-level tasks that users of ROS 2 will need to perform and descriptions of the current best-practices for achieving them according to my current understanding of the system.

### Creating Loggers
Within a node, users can use the `rclcpp::Node::get_logger()` function to get a logger named after the node.
Outside of a node, users can use `rclcpp::get_logger(name)` to get a logger with the passed in name.
Loggers are created the first time these functions are called for a given name.
### Setting Log Levels
#### Programmatically
Log level can be set programmatically with the `rcutils_set_logger_level(...)` function.
#### Externally
There is not currently a way to set the log level externally, though an example of using services to set log levels is included in the ros2 logging demo.
Log level is *supposed* to be able to be set via the command line while launching nodes, but that feature seems to be broken according to a couple reports (see: https://github.com/ros2/launch/issues/383)
### Outputting Logs
Users that want to log information should use the `rclcpp` logging macros which cover a variety of use cases at 5 different severity levels.
### Specifying A Backend
`rcl_logging` contains a couple different backend implementations, as well as an interface that can be used to add additional implementations. The environment variable `RCL_LOGGING_IMPLEMENTATION` can be set at compile time to select which implementation to compile in.
### Avoiding Performance Impact
`rcutils` and `rclcpp` include a flag called `RCLCPP_LOG_MIN_SEVERITY` which can be set to compile out all macros below a certain level. Constants of the form `RCLCPP_LOG_MIN_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL|NONE]` are provided to facilitate setting the min severity that should be compiled out. (NOTE: in `rcutils` the `RCLCPP` is replaced with `RCUTILS`).


