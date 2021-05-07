## Desired User Experience
This section describes the desired user interface of the ROS2 logging system.
To define the interface of the ROS2 logging system, it is helpful to consider all of the various tasks users will want to perform.
From this list of tasks, we can describe the steps users will have to take in order to perform each one.
Based on our experiences with ROS1, other logging systems, and community feedback, the logging system in ROS2 will support the following interactions:
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
This interaction contitutes creating the objects that users will use to output information from their code.
#### Setting Log Levels
This is how users set the severity levels for the loggers in their code.
There are a number of ways users might want to be able to change these levels, so this interaction is further broken down to describe each of those.
##### Programatically
##### Command Line
##### Via ROS2 Service Call
##### Environment Variable?
#### Outputting Logs
This is primary interaction where users output information from their code.
#### Defining Output Destinations (Sinks)
Users may want logged information printed to the screen, written to a file, published to a topic, or a combination of those. This is how users setup those preferences.
#### Specifying A Backend
There are a number of widely used third-party logging libraries already available which may better suit a users use-case or provide extra features they need.
ROS2 logging provides a default implementation, as well as a couple alternatives.
This defines how users choose the implementation that best suits their needs.
#### Creating A Backend
Users may have requirements that none of the included backends support, so ROS2 provides an interface for users to create custom backend implementations.
#### Tailoring Performance Impact
Adding a sophisticated logging system to a complex code base can have performance impacts and are often mostly useful during development and debugging.
ROS2 provides a few ways to configure the logging system to eliminate this performance impact when the logs are not needed.
This section describes how to make those configuration changes.



## Existing infrastructure
ROS2 
### `rcutils`
Contains macros for logging at several different levels and under various constraints
### `rclcpp`
Contains similar macros to those in `rcutils`, which call down to the macros in `rcutils`
### `rcl_logging`
Contains a logging interface and several implementations including log4cxx, spdlog, and a no-op implementation
### `rcl`
Contains infrastructure to tie the logger implementations defined in `rcl_logging` to the macros in `rcutils`



## User Interactions
The focus of this document is to help decide what the user experience with the ROS2 logging system should look like and to identify what changes or additions need to be made to the existing infrastructure to enable that experience. 
Below are enumerated the key high-level tasks that users of ROS2 will need to perform and descriptions of the current best-practices for achieving them according to my current understanding of the system.

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
`rcl_logging` contians a couple different backend implementations, as well as an interface that can be used to add additional implementations. The environment variable `RCL_LOGGING_IMPLEMENTATION` can be set at compile time to select which implementation to compile in.
### Avoiding Performance Impact
`rcutils` and `rclcpp` include a flag called `RCLCPP_LOG_MIN_SEVERITY` which can be set to compile out all macros below a certain level. Constants of the form `RCLCPP_LOG_MIN_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL|NONE]` are provided to facilitate setting the min severity that should be compiled out. (NOTE: in `rcutils` the `RCLCPP` is replaced with `RCUTILS`).


