1. What ROS1 had(?)
2. what exists
### Existing infrastructure
ROS2 
#### `rcutils`
Contains macros for logging at several different levels and under various constraints
#### `rclcpp`
Contains similar macros to those in `rcutils`, which call down to the macros in `rcutils`
#### `rcl_logging`
Contains a logging interface and several implementations including log4cxx, spdlog, and a no-op implementation
#### `rcl`
Contains infrastructure to tie the logger implementations defined in `rcl_logging` to the macros in `rcutils`

### User Interactions
The focus of this document is to help decide what the user experience with the ROS2 logging system should look like and to identify what changes or additions need to be made to the existing infrastructure to enable that experience. 
Below are enumerated the key high-level tasks that users of ROS2 will need to perform and descriptions of the current best-practices for achieving them according to my current understanding of the system.

#### Creating Loggers
Within a node, users can use the `rclcpp::Node::get_logger()` function to get a logger named after the node.
Outside of a node, users can use `rclcpp::get_logger(name)` to get a logger with the passed in name.
Loggers are created the first time these functions are called for a given name.
#### Setting Log Levels
##### Programmatically
Log level can be set programmatically with the `rcutils_set_logger_level(...)` function.
##### Externally
There is not currently a way to set the log level externally, though an example of using services to set log levels is included in the ros2 logging demo.
Log level is *supposed* to be able to be set via the command line while launching nodes, but that feature seems to be broken according to a couple reports (see: https://github.com/ros2/launch/issues/383)
#### Outputting Logs
Users that want to log information should use the `rclcpp` logging macros which cover a variety of use cases at 5 different severity levels.
#### Specifying A Backend
`rcl_logging` contians a couple different backend implementations, as well as an interface that can be used to add additional implementations. The environment variable `RCL_LOGGING_IMPLEMENTATION` can be set at compile time to select which implementation to compile in.
#### Avoiding Performance Impact
`rcutils` and `rclcpp` include a flag called `RCLCPP_LOG_MIN_SEVERITY` which can be set to compile out all macros below a certain level. Constants of the form `RCLCPP_LOG_MIN_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL|NONE]` are provided to facilitate setting the min severity that should be compiled out. (NOTE: in `rcutils` the `RCLCPP` is replaced with `RCUTILS`).


