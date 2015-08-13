---
layout: default
title: Migration guide from ROS 1
permalink: articles/migration_guide_from_ros1.html
abstract:
  This article describes the high-level steps to migrate a ROS 1 package to ROS 2.
  It does not aim to be a step-by-step migration instruction.
published: true
author: '[Dirk Thomas](https://github.com/dirk-thomas)'
---

* This will become a table of contents (this text will be scraped).
{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}


## Prerequisite

Before being able to migrate a ROS 1 package to ROS 2 all of its dependencies must be available in ROS 2.


## Migration steps

### Package manifests

ROS 2 only support the format 2 of the package specification which is defined in [REP 140](http://www.ros.org/reps/rep-0140.html).
Therefore the `package.xml` file must be update to format 2 if it uses format 1.
Since ROS 1 support both formats (1 as well as 2) it is safe to perform that conversion in the ROS 1 package.

Some packages might have different names in ROS 2 so the dependencies might need to be updated accordingly.


### Messages and services

Message files must end in `.msg` and must be located in the subfolder `msg`.
Service files must end in `.srv` and must be located in the subfolder `srv`.

These files might need to be updated to comply with the [ROS Interface definition](http://design.ros2.org/articles/interface_definition.html).
Some primitive types have been removed and the types `duration` and `time` which were builtin types in ROS 1 have been replaced with normal message definitions and must be used from the `builtin_interfaces` package.
Also some naming conventions are stricter then in ROS 1.


### Build system

The build system in ROS 2 is called [ament](http://design.ros2.org/articles/ament.html).


#### Build tool

Instead of using `catkin_make`, `catkin_make_isolated` or `catkin build` ROS 2 uses the command line tool [ament build](https://github.com/ament/ament_tools) to build and install a set of packages.


#### Pure Python package

If the ROS 1 package uses CMake only to invoke the `setup.py` file and does not contain anything beside Python code (e.g. also no messages, services, etc.) it should be converted into a pure Python package in ROS 2:

* Update the build type in the `package.xml` file:

    <export>
      <build_type>ament_python</build_type>
    </export>

* Remove the `CMakeLists.txt` file
* Update the `setup.py` file to be a standard Python setup script

ROS 2 supports Python 3 only.
While each package can choose to also support Python 2 it must invoke executables with Python 3 if it uses any API provided by other ROS 2 packages.


#### Update the *CMakeLists.txt* to use *ament_cmake*

Apply the following changes to use `ament_cmake` instead of `catkin`:

* Set the build type in the `package.xml` file:

    <export>
      <build_type>ament_cmake</build_type>
    </export>

* Replace the `find_package` invocation with `catkin` and the `COMPONENTS` with:

    find_package(ament_cmake REQUIRED)
    find_package(component1 REQUIRED)
    ...
    find_package(componentN REQUIRED)

* Move and update the `catkin_package` invocation with:

  * Invoke `ament_package` instead but **after** all targets have been registered.
  * The only valid argument for [ament_package](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/cmake/core/ament_package.cmake) is `CONFIG_EXTRAS`.
    All other arguments are covered by separate functions which all need to be invoked *before* `ament_package`.
  * The `INCLUDE_DIRS` arguments are passed to the new function [ament_export_include_directories](https://github.com/ament/ament_cmake/blob/master/ament_cmake_export_include_directories/cmake/ament_export_include_directories.cmake).
  * The `LIBRARIES` arguments are passed to the new functions [ament_export_libraries](https://github.com/ament/ament_cmake/blob/master/ament_cmake_export_libraries/cmake/ament_export_libraries.cmake) if the libraries are targets or [ament_export_library_names](https://github.com/ament/ament_cmake/blob/master/ament_cmake_export_libraries/cmake/ament_export_library_names.cmake) if the library is only identified by the name.
  * The `CATKIN_DEPENDS` and `DEPENDS` arguments are passed to the new function [ament_export_dependencies](https://github.com/ament/ament_cmake/blob/master/ament_cmake_export_dependencies/cmake/ament_export_dependencies.cmake).

* Replace the invocation of `add_message_files`, `add_service_files` and `generate_messages` with [rosidl_generate_interfaces](https://github.com/ros2/rosidl/blob/master/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake).

* Remove any occurrences of the *devel space*.
  Related CMake variables like `CATKIN_DEVEL_PREFIX` do not exist anymore.

* Replace `CATKIN_*_DESTINATION` variables with their actual paths according to the [FHS](http://www.pathname.com/fhs/):

  * `CATKIN_GLOBAL_BIN_DESTINATION`: `bin`
  * `CATKIN_GLOBAL_INCLUDE_DESTINATION`: `include`
  * `CATKIN_GLOBAL_LIB_DESTINATION`: `lib`
  * `CATKIN_GLOBAL_LIBEXEC_DESTINATION`: `lib`
  * `CATKIN_GLOBAL_SHARE_DESTINATION`: `share`

  * `CATKIN_PACKAGE_BIN_DESTINATION`: `lib/${PROJECT_NAME}`
  * `CATKIN_PACKAGE_INCLUDE_DESTINATION`: `include/${PROJECT_NAME}`
  * `CATKIN_PACKAGE_LIB_DESTINATION`: `lib`
  * `CATKIN_PACKAGE_SHARE_DESTINATION`: `share/${PROJECT_NAME}`

* Replace ROS 1 dependencies like `roscpp` with ROS 2 dependencies like `rclcpp`.
  Using `rclcpp` will also require to link explicitly against one rmw implementation, commonly the default is provided by the [rmw_implementation](https://github.com/ros2/rmw_implementation/tree/master/rmw_implementation) package.


#### Continue to use `catkin` in CMake

ROS 2 uses ament as the build system but for backward compatibility ROS 2 has a package called `catkin` which provides almost the same API as catkin in ROS 1.
In order to use this backward compatibility API the `CMakeLists.txt` must only be updated to call the function `catkin_ament_package()` *after* all targets.

<div class="alert alert-warning" markdown="1">
NOTE: This has not been implemented yet and is only an idea at the moment.
Due to the amount of changes related to dependencies it has not yet been decided if this compatibility API is useful enough to justify the effort.
</div>


### Update source code

#### Messages and services

The namespace of ROS 2 messages and services uses a subnamespace (`msg` or `srv`) after the package name.
Therefore an include looks like: `#include <my_interfaces/msg/my_message.hpp>`.
The C++ type is then named: `my_interfaces::msg::MyMessage`.

Shared pointer types are provided as typedefs within the message structs: `my_interfaces::msg::MyMessage::SharedPtr` as well as `my_interfaces::msg::MyMessage::ConstSharedPtr`.

For more details please see the article about the [generated C++ interfaces](http://design.ros2.org/articles/generated_interfaces_cpp.html).


#### ROS client library

<div class="alert alert-warning" markdown="1">
NOTE: to be written
</div>


## Launch files

While launch files in ROS 1 are specified using [.xml](http://wiki.ros.org/roslaunch/XML) files ROS 2 uses Python scripts to enable more flexibility (see [launch package](https://github.com/ros2/launch/tree/master/launch)).

## Example: Converting an existing ROS 1 package to use ROS 2

Let's say that we have simple ROS 1 package called `talker` that uses `roscpp`
in one node, called `talker`.
This package is in a catkin workspace, located at `~/ros1_talker`.

### The ROS 1 code

Here's the directory layout of our catkin workspace:

~~~
$ cd ~/ros1_talker
$ find .
.
./src
./src/talker
./src/talker/package.xml
./src/talker/CMakeLists.txt
./src/talker/talker.cpp
~~~

Here is the content of those three files:

~~~
$ cat src/talker/package.xml
{% include 200_migration_guide_from_ros1/ros1/src/talker/package.xml %}~~~

~~~
$ cat src/talker/CMakeLists.txt
{% include 200_migration_guide_from_ros1/ros1/src/talker/CMakeLists.txt %}~~~

~~~
$ cat src/talker/talker.cpp
{% include 200_migration_guide_from_ros1/ros1/src/talker/talker.cpp %}~~~

#### Building the ROS 1 code

We source an environment setup file (in this case for Jade using bash), then we
build our package using `catkin_make install`:

~~~
. /opt/ros/jade/setup.bash
cd ~/ros1_talker
catkin_make install
~~~

#### Running the ROS 1 node

If there's not already one running, we start a `roscore`, first sourcing the
setup file from our `catkin` install tree (the system setup file at
`/opt/ros/jade/setup.bash` would also work here):

~~~
. ~/ros1_talker/install/setup.bash
roscore
~~~

In another shell, we run the node from the `catkin` install space using
`rosrun`, again sourcing the setup file first (in this case it must be the one
from our workspace):
 
~~~
. ~/ros1_talker/install/setup.bash
rosrun talker talker
~~~

### Migrating to ROS 2

Let's start by creating a new workspace in which to work:

~~~
mkdir ~/ros2_talker
cd ~/ros2_talker
~~~

We'll copy the source tree from our ROS 1 package into that workspace, where we can modify it:

~~~
mkdir src
cp -a ~/ros1_talker/src/talker src
~~~

Now we'll modify the the C++ code in the node.
The ROS 2 C++ library, called `rclcpp`, provides a different API from that
provided by `roscpp`.
The concepts are very similar between the two libraries, which makes the changes
reasonably straightforward to make.

#### Included headers

In place of `ros/ros.h`, which gave us access to the `roscpp` library API, we
need to include `rclcpp/rclcpp.hpp`, which gives us access to the `rclcpp`
library API:

~~~
//#include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
~~~

To get the `std_msgs/String` message definition, in place of
`std_msgs/String.h`, we need to include `std_msgs/msg/string.hpp`:

~~~
//#include "std_msgs/String.h"
#include "std_msgs/msg/string.hpp"
~~~

#### Changing C++ library calls

Instead of passing the node's name to the library initialization call, we do
the initialization, then pass the node name to the creation of the node object
(we can use the `auto` keyword because now we're requiring a C++11 compiler):

~~~
//  ros::init(argc, argv, "talker");
//  ros::NodeHandle n;
    rclcpp::init(argc, argv);
    auto node = rclcpp::node::Node::make_shared("talker");
~~~

The creation of the publisher and rate objects looks pretty similar, with some
changes to the names of namespace and methods.
For the publisher, instead of an integer queue length argument, we pass a
quality of service (qos) profile, which is a far more flexible way to
controlling how message delivery is handled.
In this example, we just pass the default profile `rmw_qos_profile_default`
(it's global because it's declared in `rmw`, which is written in C and so
doesn't have namespaces).

~~~
//  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
//  ros::Rate loop_rate(10);
  auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter",
    rmw_qos_profile_default);
  rclcpp::rate::Rate loop_rate(10);
~~~

The creation of the outgoing message is different in both the namespace and the
fact that we go ahead and create a shared pointer (this may change in the future
with more publish API that accepts const references):

~~~
//  std_msgs::String msg;
  auto msg = std::make_shared<std_msgs::msg::String>();
~~~

In place of `ros::ok()`, we call `rclcpp::ok()`:

~~~
//  while (ros::ok())
  while (rclcpp::ok())
~~~

Inside the publishing loop, we use the `->` operator to access the `data` field
(because now `msg` is a shared pointer):

~~~
//    msg.data = ss.str();
    msg->data = ss.str();
~~~

To print a console message, instead of using `ROS_INFO()`, we use `printf()`
(this is temporary, because we don't yet have an equivalent of the `rosconsole`
package):

~~~
//    ROS_INFO("%s", msg.data.c_str());
    printf("%s\n", msg->data.c_str());
~~~

Publishing the message is very similar, the only noticeable difference being
that the publisher is now a shared pointer:

~~~
//    chatter_pub.publish(msg);
    chatter_pub->publish(msg);
~~~

Spinning (i.e., letting the communications system process any pending
incoming/outgoing messages) is different in that the call now takes the node as
an argument:

~~~
//    ros::spinOnce();
    rclcpp::spin_some(node);
~~~

Sleeping using the rate object is unchanged.

Putting it all together, the new `talker.cpp` looks like this:

~~~
{% include 200_migration_guide_from_ros1/ros2/src/talker/talker.cpp %}~~~

#### Changing the `package.xml`

Starting with ROS 2, only version 2 of the `package.xml` format is supported
(this format is also supported in ROS 1, but isn't used by all packages).
We start by specifying the format version in the `package` tag:

~~~
<!-- <package> -->
<package format="2">
~~~

ROS 2 uses a newer version of `catkin`, called `ament`, which we specify in the
`buildtool_depend` tag:

~~~
<!--  <buildtool_depend>catkin</buildtool_depend> -->
  <buildtool_depend>ament_cmake</buildtool_depend>
~~~

In our build dependencies, instead of `roscpp` we use `rclcpp`, which provides
the C++ API that we use.
We additionally depend on `rmw_implementation`, which pulls in the default
implementation of the `rmw` abstraction layer that allows us to support multiple
DDS implementations (we should consider restructuring / renaming things so that
it's possible to depend on one thing, analogous to `roscpp`):

~~~
<!--  <build_depend>roscpp</build_depend> -->
  <build_depend>rclcpp</build_depend>
  <build_depend>rmw_implementation</build_depend>
~~~

We make the same addition in the run dependencies and also update from the
`run_depend` tag to the `exec_depend` tag (part of the upgrade to version 2 of
the package format):

~~~
<!--  <run_depend>roscpp</run_depend> -->
  <exec_depend>rclcpp</exec_depend>
  <exec_depend>rmw_implementation</exec_depend>
<!--  <run_depend>std_msgs</run_depend> -->
  <exec_depend>std_msgs</exec_depend>
~~~

We also need to tell `ament` what *kind* of package we are, so that it knows how
to build us.
Because we're using `ament` and CMake, we add the following lines to declare our
build type to be `ament_cmake`:

~~~
  <export>
    <build_type>ament_cmake</build_type>
  </export>
~~~


Putting it all together, our `package.xml` now looks like this:

~~~
{% include 200_migration_guide_from_ros1/ros2/src/talker/package.xml %}~~~

*TODO: show simpler version of this file just using the `<depend>` tag, which is
enabled by version 2 of the package format (also supported in `catkin` so,
strictly speaking, orthogonal to ROS 2).*

#### Changing the CMake code

ROS 2 relies on the C++11 standard.
Depending on what compiler you're using, support for C++11 might not be enabled
by default.
Using `gcc` 4.8 (which is what is used on Ubuntu Trusty), we need to enable it
explicitly, which we do by adding this line near the top of the file:

~~~
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
~~~

Using `catkin`, we specify the packages we want to build against by passing them
as `COMPONENTS` arguments when initially finding `catkin` itself.
With `ament`, we find each package individually, starting with `ament_cmake`
(and adding our new dependency, `rmw_implementation`):

~~~
#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
~~~

We call `catkin_package()` to auto-generate things like CMake configuration
files for other packages that use our package.
Whereas that call happens *before* specifying targets to build, we now call the
analogous `ament_package()` *after* the targets:

~~~
#catkin_package()
# At the bottom of the file:
ament_package()
~~~

Similarly to how we found each dependent package separately, instead of finding
them as parts of catkin, we also need to add their include directories
separately (see also `ament_target_dependencies()` below, which is a more
concise and more thorough way of handling dependent packages' build flags):

~~~
#include_directories(${catkin_INCLUDE_DIRS})
include_directories(${rclcpp_INCLUDE_DIRS}
                    ${rmw_implementation_INCLUDE_DIRS}
                    ${std_msgs_INCLUDE_DIRS})
~~~

We do the same to link against our dependent packages' libraries:

~~~
#target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(talker
                      ${rclcpp_LIBRARIES}
                      ${rmw_implementation_LIBRARIES}
                      ${std_msgs_LIBRARIES})
~~~

*TODO: explain how `ament_target_dependencies()` simplifies the above steps and
is also better (also handling `*_DEFINITIONS`, doing target-specific include
directories, etc.).*

For installation, `catkin` defines variables like
`CATKIN_PACKAGE_BIN_DESTINATION`.
With `ament`, we just give a path relative to the installation root, like `bin`
for executables (this is in part because we don't yet have an equivalent of
`rosrun`):

~~~
#install(TARGETS talker
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS talker RUNTIME DESTINATION bin)
~~~

Putting it all together, the new `CMakeLists.txt` looks like this:

~~~
{% include 200_migration_guide_from_ros1/ros2/src/talker/CMakeLists.txt %}~~~

*TODO: Show what this would look like with `ament_auto`.*

#### Building the ROS 2 code

We source an environment setup file (in this case the one generated by following
the ROS 2 installation tutorial, which builds in `~/ros2_ws`, then we build our
package using `ament build`:

~~~
. ~/ros2_ws/install/setup.bash
cd ~/ros2_talker
ament build
~~~

#### Running the ROS 2 node

Because we installed the `talker` executable into `bin`, after sourcing the
setup file, from our `ament` install tree, we can invoke it by name directly
(also, there is not yet a ROS 2 equivalent for `rosrun`):

~~~
. ~/ros2_ws/install/setup.bash
talker
~~~
