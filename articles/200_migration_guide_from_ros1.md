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

### Package naming

Packages containing messages and / or services which should be supported by the [ROS bridge](https://github.com/ros2/ros1_bridge/) need to have a different name in ROS 1 and ROS 2.
Therefore the suffixes `_msgs` and `_srvs` are replaced (and merged in a single package) with the suffix `_interfaces`.


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
